#!/usr/bin/env python3
"""Run against an isolated demo and verify physical grasp, transport and placement.

Source ROS and the workspace first. --launch starts and owns a fresh demo.
Simulation object truth is consumed only by this evaluator, never as a target.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, CameraInfo, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from fr3_vision_grasp.core import ARM, HOME


class Evaluator(Node):
    def __init__(self):
        super().__init__('fr3_demo_evaluator')
        self.latest = {}
        self.counts = {}
        self.events = []
        self.subs = []
        self.carry_samples = {c: [] for c in ('red', 'green', 'blue')}
        self.home_visits = []
        topics = [(Image, '/camera/color/image_raw', 'rgb'),
                  (Image, '/camera/depth/image_raw', 'depth'),
                  (CameraInfo, '/camera/color/camera_info', 'info'),
                  (Image, '/perception/annotated_image', 'annotated'),
                  (JointState, '/joint_states', 'joints'),
                  (String, '/simulation/grasp_metrics', 'metrics')]
        topics += [(PoseStamped, f'/perception/{c}/pose', c) for c in ('red', 'green', 'blue')]
        for kind, topic, key in topics:
            self.subs.append(self.create_subscription(kind, topic,
                lambda msg, key=key: self.receive(key, msg), qos_profile_sensor_data))
        self.subs.append(self.create_subscription(String, '/pick/status', self.on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))

    def receive(self, key, msg):
        self.latest[key] = json.loads(msg.data) if key == 'metrics' else msg
        self.counts[key] = self.counts.get(key, 0) + 1
        status = self.latest.get('status', {})
        if key == 'metrics' and status.get('state') == 'TRANSPORTING':
            color = status['color']
            self.carry_samples[color].append(self.latest[key]['objects'][color])

    def on_status(self, msg):
        status = json.loads(msg.data)
        self.latest['status'] = status
        self.events.append(status)
        # DETECTING persists while collecting fresh frames at home. Very brief
        # intermediate statuses can be coalesced by the depth-1 status topic.
        if status['state'] == 'DETECTING' and 'joints' in self.latest:
            joints = self.latest['joints']
            positions = np.array([joints.position[joints.name.index(n)] for n in ARM])
            self.home_visits.append({'placed_colors': status['placed_colors'],
                                    'max_joint_error_rad': float(np.max(np.abs(positions - HOME)))})
        print(status['state'] + ': ' + status['message'], flush=True)

    def until(self, predicate, timeout=30):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('Timed out waiting for ROS data/result')
            rclpy.spin_once(self, timeout_sec=0.1)

    def service(self, name):
        client = self.create_client(Trigger, name)
        try:
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError(name + ' unavailable')
            future = client.call_async(Trigger.Request())
            self.until(future.done, 20)
            result = future.result()
            if not result.success:
                raise RuntimeError(result.message)
        finally:
            self.destroy_client(client)

    def capture(self, directory, prefix):
        cv = CvBridge()
        for key in ('rgb', 'annotated'):
            cv2.imwrite(str(directory / f'{prefix}_{key}.png'),
                        cv.imgmsg_to_cv2(self.latest[key], 'bgr8'))

    def trial(self, directory, index, color):
        if index:
            self.latest.pop('status', None)
            self.service('/pick/reset')
            self.until(lambda: self.latest.get('status', {}).get('state') in ('READY', 'FAILED'))
            assert self.latest['status']['state'] == 'READY', self.latest['status']
            for c in ('red', 'green', 'blue'):
                self.latest.pop(c, None)
        self.until(lambda: all(k in self.latest for k in
            ('rgb', 'depth', 'info', 'annotated', 'joints', 'metrics', 'red', 'green', 'blue')), 60)
        started = time.monotonic()
        first_count = self.counts.copy()
        sim_start = self.latest['metrics']['sim_time']
        self.until(lambda: time.monotonic() - started >= 4)
        wall_duration = time.monotonic() - started
        rates = {k: (v - first_count.get(k, 0)) / wall_duration for k, v in self.counts.items()}
        errors = {}
        for c in ('red', 'green', 'blue'):
            pose = self.latest[c]
            assert pose.header.frame_id == 'fr3_link0'
            p = pose.pose.position
            truth = self.latest['metrics']['objects'][c]['position']
            error = float(np.linalg.norm(np.array([p.x, p.y, p.z]) - truth))
            assert error < 0.005, (c, error)
            errors[c] = error
        assert self.latest['rgb'].encoding == 'rgb8'
        assert self.latest['depth'].encoding == '32FC1'
        assert len(self.latest['joints'].position) == 9
        self.capture(directory, f'trial_{index}_before')
        initial_z = {c: obj['position'][2] for c, obj in self.latest['metrics']['objects'].items()}
        report = {'vision_error_m': errors, 'wall_topic_rates_hz': rates,
                  'simulation_speed': (self.latest['metrics']['sim_time'] - sim_start) / wall_duration}
        self.events.clear()
        for samples in self.carry_samples.values():
            samples.clear()
        self.home_visits.clear()
        self.latest.pop('status', None)
        self.service('/pick/start')
        self.until(lambda: self.latest.get('status', {}).get('state') in
                   ('SUCCEEDED', 'FAILED', 'CANCELED'), 900)
        assert self.latest['status']['state'] == 'SUCCEEDED', self.latest['status']
        order = [color, *[c for c in ('red', 'green', 'blue') if c != color]]
        assert self.latest['status']['placed_colors'] == order
        assert len(self.home_visits) == 4, self.home_visits
        for count, visit in enumerate(self.home_visits):
            assert visit['placed_colors'] == order[:count], visit
            assert visit['max_joint_error_rad'] < 0.02, visit
        min_lift = {}
        for c, samples in self.carry_samples.items():
            assert samples, f'No transport measurements for {c}'
            min_lift[c] = min(obj['position'][2] - initial_z[c] for obj in samples)
            assert min_lift[c] > 0.05, (c, min_lift[c])
            assert all(len(obj['contacts']) == 2 for obj in samples), f'Lost {c} during transport'
        expected_xy = {'red': [0.35, -0.42], 'green': [0.35, 0.42], 'blue': [0.60, 0.42]}
        # All three cubes remain placed, while the arm stays at HOME after success.
        start_sim = self.latest['metrics']['sim_time']
        positions = {c: [] for c in order}
        joint_positions = []
        event_count = len(self.events)
        deadline = time.monotonic() + 20
        while self.latest['metrics']['sim_time'] - start_sim < 2:
            assert time.monotonic() < deadline, 'Simulation stalled after placement'
            rclpy.spin_once(self, timeout_sec=0.1)
            for c in order:
                obj = self.latest['metrics']['objects'][c]
                assert not obj['contacts'], (c, obj)
                assert np.max(np.abs(np.array(obj['position'][:2]) - expected_xy[c])) < 0.045, (c, obj)
                assert abs(obj['position'][2] - 0.072) < 0.004, (c, obj)
                positions[c].append(obj['position'])
            joints = self.latest['joints']
            actual = np.array([joints.position[joints.name.index(n)] for n in ARM])
            assert np.max(np.abs(actual - HOME)) < 0.02, actual
            joint_positions.append(actual)
        for c, samples in positions.items():
            assert np.max(np.ptp(samples, axis=0)) < 0.001, f'{c} is still moving'
        assert np.max(np.ptp(joint_positions, axis=0)) < 0.001, 'Robot moved after sorting finished'
        assert len(self.events) == event_count, 'Task restarted after finishing'
        states = [event['state'] for event in self.events]
        for state in ('LIFTING', 'TRANSPORTING', 'PLACING', 'RELEASING', 'RETREATING', 'VERIFYING'):
            assert state in states, states
        self.capture(directory, f'trial_{index}_after')
        report.update(minimum_lift_m=min_lift, final_objects=self.latest['metrics']['objects'],
                      home_visits=self.home_visits.copy(), final_joint_positions=joint_positions[-1].tolist(),
                      states=self.events.copy(), success=True)
        return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch', action='store_true')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--color', choices=['red', 'green', 'blue'], default='red')
    parser.add_argument('--trials', type=int, default=2)
    parser.add_argument('--output', type=Path, default=Path('validation_results'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    report = {'mujoco': mujoco.__version__, 'color': args.color, 'trials': []}
    process = log = node = None
    rclpy.init()
    try:
        if args.launch:
            log = (args.output / 'launch.log').open('w')
            gui = str(args.gui).lower()
            process = subprocess.Popen(['ros2', 'launch', 'fr3_vision_grasp', 'fr3_sim.launch.py',
                f'viewer:={gui}', f'use_rviz:={gui}', f'camera_view:={gui}', f'color:={args.color}'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        node = Evaluator()
        for index in range(args.trials):
            report['trials'].append(node.trial(args.output, index, args.color))
        report['success'] = True
        print(json.dumps(report, indent=2), flush=True)
    except Exception as exc:
        report['success'], report['error'] = False, str(exc)
        if node:
            report['last_status'] = node.latest.get('status')
        raise
    finally:
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        # SIGINT can already have shut down rclpy. Always stop the owned launch
        # before ROS cleanup so interrupted checks cannot leave duplicate robots.
        try:
            if process is not None:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                log.close()
        finally:
            if node:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
