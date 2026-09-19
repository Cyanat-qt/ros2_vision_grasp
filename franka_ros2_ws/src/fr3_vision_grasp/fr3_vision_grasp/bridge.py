"""MuJoCo viewer, RGB-D publisher and standard ROS control action servers."""
import json
import math
import threading
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time, Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster
from .core import ARM, FINGERS, seconds, sample_trajectory, validate_trajectory
from .simulation import Simulation


def stamp(t):
    ns = round(t * 1e9)
    return Time(sec=ns // 1000000000, nanosec=ns % 1000000000)


class Bridge(Node):
    def __init__(self):
        super().__init__('fr3_mujoco_bridge')
        default = str(Path(get_package_share_directory('fr3_vision_grasp')) / 'scene/scene_grasp.xml')
        for name, value in [('scene', default), ('viewer', True), ('width', 640),
                            ('height', 480)]:
            self.declare_parameter(name, value)
        self.declare_parameter('camera_rate', 15.0, ParameterDescriptor(dynamic_typing=True))
        camera_rate = self.get_parameter('camera_rate').value
        if isinstance(camera_rate, bool) or not isinstance(camera_rate, (int, float)) or not math.isfinite(camera_rate) or camera_rate <= 0:
            raise ValueError('camera_rate must be a positive number')
        self.sim = Simulation(self.get_parameter('scene').value,
                              self.get_parameter('width').value, self.get_parameter('height').value)
        self.lock = threading.RLock()
        self.arm_busy = False
        self.hand_busy = False
        self.alive = True
        self.cv = CvBridge()
        self.state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', qos_profile_sensor_data)
        # Evaluation only. Perception and task target selection never consume object truth.
        self.metrics_pub = self.create_publisher(String, '/simulation/grasp_metrics', 10)
        self.tf = StaticTransformBroadcaster(self)
        self.publish_transforms()
        group = ReentrantCallbackGroup()
        self.arm_server = ActionServer(self, FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory', self.execute_arm,
            goal_callback=self.arm_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group)
        self.hand_server = ActionServer(self, GripperCommand, '/franka_gripper/gripper_action',
            self.execute_hand, goal_callback=self.hand_goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.create_service(Trigger, '/simulation/reset', self.reset, callback_group=group)
        self.get_logger().info('MuJoCo bridge ready: RGB-D, joint states, arm and gripper actions')

    def publish_transforms(self):
        transforms = []
        # Existing URDF root is base; its fr3_link0 is coincident with world.
        for parent, child in [('world', 'base'), ('world', 'camera_optical_frame')]:
            t = TransformStamped()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.rotation.w = 1.0
            if child == 'camera_optical_frame':
                p, q = self.sim.camera_pose()
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
                t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z = map(float, q)
            transforms.append(t)
        self.tf.sendTransform(transforms)

    def reset(self, request, response):
        with self.lock:
            if self.arm_busy or self.hand_busy:
                response.message = 'An action is running; cancel it before resetting.'
                return response
            # Keep time monotonic so TF and perception caches cannot use old timestamps.
            elapsed = self.sim.data.time
            self.sim.reset()
            self.sim.data.time = elapsed
        response.success, response.message = True, 'Simulation reset; restart pick to rebuild its planning scene.'
        return response

    def arm_goal(self, request):
        with self.lock:
            if self.arm_busy:
                return GoalResponse.REJECT
            try:
                validate_trajectory(request.trajectory.joint_names, request.trajectory.points, self.sim.limits)
                if seconds(request.trajectory.header.stamp) > 0:
                    raise ValueError('Use a zero header stamp (start immediately)')
                for tolerance in [*request.path_tolerance, *request.goal_tolerance]:
                    if tolerance.name not in ARM or tolerance.velocity not in (0, -1) or tolerance.acceleration not in (0, -1):
                        raise ValueError('Only named position tolerances are supported')
                    if not math.isfinite(tolerance.position) or tolerance.position < -1:
                        raise ValueError('Invalid position tolerance')
            except ValueError as exc:
                self.get_logger().warning(str(exc))
                return GoalResponse.REJECT
            self.arm_busy = True
        return GoalResponse.ACCEPT

    def hand_goal(self, request):
        command = request.command
        with self.lock:
            if (self.hand_busy or not math.isfinite(command.position) or
                    not 0 <= command.position <= 0.04 or
                    not math.isfinite(command.max_effort) or command.max_effort < 0):
                return GoalResponse.REJECT
            self.hand_busy = True
        return GoalResponse.ACCEPT

    def execute_arm(self, goal):
        result = FollowJointTrajectory.Result()
        succeeded = False
        try:
            request = goal.request
            trajectory = request.trajectory
            order = validate_trajectory(trajectory.joint_names, trajectory.points, self.sim.limits)
            times = [seconds(p.time_from_start) for p in trajectory.points]
            positions = [np.array(p.positions)[order] for p in trajectory.points]
            velocities = ([np.array(p.velocities)[order] for p in trajectory.points]
                          if all(len(p.velocities) == 7 for p in trajectory.points) else None)
            with self.lock:
                initial = self.sim.data.qpos[self.sim.qadr].copy()
                start = self.sim.data.time
            if times[0] > 0:
                times.insert(0, 0.0)
                positions.insert(0, initial)
                if velocities is not None:
                    velocities.insert(0, np.zeros(7))
            elif np.max(np.abs(initial - positions[0])) > 0.03:
                raise ValueError('Trajectory start differs from measured position by over 0.03 rad')
            path_tol, goal_tol = np.full(7, 0.15), np.full(7, 0.015)
            for values, dest in [(request.path_tolerance, path_tol), (request.goal_tolerance, goal_tol)]:
                for item in values:
                    if item.position:
                        dest[ARM.index(item.name)] = np.inf if item.position == -1 else item.position
            allowance = seconds(request.goal_time_tolerance) or 2.0
            wall_deadline = time.monotonic() + max(30, 6 * (times[-1] + allowance))
            while self.alive and rclpy.ok():
                if goal.is_cancel_requested:
                    goal.canceled()
                    result.error_string = 'Canceled; holding measured position'
                    return result
                with self.lock:
                    elapsed = self.sim.data.time - start
                    target = sample_trajectory(times, positions, velocities, elapsed)
                    if np.any(target < self.sim.limits[:, 0]) or np.any(target > self.sim.limits[:, 1]):
                        raise ValueError('Interpolated trajectory exceeds a joint limit')
                    self.sim.data.ctrl[self.sim.actuators] = target
                    actual = self.sim.data.qpos[self.sim.qadr].copy()
                    speed = self.sim.data.qvel[self.sim.dadr].copy()
                error = target - actual
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = stamp(start + elapsed)
                feedback.joint_names = ARM
                feedback.desired.positions = target.tolist()
                feedback.actual.positions = actual.tolist()
                feedback.actual.velocities = speed.tolist()
                feedback.error.positions = error.tolist()
                feedback.desired.time_from_start = Duration(sec=int(elapsed), nanosec=int(elapsed % 1 * 1e9))
                goal.publish_feedback(feedback)
                if elapsed >= times[-1] and np.all(np.abs(error) <= goal_tol):
                    goal.succeed()
                    succeeded = True
                    result.error_code = result.SUCCESSFUL
                    return result
                if elapsed < times[-1] and np.any(np.abs(error) > path_tol):
                    result.error_code = result.PATH_TOLERANCE_VIOLATED
                    raise RuntimeError('Measured trajectory tracking error exceeds path tolerance')
                if elapsed > times[-1] + allowance or time.monotonic() > wall_deadline:
                    result.error_code = result.GOAL_TOLERANCE_VIOLATED
                    raise RuntimeError('Trajectory execution timed out')
                time.sleep(0.01)
            raise RuntimeError('Simulation stopped')
        except Exception as exc:
            result.error_string = str(exc)
            if not result.error_code:
                result.error_code = result.INVALID_GOAL
            if goal.is_active:
                goal.abort()
            self.get_logger().error(result.error_string)
            return result
        finally:
            with self.lock:
                if not succeeded:
                    self.sim.data.ctrl[self.sim.actuators] = self.sim.data.qpos[self.sim.qadr]
                self.arm_busy = False

    def execute_hand(self, goal):
        result = GripperCommand.Result()
        succeeded = False
        with self.lock:
            start = self.sim.data.time
            initial = float(np.mean(self.sim.data.qpos[self.sim.fqadr]))
            old_limit = self.sim.model.actuator_forcerange[self.sim.gripper].copy()
            force = min(goal.request.command.max_effort or 20.0, 20.0)
            self.sim.model.actuator_forcerange[self.sim.gripper] = [-force, force]
        target = goal.request.command.position  # Single finger travel, NOT total width.
        stable_since = None
        wall_deadline = time.monotonic() + 40
        try:
            while self.alive and rclpy.ok():
                with self.lock:
                    now = self.sim.data.time
                    duration = max(0.5, abs(target - initial) / 0.035)
                    self.sim.data.ctrl[self.sim.gripper] = initial + min(1., (now-start)/duration)*(target-initial)
                    position = float(np.mean(self.sim.data.qpos[self.sim.fqadr]))
                    velocity = float(np.max(np.abs(self.sim.data.qvel[self.sim.fdadr])))
                    effort = abs(float(self.sim.data.actuator_force[self.sim.gripper]))
                reached = abs(position - target) < 0.0015
                stalled = now-start > duration and velocity < 0.002 and not reached
                stable_since = now if (reached or stalled) and stable_since is None else stable_since
                if not reached and not stalled:
                    stable_since = None
                result.position, result.effort = position, effort
                result.reached_goal, result.stalled = reached, stalled
                feedback = GripperCommand.Feedback(position=position, effort=effort,
                                                   stalled=stalled, reached_goal=reached)
                goal.publish_feedback(feedback)
                if goal.is_cancel_requested:
                    goal.canceled()
                    return result
                if stable_since is not None and now - stable_since >= 0.3:
                    # Stalling is a normal contact outcome. The task verifies actual lifting.
                    succeeded = True
                    goal.succeed()
                    return result
                if now-start > 6 or time.monotonic() > wall_deadline:
                    goal.abort()
                    return result
                time.sleep(0.02)
            if goal.is_active:
                goal.abort()
            return result
        finally:
            with self.lock:
                if not succeeded:
                    self.sim.data.ctrl[self.sim.gripper] = np.mean(self.sim.data.qpos[self.sim.fqadr])
                # Keep requested force limit while holding a successful grasp.
                if not succeeded:
                    self.sim.model.actuator_forcerange[self.sim.gripper] = old_limit
                self.hand_busy = False

    def publish_state(self):
        s, d = self.sim, self.sim.data
        header = stamp(d.time)
        self.clock_pub.publish(Clock(clock=header))
        state = JointState()
        state.header.stamp, state.name = header, ARM + FINGERS
        state.position = [*d.qpos[s.qadr].tolist(), *d.qpos[s.fqadr].tolist()]
        state.velocity = [*d.qvel[s.dadr].tolist(), *d.qvel[s.fdadr].tolist()]
        self.state_pub.publish(state)

    def publish_images(self):
        rgb, depth = self.sim.render(self.lock)
        header = stamp(self.sim.frame_time)
        for array, encoding, publisher in [(rgb, 'rgb8', self.rgb_pub), (depth, '32FC1', self.depth_pub)]:
            msg = self.cv.cv2_to_imgmsg(array, encoding)
            msg.header.stamp, msg.header.frame_id = header, 'camera_optical_frame'
            publisher.publish(msg)
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = header, 'camera_optical_frame'
        info.width, info.height = self.sim.width, self.sim.height
        info.distortion_model, info.d = 'plumb_bob', [0.] * 5
        info.k = self.sim.k.ravel().tolist()
        info.r = np.eye(3).ravel().tolist()
        info.p = np.column_stack((self.sim.k, np.zeros(3))).ravel().tolist()
        self.info_pub.publish(info)
        self.depth_info_pub.publish(info)

    def publish_metrics(self):
        metrics = {'sim_time': self.sim.data.time, 'objects': {}}
        for color in ('red', 'green', 'blue'):
            name = color + '_object'
            try:
                body = self.sim.model.body(name).id
            except KeyError:
                continue
            metrics['objects'][color] = {'position': self.sim.data.xpos[body].tolist(),
                                          'contacts': self.sim.contacts(name)}
        self.metrics_pub.publish(String(data=json.dumps(metrics)))

    def run(self):
        viewer = None
        if self.get_parameter('viewer').value:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(self.sim.model, self.sim.data)
            viewer.cam.lookat[:] = [0.45, 0, 0.25]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 1.9, 135, -30
        last_viewer, last_metrics = -1., -1.
        rate = float(self.get_parameter('camera_rate').value)
        camera_stop = threading.Event()
        def camera_loop():
            try:
                while self.alive and rclpy.ok() and not camera_stop.is_set():
                    started = time.monotonic()
                    self.publish_images()
                    camera_stop.wait(max(0., 1 / rate - (time.monotonic() - started)))
            except Exception as exc:
                self.get_logger().error(f'RGB-D renderer failed: {exc}')
                self.alive = False
            finally:
                self.sim.close()
        camera_thread = threading.Thread(target=camera_loop, name='rgbd-renderer')
        camera_thread.start()
        next_tick = time.monotonic()
        try:
            while rclpy.ok() and self.alive and (viewer is None or viewer.is_running()):
                next_tick += 0.01
                with self.lock:
                    for _ in range(max(1, round(0.01 / self.sim.model.opt.timestep))):
                        self.sim.step()
                    self.publish_state()
                    now = self.sim.data.time
                    if now - last_metrics >= 0.1:
                        self.publish_metrics()
                        last_metrics = now
                    if viewer is not None and now - last_viewer >= 1 / 30:
                        viewer.sync()
                        last_viewer = now
                # Fixed wall schedule catches up small scheduling/render delays.
                # Cap catch-up after a long pause instead of jumping physics time.
                if next_tick < time.monotonic() - 0.25:
                    next_tick = time.monotonic()
                time.sleep(max(0, next_tick - time.monotonic()))
        finally:
            self.alive = False
            camera_stop.set()
            camera_thread.join()
            if viewer is not None:
                viewer.close()


def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.alive = False
        executor.shutdown(timeout_sec=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
