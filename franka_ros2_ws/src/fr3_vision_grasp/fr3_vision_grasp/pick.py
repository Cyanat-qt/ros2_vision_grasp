"""Perception-driven MoveIt pick, with separate simulation-only success evaluation."""
import copy
import json
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, MotionPlanRequest, PlanningOptions, PlanningScene, CollisionObject,
    AttachedCollisionObject, AllowedCollisionEntry, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class Pick(Node):
    def __init__(self):
        super().__init__('fr3_pick_place')
        for name, value in [('color', 'red'), ('object_size', 0.03), ('approach_height', 0.12),
                            ('lift_height', 0.10), ('verify_simulation', True)]:
            self.declare_parameter(name, value)
        self.color = self.get_parameter('color').value
        if self.color not in ('red', 'green', 'blue'):
            raise ValueError('color must be red, green or blue')
        self.size = self.get_parameter('object_size').value
        self.lock = threading.Lock()
        self.busy = False
        self.holding = False
        self.running = True
        self.samples = {color: deque(maxlen=8) for color in ('red', 'green', 'blue')}
        self.metrics = None
        self.metric_wall = 0.0
        self.active_goal = None
        self.stop_requested = threading.Event()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.gripper = ActionClient(self, GripperCommand, '/franka_gripper/gripper_action')
        self.apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.reset_sim = self.create_client(Trigger, '/simulation/reset')
        self.subs = [self.create_subscription(PoseStamped, f'/perception/{c}/pose',
                    lambda msg, c=c: self.observe(c, msg), 10) for c in self.samples]
        self.create_subscription(String, '/simulation/grasp_metrics', self.on_metrics, 10)
        self.create_subscription(String, '/perception/pick_request', self.on_request, 10)
        self.create_service(Trigger, '/pick/start', self.start)
        self.create_service(Trigger, '/pick/cancel', self.cancel)
        self.create_service(Trigger, '/pick/reset', self.reset)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/pick/status', qos)
        self.worker = None
        self.set_status('READY', 'Waiting for stable detections and /pick/start')

    def set_status(self, state, message):
        self.status.publish(String(data=json.dumps({'state': state, 'message': message, 'color': self.color})))
        self.get_logger().info(f'{state}: {message}')

    def observe(self, color, msg):
        if msg.header.frame_id == 'fr3_link0':
            self.samples[color].append(copy.deepcopy(msg))

    def on_metrics(self, msg):
        self.metrics = json.loads(msg.data)
        self.metric_wall = time.monotonic()

    def on_request(self, msg):
        if msg.data.strip().lower() == 'pick':
            self.start(Trigger.Request(), Trigger.Response())

    def start(self, request, response):
        with self.lock:
            if self.busy or self.holding:
                response.message = 'Busy or holding an object. Use /pick/reset before another trial.'
                return response
            self.busy = True
            self.stop_requested.clear()
        self.worker = threading.Thread(target=self.run_pick, daemon=True)
        self.worker.start()
        response.success, response.message = True, 'Pick started; monitor /pick/status'
        return response

    def cancel(self, request, response):
        self.stop_requested.set()
        if self.active_goal is not None:
            self.active_goal.cancel_goal_async()
        response.success, response.message = True, 'Cancel requested; arm holds and gripper remains at its current target'
        return response

    def reset(self, request, response):
        with self.lock:
            if self.busy:
                response.message = 'Cancel and wait for completion first'
                return response
            self.busy = True
        self.stop_requested.clear()
        def run():
            try:
                scene = PlanningScene(is_diff=True)
                scene.robot_state.is_diff = True
                scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                    object=CollisionObject(id='target_' + self.color, operation=CollisionObject.REMOVE))]
                self.apply(scene)
                self.allow_touch(False)
                self.allow_touch(False, ['table_top'])
                if not self.reset_sim.wait_for_service(timeout_sec=5):
                    raise RuntimeError('Simulation reset service unavailable')
                result = self.wait(self.reset_sim.call_async(Trigger.Request()), 8)
                if not result.success:
                    raise RuntimeError(result.message)
                for samples in self.samples.values():
                    samples.clear()
                self.holding = False
                self.set_status('READY', 'Simulation reset; waiting for new images')
            except Exception as exc:
                self.set_status('FAILED', str(exc))
            finally:
                self.busy = False
        self.worker = threading.Thread(target=run, daemon=True)
        self.worker.start()
        response.success, response.message = True, 'Reset started'
        return response

    def wait(self, future, timeout):
        deadline = time.monotonic() + timeout
        while not future.done():
            if not self.running or self.stop_requested.is_set():
                raise RuntimeError('Task canceled')
            if time.monotonic() >= deadline:
                raise RuntimeError('ROS request timed out')
            time.sleep(0.01)
        return future.result()

    def action(self, client, request, timeout=90):
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError(f'Action server unavailable: {client._action_name}')
        handle = self.wait(client.send_goal_async(request), 10)
        if handle is None or not handle.accepted:
            raise RuntimeError('Action goal rejected')
        self.active_goal = handle
        try:
            wrapped = self.wait(handle.get_result_async(), timeout)
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'Action failed with status {wrapped.status}: {wrapped.result}')
            return wrapped.result
        except Exception:
            handle.cancel_goal_async()
            raise
        finally:
            self.active_goal = None

    def move_to(self, position, linear=False):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, position)
        # Tool +Z points down, fingers close parallel to world Y for axis-aligned cubes.
        pose.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        if linear:
            if not self.cartesian.wait_for_service(timeout_sec=10):
                raise RuntimeError('MoveIt Cartesian path service unavailable')
            request = GetCartesianPath.Request(group_name='fr3_arm', link_name='fr3_hand_tcp',
                waypoints=[pose], max_step=0.005, jump_threshold=2.0, avoid_collisions=True)
            request.header.frame_id = 'fr3_link0'
            request.start_state.is_diff = True
            # Older Humble moveit_msgs revisions lack these optional fields.
            has_scaling = hasattr(request, 'max_velocity_scaling_factor')
            if has_scaling:
                request.max_velocity_scaling_factor = 0.15
                request.max_acceleration_scaling_factor = 0.15
            plan = self.wait(self.cartesian.call_async(request), 15)
            if plan.error_code.val != plan.error_code.SUCCESS or plan.fraction < 0.999:
                raise RuntimeError(f'Incomplete collision-free Cartesian path: {plan.fraction:.1%}')
            trajectory = plan.solution
            if len(trajectory.joint_trajectory.points) < 2:
                raise RuntimeError('Cartesian planner returned no motion')
            if not has_scaling:
                from builtin_interfaces.msg import Duration
                for point in trajectory.joint_trajectory.points:
                    ns = round((point.time_from_start.sec * 1000000000 + point.time_from_start.nanosec) / 0.15)
                    point.time_from_start = Duration(sec=ns // 1000000000, nanosec=ns % 1000000000)
                    point.velocities = [v * 0.15 for v in point.velocities]
                    point.accelerations = [a * 0.15**2 for a in point.accelerations]
            result = self.action(self.execute, ExecuteTrajectory.Goal(trajectory=trajectory), 120)
            if result.error_code.val != result.error_code.SUCCESS:
                raise RuntimeError(f'MoveIt Cartesian execution error {result.error_code.val}')
            return
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = 'fr3_link0'
        position_constraint.link_name, position_constraint.weight = 'fr3_hand_tcp', 1.0
        position_constraint.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.002])],
            primitive_poses=[pose])
        orientation = OrientationConstraint()
        orientation.header.frame_id, orientation.link_name = 'fr3_link0', 'fr3_hand_tcp'
        orientation.orientation = pose.orientation
        orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = 0.015
        orientation.absolute_z_axis_tolerance = 0.025
        orientation.weight = 1.0
        constraints = Constraints(position_constraints=[position_constraint], orientation_constraints=[orientation])
        request = MotionPlanRequest(group_name='fr3_arm', pipeline_id='ompl',
            num_planning_attempts=10, allowed_planning_time=10., goal_constraints=[constraints],
            max_velocity_scaling_factor=0.25, max_acceleration_scaling_factor=0.25)
        request.start_state.is_diff = True
        result = self.action(self.move, MoveGroup.Goal(request=request,
            planning_options=PlanningOptions(plan_only=False, replan=False)), 120)
        if result.error_code.val != result.error_code.SUCCESS:
            raise RuntimeError(f'MoveIt error {result.error_code.val}')

    def hand(self, width):
        goal = GripperCommand.Goal()
        goal.command.position, goal.command.max_effort = width / 2, 20.0
        return self.action(self.gripper, goal, 40)

    def apply(self, scene):
        if not self.apply_scene.wait_for_service(timeout_sec=10):
            raise RuntimeError('MoveIt planning scene service unavailable')
        response = self.wait(self.apply_scene.call_async(ApplyPlanningScene.Request(scene=scene)), 10)
        if not response.success:
            raise RuntimeError('Planning scene update rejected')

    def box(self, name, position, size):
        obj = CollisionObject(id=name, operation=CollisionObject.ADD)
        obj.header.frame_id = 'fr3_link0'
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, position)
        pose.orientation.w = 1.0
        obj.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size)))]
        obj.primitive_poses = [pose]
        return obj

    def populate_scene(self, targets):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        xml = Path(get_package_share_directory('fr3_vision_grasp')) / 'scene/scene_grasp.xml'
        # Known static environment only. Movable object poses come exclusively from perception.
        for body in ET.parse(xml).find('worldbody').findall('body'):
            if body.find('freejoint') is not None:
                continue
            origin = np.fromstring(body.get('pos', '0 0 0'), sep=' ')
            for geom in body.findall('geom'):
                if geom.get('type') == 'box':
                    pos = origin + np.fromstring(geom.get('pos', '0 0 0'), sep=' ')
                    size = 2*np.fromstring(geom.get('size'), sep=' ')
                    scene.world.collision_objects.append(self.box(geom.get('name'), pos, size))
        for color, target in targets.items():
            p = target.pose.position
            scene.world.collision_objects.append(self.box('target_'+color, [p.x, p.y, p.z], [self.size]*3))
        self.apply(scene)

    def allow_touch(self, allow, links=None):
        if not self.get_scene.wait_for_service(timeout_sec=10):
            raise RuntimeError('get_planning_scene unavailable')
        request = GetPlanningScene.Request(components=PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))
        matrix = self.wait(self.get_scene.call_async(request), 10).scene.allowed_collision_matrix
        names = ['target_'+self.color, *(links if links is not None else ['fr3_leftfinger', 'fr3_rightfinger'])]
        for name in names:
            if name not in matrix.entry_names:
                matrix.entry_names.append(name)
                for entry in matrix.entry_values:
                    entry.enabled.append(False)
                matrix.entry_values.append(AllowedCollisionEntry(enabled=[False]*len(matrix.entry_names)))
        i = matrix.entry_names.index(names[0])
        for name in names[1:]:
            j = matrix.entry_names.index(name)
            matrix.entry_values[i].enabled[j] = matrix.entry_values[j].enabled[i] = allow
        self.apply(PlanningScene(is_diff=True, allowed_collision_matrix=matrix))

    def attach(self, target):
        transform = self.tf.lookup_transform('fr3_hand_tcp', target.header.frame_id, Time())
        pose = do_transform_pose(target.pose, transform)
        attached = AttachedCollisionObject(link_name='fr3_hand_tcp',
            touch_links=['fr3_hand', 'fr3_leftfinger', 'fr3_rightfinger'])
        attached.object = self.box('target_'+self.color, [0, 0, 0], [self.size]*3)
        attached.object.header.frame_id = 'fr3_hand_tcp'
        attached.object.primitive_poses = [pose]
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [attached]
        # MoveIt removes the same-id world object when attaching. Sending an
        # additional REMOVE in this diff fails in Humble after that removal.
        self.apply(scene)

    def stable_targets(self):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if self.stop_requested.is_set():
                raise RuntimeError('Task canceled')
            valid = {}
            now = self.get_clock().now()
            for color, history in self.samples.items():
                messages = list(history)
                if len(messages) < 5 or (now-Time.from_msg(messages[-1].header.stamp)).nanoseconds > 1e9:
                    continue
                points = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in messages])
                if np.max(np.std(points, axis=0)) < 0.003:
                    valid[color] = copy.deepcopy(messages[-1])
            if self.color in valid:
                return valid
            time.sleep(0.05)
        raise RuntimeError('No fresh stable target detected in 15 seconds')

    def evaluate(self, minimum_z, duration=0.5, timeout=8):
        """Ground truth is used for evaluation only, never for target or trajectory generation."""
        deadline, stable = time.monotonic()+timeout, None
        while time.monotonic() < deadline:
            if self.stop_requested.is_set():
                raise RuntimeError('Task canceled')
            metrics = self.metrics
            obj = metrics.get('objects', {}).get(self.color) if metrics else None
            if obj and time.monotonic()-self.metric_wall < 2 and obj['position'][2] > minimum_z and len(obj['contacts']) == 2:
                if stable is None:
                    stable = metrics['sim_time']
                if metrics['sim_time']-stable >= duration:
                    return
            else:
                stable = None
            time.sleep(0.03)
        raise RuntimeError('Physical grasp verification failed: require two-finger contact and sustained object height')

    def run_pick(self):
        try:
            self.set_status('DETECTING', 'Waiting for stable RGB-D target')
            targets = self.stable_targets()
            target = targets[self.color]
            p = target.pose.position
            center = np.array([p.x, p.y, p.z])
            self.populate_scene(targets)
            self.allow_touch(False)
            self.set_status('OPENING', 'Opening gripper')
            opened = self.hand(0.08)
            if not opened.reached_goal:
                raise RuntimeError('Gripper failed to open fully')
            self.set_status('APPROACHING', f'Planning above visual target {center.round(4).tolist()}')
            self.move_to(center + [0, 0, self.get_parameter('approach_height').value])
            self.allow_touch(True)
            self.set_status('DESCENDING', 'Planning to grasp centre with downward tool orientation')
            self.move_to(center, linear=True)
            self.set_status('CLOSING', 'Closing until contact')
            closed = self.hand(0.0)
            if not closed.stalled or closed.position < 0.003:
                raise RuntimeError('Empty closure: no object held between fingers')
            if self.get_parameter('verify_simulation').value:
                self.evaluate(center[2]-0.015, duration=0.2)
            self.holding = True
            self.attach(target)
            # The grasped cube initially rests on its support surface. Permit
            # only this object/support pair during the straight upward retreat.
            self.allow_touch(True, ['table_top'])
            self.set_status('LIFTING', 'Planning upward motion with attached collision object')
            self.move_to(center + [0, 0, self.get_parameter('lift_height').value], linear=True)
            if self.get_parameter('verify_simulation').value:
                self.evaluate(center[2]+0.05)
            self.allow_touch(False, ['table_top'])
            self.set_status('SUCCEEDED', 'Object lifted and held; /pick/reset starts a new trial')
        except Exception as exc:
            self.set_status('CANCELED' if self.stop_requested.is_set() else 'FAILED', str(exc))
        finally:
            self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = Pick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.stop_requested.set()
        if node.worker is not None:
            node.worker.join(timeout=2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
