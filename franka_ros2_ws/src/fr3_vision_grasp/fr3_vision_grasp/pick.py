"""Perception-driven MoveIt pick and place, with physical success evaluation."""
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
from moveit_msgs.msg import (Constraints, JointConstraint,
    MotionPlanRequest, PlanningOptions, PlanningScene, CollisionObject,
    AttachedCollisionObject, AllowedCollisionEntry, PlanningSceneComponents)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetCartesianPath, GetPositionIK
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from .core import ARM, HOME, EmptyTableConfirmation


class Pick(Node):
    def __init__(self):
        super().__init__('fr3_pick_place')
        for name, value in [('color', 'red'), ('object_size', 0.03), ('approach_height', 0.12),
                            ('lift_height', 0.10), ('verify_simulation', True)]:
            self.declare_parameter(name, value)
        self.color = self.get_parameter('color').value
        if self.color not in ('red', 'green', 'blue'):
            raise ValueError('color must be red, green or blue')
        self.first_color = self.color
        self.color_order = [self.first_color, *[c for c in ('red', 'green', 'blue') if c != self.first_color]]
        self.placed_colors = []
        self.size = self.get_parameter('object_size').value
        self.lock = threading.Lock()
        self.busy = False
        self.holding = False
        self.completed = False
        self.monitor_grasp = False
        self.contact_lost_since = None
        self.running = True
        self.samples = {color: deque(maxlen=8) for color in ('red', 'green', 'blue')}
        self.detection_frames = deque(maxlen=120)
        self.metrics = None
        self.metric_wall = 0.0
        self.active_goal = None
        self.stop_requested = threading.Event()
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')
        self.gripper = ActionClient(self, GripperCommand, '/franka_gripper/gripper_action')
        self.apply_scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.get_scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.reset_sim = self.create_client(Trigger, '/simulation/reset')
        self.subs = [self.create_subscription(PoseStamped, f'/perception/{c}/pose',
                    lambda msg, c=c: self.observe(c, msg), 10) for c in self.samples]
        self.create_subscription(String, '/simulation/grasp_metrics', self.on_metrics, 10)
        self.create_subscription(Detection2DArray, '/perception/detections', self.on_detections, 10)
        self.create_subscription(String, '/perception/pick_request', self.on_request, 10)
        self.create_service(Trigger, '/pick/start', self.start)
        self.create_service(Trigger, '/pick/cancel', self.cancel)
        self.create_service(Trigger, '/pick/reset', self.reset)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status = self.create_publisher(String, '/pick/status', qos)
        self.worker = None
        self.scene_xml = Path(get_package_share_directory('fr3_vision_grasp')) / 'scene/scene_grasp.xml'
        self.set_status('READY', 'Waiting for stable detections and /pick/start')

    def set_status(self, state, message):
        self.status.publish(String(data=json.dumps({'state': state, 'message': message,
            'color': self.color, 'placed_colors': self.placed_colors.copy()})))
        self.get_logger().info(f'{state}: {message}')

    def observe(self, color, msg):
        if msg.header.frame_id == 'fr3_link0':
            self.samples[color].append(copy.deepcopy(msg))

    def on_metrics(self, msg):
        self.metrics = json.loads(msg.data)
        self.metric_wall = time.monotonic()

    def on_detections(self, msg):
        self.detection_frames.append((Time.from_msg(msg.header.stamp).nanoseconds,
                                      {d.id for d in msg.detections}, time.monotonic()))

    def on_request(self, msg):
        if msg.data.strip().lower() == 'pick':
            self.start(Trigger.Request(), Trigger.Response())

    def start(self, request, response):
        with self.lock:
            if self.busy or self.holding or self.completed:
                response.message = 'Busy or trial already run. Use /pick/reset before another trial.'
                return response
            self.busy = True
            self.stop_requested.clear()
        self.worker = threading.Thread(target=self.run_pick, daemon=True)
        self.worker.start()
        response.success, response.message = True, 'Table sorting started; monitor /pick/status'
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
                if not self.get_scene.wait_for_service(timeout_sec=10):
                    raise RuntimeError('get_planning_scene unavailable')
                current = self.wait(self.get_scene.call_async(GetPlanningScene.Request(
                    components=PlanningSceneComponents(components=(
                        PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
                        PlanningSceneComponents.WORLD_OBJECT_NAMES)))), 10).scene
                target_ids = {'target_' + c for c in self.samples}
                attached_ids = {a.object.id for a in current.robot_state.attached_collision_objects} & target_ids
                world_ids = {o.id for o in current.world.collision_objects} & target_ids
                scene = PlanningScene(is_diff=True)
                scene.robot_state.is_diff = True
                scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
                    object=CollisionObject(id=name, operation=CollisionObject.REMOVE)) for name in attached_ids]
                scene.world.collision_objects = [CollisionObject(id=name, operation=CollisionObject.REMOVE)
                                                for name in world_ids | attached_ids]
                self.apply(scene)
                for color in self.samples:
                    self.allow_touch(False, color=color)
                    _, support, _, _ = self.destination(color)
                    self.allow_touch(False, ['table_top', support], color=color)
                if not self.reset_sim.wait_for_service(timeout_sec=5):
                    raise RuntimeError('Simulation reset service unavailable')
                result = self.wait(self.reset_sim.call_async(Trigger.Request()), 8)
                if not result.success:
                    raise RuntimeError(result.message)
                for samples in self.samples.values():
                    samples.clear()
                self.detection_frames.clear()
                self.placed_colors.clear()
                self.color = self.first_color
                self.holding = False
                self.completed = False
                self.monitor_grasp = False
                self.contact_lost_since = None
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
            self.check_grasp()
            time.sleep(0.01)
        return future.result()

    def check_grasp(self):
        """Abort carrying motions if physical contact is lost; never steer from truth."""
        if not self.monitor_grasp or not self.get_parameter('verify_simulation').value:
            return
        metrics = self.metrics
        if not metrics or time.monotonic() - self.metric_wall > 2:
            raise RuntimeError('Grasp monitoring lost fresh simulation feedback')
        obj = metrics.get('objects', {}).get(self.color)
        if obj and len(obj['contacts']) == 2:
            self.contact_lost_since = None
        elif self.contact_lost_since is None:
            self.contact_lost_since = metrics['sim_time']
        elif metrics['sim_time'] - self.contact_lost_since >= 0.3:
            raise RuntimeError('Object slipped: lost two-finger contact during transport')

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
        # Seed IK from the current robot state. Random pose-goal sampling can
        # choose a redundant elbow posture that cannot descend to the cube.
        if not self.ik.wait_for_service(timeout_sec=10):
            raise RuntimeError('MoveIt IK service unavailable')
        state_request = GetPlanningScene.Request(components=PlanningSceneComponents(
            components=PlanningSceneComponents.ROBOT_STATE))
        state = self.wait(self.get_scene.call_async(state_request), 10).scene.robot_state
        ik_request = GetPositionIK.Request()
        ik_request.ik_request.group_name = 'fr3_arm'
        ik_request.ik_request.ik_link_name = 'fr3_hand_tcp'
        ik_request.ik_request.robot_state = state
        ik_request.ik_request.pose_stamped = PoseStamped(pose=pose)
        ik_request.ik_request.pose_stamped.header.frame_id = 'fr3_link0'
        ik_request.ik_request.avoid_collisions = True
        ik_request.ik_request.timeout.sec = 2
        solution = self.wait(self.ik.call_async(ik_request), 10)
        if solution.error_code.val != solution.error_code.SUCCESS:
            raise RuntimeError(f'No collision-free approach IK: {solution.error_code.val}')
        joints = solution.solution.joint_state
        self.move_joints([joints.position[joints.name.index(name)] for name in ARM])

    def move_joints(self, positions):
        constraints = Constraints(joint_constraints=[JointConstraint(
            joint_name=name, position=float(position), tolerance_above=0.001,
            tolerance_below=0.001, weight=1.0) for name, position in zip(ARM, positions)])
        request = MotionPlanRequest(group_name='fr3_arm', pipeline_id='ompl',
            num_planning_attempts=10, allowed_planning_time=10., goal_constraints=[constraints],
            max_velocity_scaling_factor=0.25, max_acceleration_scaling_factor=0.25)
        request.start_state.is_diff = True
        result = self.action(self.move, MoveGroup.Goal(request=request,
            planning_options=PlanningOptions(plan_only=False, replan=False)), 120)
        if result.error_code.val != result.error_code.SUCCESS:
            raise RuntimeError(f'MoveIt error {result.error_code.val}')

    def return_home(self):
        self.set_status('RETURNING_HOME', 'Planning back to the initial joint posture')
        self.move_joints(HOME)
        self.set_status('AT_HOME', 'At initial posture; ready to observe the table')

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
        # Known static environment only. Movable object poses come exclusively from perception.
        for body in ET.parse(self.scene_xml).find('worldbody').findall('body'):
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

    def destination(self, color=None):
        """Read the matching fixed bin, never the movable cube, from the scene."""
        name = {'red': 'left_red_box', 'green': 'right_green_box',
                'blue': 'right_blue_box'}[color or self.color]
        body = ET.parse(self.scene_xml).find(f'worldbody/body[@name="{name}"]')
        if body is None:
            raise RuntimeError(f'Placement bin is missing: {name}')
        origin = np.fromstring(body.get('pos'), sep=' ')
        bottom = body.find(f'geom[@name="{name}_bottom"]')
        center = origin + np.fromstring(bottom.get('pos'), sep=' ')
        center[2] += np.fromstring(bottom.get('size'), sep=' ')[2] + self.size / 2
        rim_z = max(origin[2] + np.fromstring(g.get('pos', '0 0 0'), sep=' ')[2]
                    + np.fromstring(g.get('size'), sep=' ')[2] for g in body.findall('geom'))
        # The palm is wider than the bin opening. Put the lower half of the
        # cube below the rim, but keep the palm above it while opening.
        release_z = max(center[2] + 0.006, rim_z - self.size / 2 + 0.005)
        return center, bottom.get('name'), rim_z + 0.10, release_z

    def allow_touch(self, allow, links=None, color=None):
        if not self.get_scene.wait_for_service(timeout_sec=10):
            raise RuntimeError('get_planning_scene unavailable')
        request = GetPlanningScene.Request(components=PlanningSceneComponents(
            components=PlanningSceneComponents.ALLOWED_COLLISION_MATRIX))
        matrix = self.wait(self.get_scene.call_async(request), 10).scene.allowed_collision_matrix
        names = ['target_'+(color or self.color), *(links if links is not None else ['fr3_leftfinger', 'fr3_rightfinger'])]
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

    def detach(self, position):
        scene = PlanningScene(is_diff=True)
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [AttachedCollisionObject(
            link_name='fr3_hand_tcp', object=CollisionObject(
                id='target_' + self.color, operation=CollisionObject.REMOVE))]
        # Explicitly replace the detached world pose with the intended resting pose.
        scene.world.collision_objects = [self.box('target_' + self.color, position, [self.size]*3)]
        self.apply(scene)

    def stable_targets(self):
        # Discard images taken during motion; the home posture clears the camera view.
        after_ns = self.get_clock().now().nanoseconds
        empty = EmptyTableConfirmation(after_ns)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if not self.running or self.stop_requested.is_set():
                raise RuntimeError('Task canceled')
            valid = {}
            now_ns = self.get_clock().now().nanoseconds
            frames = [f for f in list(self.detection_frames) if after_ns < f[0] <= now_ns]
            if not frames or time.monotonic() - frames[-1][2] > 2 or now_ns - frames[-1][0] > 1e9:
                time.sleep(0.05)
                continue
            for stamp_ns, colors, _ in frames:
                empty.observe(stamp_ns, colors)
            for color, history in self.samples.items():
                messages = [m for m in list(history) if Time.from_msg(m.header.stamp).nanoseconds > after_ns]
                if (color not in frames[-1][1] or len(messages) < 5 or
                        now_ns - Time.from_msg(messages[-1].header.stamp).nanoseconds > 1e9):
                    continue
                points = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for m in messages])
                if np.max(np.std(points, axis=0)) < 0.003:
                    valid[color] = copy.deepcopy(messages[-1])
            if valid:
                return valid
            if empty.confirmed(now_ns):
                return {}
            time.sleep(0.05)
        raise RuntimeError('Cannot confirm a stable target or an empty table from fresh RGB-D frames in 30 seconds')

    def evaluate(self, minimum_z, duration=2.0, timeout=15):
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

    def evaluate_placement(self, destination, duration=2.0, timeout=15):
        deadline, stable = time.monotonic() + timeout, None
        previous = None
        while time.monotonic() < deadline:
            if self.stop_requested.is_set() or not self.running:
                raise RuntimeError('Task canceled')
            metrics = self.metrics
            obj = metrics.get('objects', {}).get(self.color) if metrics else None
            valid = False
            if obj and time.monotonic() - self.metric_wall < 2:
                position = np.array(obj['position'])
                # Inside the bin, resting on its floor, released and no longer moving.
                valid = (not obj['contacts'] and
                         np.max(np.abs(position[:2] - destination[:2])) < 0.045 and
                         abs(position[2] - destination[2]) < 0.004 and
                         (previous is None or np.linalg.norm(position - previous) < 0.001))
                previous = position
            if valid:
                if stable is None:
                    stable = metrics['sim_time']
                if metrics['sim_time'] - stable >= duration:
                    return
            else:
                stable = None
            time.sleep(0.03)
        raise RuntimeError('Placement verification failed: object must rest released inside the matching bin')

    def pick_one(self, targets):
        target = targets[self.color]
        p = target.pose.position
        center = np.array([p.x, p.y, p.z])
        destination, support, clearance_z, release_z = self.destination()
        travel_z = max(center[2] + self.get_parameter('lift_height').value, clearance_z)
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
        self.monitor_grasp = True
        self.contact_lost_since = None
        self.attach(target)
        # The grasped cube initially rests on its support surface. Permit
        # only this object/support pair during the straight upward retreat.
        self.allow_touch(True, ['table_top'])
        self.set_status('LIFTING', 'Planning upward motion with attached collision object')
        self.move_to([center[0], center[1], travel_z], linear=True)
        if self.get_parameter('verify_simulation').value:
            self.evaluate(center[2]+0.05)
        self.allow_touch(False, ['table_top'])
        self.set_status('TRANSPORTING', f'Moving above {self.color} bin with gripper closed')
        above_bin = [destination[0], destination[1], travel_z]
        self.move_to(above_bin, linear=True)
        if self.get_parameter('verify_simulation').value:
            self.evaluate(center[2] + 0.05, duration=0.5)
        self.allow_touch(True, [support])
        self.set_status('PLACING', 'Lowering into bin before releasing')
        self.move_to([destination[0], destination[1], release_z], linear=True)
        self.check_grasp()
        self.set_status('RELEASING', 'Opening gripper inside bin')
        self.monitor_grasp = False
        opened = self.hand(0.08)
        if not opened.reached_goal:
            raise RuntimeError('Gripper failed to release fully')
        self.holding = False
        self.detach(destination)
        self.set_status('RETREATING', 'Moving clear of the placed object')
        self.move_to(above_bin, linear=True)
        self.allow_touch(False)
        self.set_status('VERIFYING', 'Checking object is released and settled inside bin')
        if self.get_parameter('verify_simulation').value:
            self.evaluate_placement(destination)
        self.placed_colors.append(self.color)
        self.set_status('OBJECT_PLACED', f'{self.color} placed; returning home before the next scan')

    def run_pick(self):
        try:
            self.populate_scene({})
            self.return_home()
            while self.running and not self.stop_requested.is_set():
                self.set_status('DETECTING', 'Checking the table for remaining blocks')
                targets = self.stable_targets()
                if not targets:
                    self.completed = True
                    self.set_status('SUCCEEDED',
                        f'Table clear; placed {len(self.placed_colors)} blocks and stopped at initial posture')
                    return
                self.color = next(c for c in self.color_order if c in targets)
                self.pick_one(targets)
                self.return_home()
            raise RuntimeError('Task canceled')
        except Exception as exc:
            self.set_status('CANCELED' if self.stop_requested.is_set() else 'FAILED', str(exc))
        finally:
            self.monitor_grasp = False
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
