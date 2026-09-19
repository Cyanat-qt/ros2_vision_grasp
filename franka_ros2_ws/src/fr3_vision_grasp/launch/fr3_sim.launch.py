"""MuJoCo execution; no fake hardware or real Franka drivers are started."""
from pathlib import Path
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = Path(get_package_share_directory('franka_description'))
    share = Path(get_package_share_directory('fr3_vision_grasp'))
    urdf = xacro.process_file(str(description / 'robots/fr3/fr3.urdf.xacro'),
                             mappings={'hand': 'true', 'tcp_xyz': '0 0 0.1034', 'with_sc': 'false'}).toxml()
    root = ET.fromstring(urdf)
    mjcf = ET.parse(share / 'scene/fr3.xml')
    joint_limits = {}
    for i in range(1, 8):
        name = f'fr3_joint{i}'
        low, high = map(float, mjcf.find(f'.//joint[@name="{name}"]').get('range').split())
        joint = root.find(f'joint[@name="{name}"]')
        joint.find('limit').set('lower', str(low))
        joint.find('limit').set('upper', str(high))
        safety = joint.find('safety_controller')
        if safety is not None:
            safety.set('soft_lower_limit', str(low))
            safety.set('soft_upper_limit', str(high))
        joint_limits[name] = {'has_velocity_limits': True, 'max_velocity': 0.7,
                             'has_acceleration_limits': True, 'max_acceleration': 0.8}
    robot = {'robot_description': ET.tostring(root, encoding='unicode')}
    semantic = {'robot_description_semantic': xacro.process_file(
        str(description / 'robots/fr3/fr3.srdf.xacro'), mappings={'hand': 'true'}).toxml()}
    kinematics = {'robot_description_kinematics': {'fr3_arm': {
        'kinematics_solver': 'kdl_kinematics_plugin/KDLKinematicsPlugin',
        'kinematics_solver_search_resolution': 0.005, 'kinematics_solver_timeout': 0.1}}}
    planning = {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
                'ompl': {'planning_plugin': 'ompl_interface/OMPLPlanner',
                         'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                                             'default_planner_request_adapters/FixWorkspaceBounds '
                                             'default_planner_request_adapters/FixStartStateBounds '
                                             'default_planner_request_adapters/FixStartStateCollision '
                                             'default_planner_request_adapters/FixStartStatePathConstraints',
                         'start_state_max_bounds_error': 0.05,
                         'planner_configs': {'RRTConnect': {'type': 'geometric::RRTConnect', 'range': 0.0}},
                         'fr3_arm': {'planner_configs': ['RRTConnect']}}}
    controllers = {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
                   'moveit_simple_controller_manager': {
                       'controller_names': ['fr3_arm_controller', 'franka_gripper'],
                       'fr3_arm_controller': {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                                              'default': True, 'joints': list(joint_limits)},
                       'franka_gripper': {'type': 'GripperCommand', 'action_ns': 'gripper_action',
                                          'default': True, 'joints': ['fr3_finger_joint1']}},
                   'trajectory_execution.allowed_execution_duration_scaling': 5.0,
                   'trajectory_execution.allowed_goal_duration_margin': 5.0,
                   'trajectory_execution.allowed_start_tolerance': 0.03,
                   'moveit_manage_controllers': False}
    use_sim = {'use_sim_time': True}
    parameters = [robot, semantic, kinematics, planning, controllers, use_sim,
                  {'robot_description_planning': {'joint_limits': joint_limits}},
                  {'publish_robot_description': True, 'publish_robot_description_semantic': True,
                   'publish_planning_scene': True, 'publish_geometry_updates': True,
                   'publish_state_updates': True, 'publish_transforms_updates': True}]
    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('camera_view', default_value='true'),
        DeclareLaunchArgument('color', default_value='red'),
        DeclareLaunchArgument('camera_rate', default_value='15.0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        Node(package='fr3_vision_grasp', executable='mujoco_bridge', output='screen',
             parameters=[{'viewer': ParameterValue(LaunchConfiguration('viewer'), value_type=bool),
                          'camera_rate': ParameterValue(LaunchConfiguration('camera_rate'), value_type=float),
                          'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
                          'height': ParameterValue(LaunchConfiguration('height'), value_type=int)}]),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot, use_sim]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen', parameters=parameters),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', str(share / 'config/fr3.rviz')], parameters=parameters,
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
        Node(package='fr3_vision_grasp', executable='color_detector', output='screen', parameters=[use_sim]),
        Node(package='fr3_vision_grasp', executable='pick', output='screen',
             parameters=[use_sim, {'color': LaunchConfiguration('color')}]),
        Node(package='fr3_vision_grasp', executable='camera_view', output='screen', parameters=[use_sim],
             condition=IfCondition(LaunchConfiguration('camera_view'))),
    ])
