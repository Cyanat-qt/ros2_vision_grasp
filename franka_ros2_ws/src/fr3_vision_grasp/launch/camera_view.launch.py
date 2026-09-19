from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([Node(package='fr3_vision_grasp', executable='camera_view', output='screen')])
