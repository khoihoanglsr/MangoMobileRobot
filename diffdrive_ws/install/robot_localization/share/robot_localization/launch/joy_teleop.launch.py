from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('mango_teleop'),  # đổi thành tên package chứa file
        'config',
        'teleop.yaml'
    ])

    return LaunchDescription([
        Node(package='joy', executable='joy_node', name='joy_node'),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[params_file]
        ),
    ])


