import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('view_robot_pkg') # Đổi tên package của bạn nếu khác

    # 1. Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')

    #default la false khi khong mo phong trong gazebo, khi mo phong bat len true
    arg_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use sim time if true (usually for Gazebo)'
    )
    
    arg_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='Use joint_state_publisher_gui'
    )

    # 2.Process URDF
    urdf_path = os.path.join(pkg, 'urdf', 'robot_description.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc_content = infp.read()
    
    # Tạo dictionary tham số để dùng lại nhiều lần
    # Đây là chìa khóa để sửa lỗi "Waiting..."
    robot_desc_param = {'robot_description': robot_desc_content}

    # 3. Định nghĩa các Node

    # Node: Robot State Publisher to read urdf of my robot
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_desc_param, {'use_sim_time': use_sim_time}]
    )

    # Node: Joint State Publisher (GUI)
    jsp_gui = Node(
        condition=IfCondition(use_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        # Truyền robot_description vào đây để nó không phải chờ
        parameters=[robot_desc_param, {'use_sim_time': use_sim_time}]
    )

    # Node: Joint State Publisher (Non-GUI)
    jsp = Node(
        condition=UnlessCondition(use_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        # Truyền robot_description vào đây để nó không phải chờ
        parameters=[robot_desc_param, {'use_sim_time': use_sim_time}]
    )

    # Node: RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'default_view.rviz'])],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        arg_sim_time,
        arg_use_gui,
        rsp,
        jsp_gui,
        jsp,
        rviz
    ])