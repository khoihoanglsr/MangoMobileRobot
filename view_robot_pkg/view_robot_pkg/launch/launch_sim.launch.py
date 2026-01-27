import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue 
#File danh cho mo phong Gazebo
def generate_launch_description():
    pkg_name = 'view_robot_pkg' 
    pkg_share = get_package_share_directory(pkg_name)

    # Cấu hình đường dẫn Meshes
    set_env_vars = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_share, '..')
    )

    # Đường dẫn file URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_description.urdf')
    
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )
    
    # Node Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config, 
            'use_sim_time': True
        }]
    )

    # Run Gazebo Harmonic
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn Robot
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'my_bot',
                   '-z', '0.5'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model'
        ],
        output='screen'
    )

    # === PHẦN THÊM MỚI: GỌI JOYSTICK ===
    # Gọi file joystick.launch.py đang nằm trong thư mục launch của view_robot_pkg
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'joystick.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    # ===================================

    return LaunchDescription([
        set_env_vars,
        node_robot_state_publisher,
        gazebo,
        spawn_entity,
        bridge,
        joystick  # <--- Đừng quên thêm dòng này!
    ])