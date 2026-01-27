import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ----------------------------------------------------
    # 1. INIT PACKAGE & DIRECTORIES
    # ----------------------------------------------------
    PACKAGE_NAME = 'view_robot_pkg' 
    pkg_dir = get_package_share_directory(PACKAGE_NAME)
    slam_dir = get_package_share_directory('slam_toolbox')
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    
    # ----------------------------------------------------
    # 2. DECLARE ARGUMENTS
    # ----------------------------------------------------
    
    # File Config & Map
    nav2_params_file = PathJoinSubstitution([pkg_dir, 'config', 'nav2_params.yaml'])
    slam_params_file = PathJoinSubstitution([pkg_dir, 'config', 'mapper_params_online_async.yaml'])
    map_file_path = PathJoinSubstitution([pkg_dir, 'maps', 'khoi_sofa_map.yaml'])

    # Hardware Params
    lidar_frame_arg = DeclareLaunchArgument('lidar_frame', default_value='lidar_link')
    lidar_port_arg = DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0')
    lidar_baud_arg = DeclareLaunchArgument('lidar_baud', default_value='1000000')
    stm32_port_arg = DeclareLaunchArgument('stm32_port', default_value='/dev/ttyUSB1')
    
    # Simulation Time
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')

    # Map Argument
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Full path to map yaml file to load'
    )
    
    # ----------------------------------------------------
    # 3. HARDWARE SECTION
    # ----------------------------------------------------
    
    # 3.1 Robot State Publisher
    robot_state_and_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'view_robot.launch.py'])
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # 3.2 Node Odom
    odom_node = Node(
        package='view_robot_pkg',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen'
    )
    
    # 3.3 Node Micro-ROS
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', LaunchConfiguration('stm32_port'), '-b', '115200'],
        output='screen'
    )

    # 3.4 Node Lidar
    sllidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sllidar_dir, 'launch', 'sllidar_s2_launch.py']) 
        ),
        launch_arguments={
            'frame_id': LaunchConfiguration('lidar_frame'),
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baud')
        }.items()
    )

    # ----------------------------------------------------
    # 4. MAPPING SECTION (SLAM TOOLBOX)
    # ----------------------------------------------------
    # Dùng khi muốn vẽ bản đồ mới.
    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([slam_dir, 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': slam_params_file
        }.items()
    )

    # ----------------------------------------------------
    # 5. NAVIGATION SECTION (MODULES)
    # ----------------------------------------------------
    
    # 5.1 LOCALIZATION (Map + AMCL) - Dùng khi chạy map có sẵn
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'zackoi_localization.launch.py'])
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_file
        }.items()
    )

    # 5.2 NAVIGATION (Controller + Planner)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dir, 'launch', 'zackoi_navigation.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_params_file
        }.items()
    )
    
    # ----------------------------------------------------
    # 6. RETURN
    # ----------------------------------------------------
    return LaunchDescription([
        # --- Arguments ---
        map_arg,
        lidar_frame_arg,
        lidar_port_arg,
        lidar_baud_arg,
        stm32_port_arg,
        use_sim_time_arg,
        
        # --- Hardware (Luôn chạy) ---
        odom_node,
        robot_state_and_rviz,
        micro_ros_agent,
        sllidar_driver,
        
        # --- CHẾ ĐỘ 1: NAVIGATION (Chạy map cũ) ---
        # (Bỏ comment 2 dòng dưới để chạy Nav2)
        localization_launch,
        navigation_launch,

        # --- CHẾ ĐỘ 2: MAPPING (Vẽ map mới) ---
        # (Bỏ comment dòng dưới, và comment 2 dòng trên để chạy SLAM)
        # slam_node, 
    ])