from launch import LaunchDescription
from launch_ros.actions import Node

# ĐÂY LÀ HÀM BẮT BUỘC PHẢI CÓ TRONG MỌI FILE LAUNCH
def generate_launch_description():

    # 1. Tự động chạy lệnh: ros2 run joy_linux joy_linux_node
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0', # cap quyen cho tay cam
            'deadzone': 0.1,         # Vùng chết để cần không bị trôi
            'autorepeat_rate': 20.0,
        }]
    )

    # 2. Tự động chạy lệnh: ros2 run my_joy_teleop teleop_node
    joy_teleop_node = Node(
        package='my_joy_teleop',     # Tên package phụ của bạn
        executable='my_teleop_node',    # Tên lệnh (phải khớp với console_scripts trong setup.py)
        name='my_joy_teleop_node',
        output='screen',
        parameters=[{
            'scale_linear': 1,     # Tốc độ đi thẳng
            'scale_angular': 3.14     # Tốc độ xoay
        }]
    )

    # Trả về danh sách các node cần chạy
    return LaunchDescription([
        joy_node,
        joy_teleop_node
    ])