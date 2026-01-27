from launch import LaunchDescription
from launch_ros.actions import Node

# ĐÂY LÀ HÀM BẮT BUỘC PHẢI CÓ TRONG MỌI FILE LAUNCH
def generate_launch_description():

    #linux_joy
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

    # external_joy
    joy_teleop_node = Node(
        package='view_robot_pkg',     # Tên package phụ của bạn
        executable='my_teleop_node',    # Tên lệnh (phải khớp với console_scripts trong setup.py)
        name='my_joy_teleop_node',
        output='screen',
        parameters=[{
            'scale_linear': 0.3,     # Tốc độ đi thẳng
            'scale_angular': 1.0     # Tốc độ xoay
        }]
    )

    # Trả về danh sách các node cần chạy
    return LaunchDescription([
        joy_node,
        joy_teleop_node
    ])