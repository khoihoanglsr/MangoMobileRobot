from setuptools import find_packages, setup
from glob import glob

package_name = 'view_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #('share/' + package_name + '/launch', ['launch/view_robot.launch']),
        ('share/' + package_name + '/urdf', ['urdf/robot_description.urdf']),
        ('share/' + package_name + '/rviz', ['rviz/default_view.rviz']),
        ('share/' + package_name + '/meshes', glob('meshes/*')), #include meshes,
        ('share/' + package_name + '/maps', glob('maps/*')),
        
        # 1. CẬP NHẬT FILE LAUNCH MỚI VÀ CŨ
        # use gloal to get all
        ('share/' + package_name + '/launch', glob('launch/*')),

        # 2. THÊM THƯ MỤC Cau HÌNH (CONFIG)
        ('share/' + package_name + '/config', glob('config/*')),
        ('share' + package_name + '/view_robot_pkg', glob('view_robot_pkg/*')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apostolos-ubuntu-pc',
    maintainer_email='apostolos-ubuntu-pc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'fake_diff_drive = view_robot_pkg.fake_diff_drive:main',
            'odom_frame_publisher = view_robot_pkg.odom_frame_publisher:main',
            'odom_tf_broadcaster = view_robot_pkg.odom_tf_broadcaster:main',
            'my_teleop_node = view_robot_pkg.teleop_node:main',
        ],
    },
)
