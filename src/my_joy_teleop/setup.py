from setuptools import find_packages, setup

package_name = 'my_joy_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robomango',
    maintainer_email='creeperthien@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Co the thay my_teleop_node thanh ten khac 
            'my_teleop_node = my_joy_teleop.teleop_node:main', 
        ],
    },
)
