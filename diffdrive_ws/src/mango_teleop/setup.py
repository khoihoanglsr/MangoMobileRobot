from setuptools import setup
from glob import glob

package_name = 'mango_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robomango',
    maintainer_email='creeperthien@gmail.com',
    description='Gamepad teleop launcher',
    license='MIT',
)

