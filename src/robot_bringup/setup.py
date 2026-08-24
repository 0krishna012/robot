import os
from glob import glob
from setuptools import setup

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config lives at the workspace root (../../config), not inside this
        # package, so all ROS packages in the workspace can share one place
        # for tunable yaml params.
        (os.path.join('share', package_name, 'config'), glob('../../config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='services@atsnai.com',
    description='Bringup, serial bridge, and SLAM launch files for the diff-drive hub-motor robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'serial_bridge_node = robot_bringup.serial_bridge:main',
        ],
    },
)
