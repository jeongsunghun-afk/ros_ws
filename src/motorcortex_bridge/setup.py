from setuptools import setup
import os
from glob import glob

package_name = 'motorcortex_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@email.com',
    description='Motorcortex to ROS2 joint state bridge',
    license='BSD',
    entry_points={
        'console_scripts': [
            'joint_state_bridge = motorcortex_bridge.joint_state_bridge:main',
        ],
    },
)
