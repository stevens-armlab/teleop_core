from glob import glob
import os

from setuptools import setup

package_name = 'teleop_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Srikarran Sowrirajan',
    maintainer_email='ssowrira@stevens.edu',
    description='Teleoperation nodes driven by the 3D Systems Touch haptic device (ROS 2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_cmd = teleop_core.teleop_cmd:main',
            'teleop_fr3 = teleop_core.teleop_fr3:main',
            'teleop_xarm7 = teleop_core.teleop_xarm7:main',
        ],
    },
)
