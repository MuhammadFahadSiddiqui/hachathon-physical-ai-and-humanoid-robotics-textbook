from setuptools import setup
import os
from glob import glob

package_name = 'vla_coordinator'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='Vision-Language-Action behavior coordination and system integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_coordinator = vla_coordinator.behavior_coordinator:main',
            'action_executor = vla_coordinator.action_executor:main',
            'system_monitor = vla_coordinator.system_monitor:main',
            'gripper_controller = vla_coordinator.gripper_controller:main',
        ],
    },
)
