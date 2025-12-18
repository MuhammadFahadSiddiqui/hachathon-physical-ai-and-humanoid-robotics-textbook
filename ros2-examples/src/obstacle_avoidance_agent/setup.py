from setuptools import find_packages, setup

package_name = 'obstacle_avoidance_agent'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Obstacle avoidance agent example for Python agents and controllers chapter',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance_agent = obstacle_avoidance_agent.obstacle_avoidance_agent:main',
            'mock_sensor_publisher = obstacle_avoidance_agent.mock_sensor_publisher:main',
        ],
    },
)
