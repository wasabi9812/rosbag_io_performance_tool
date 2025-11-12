from setuptools import find_packages, setup
import os

package_name = 'robot_image_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/multi_robot_image_launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/robot_image_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jh',
    maintainer_email='jh@todo.todo',
    description='Publishes robot status messages as JSON over ROS 2 topics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'robot_image_publisher = robot_image_publisher.robot_image_publisher:main',
        ],
    },
)
