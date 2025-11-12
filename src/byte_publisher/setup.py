from setuptools import setup

package_name = 'byte_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/byte_publisher_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/byte_publisher.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Simple high-IO load byte publisher',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'byte_publisher = byte_publisher.byte_publisher:main',
        ],
    },
)
