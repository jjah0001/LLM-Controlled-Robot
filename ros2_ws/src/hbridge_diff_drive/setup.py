from setuptools import find_packages, setup
from setuptools import setup

package_name = 'hbridge_diff_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hbridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='ROS 2 diff-drive motor driver for H-bridge (L298N/TB6612) using pigpio.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'hcsr04 = ultrasonic_hcsr04.hcsr04_node:main',
            'driver = hbridge_diff_drive.driver_node:main',
        ],
    },
)
