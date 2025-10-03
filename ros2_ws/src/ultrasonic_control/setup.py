from setuptools import setup

package_name = 'ultrasonic_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeffrey',
    maintainer_email='you@example.com',
    description='ROS2 control: drive forward until ultrasonic detects obstacle.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_publisher = ultrasonic_control.ultrasonic_publisher:main',
            'motor_driver = ultrasonic_control.motor_driver:main',
        ],
    },
)
