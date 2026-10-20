from setuptools import find_packages, setup

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeffrey',
    maintainer_email='jeffrey.jahja7@gmail.com',
    description='L298N differential-drive motor driver for /cmd_vel.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver_node = motor_driver.motor_driver_node:main',
            'keyboard_controller = motor_driver.keyboard_controller:main',
        ],
    },
)
