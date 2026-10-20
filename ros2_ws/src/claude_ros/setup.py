import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'claude_ros'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeffrey',
    maintainer_email='jeffrey.jahja7@gmail.com',
    description='Autonomous robot navigation using Claude as the decision-making brain.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'claude_agent_node = claude_ros.claude_agent_node:main',
            'motion_controller_node = claude_ros.motion_controller_node:main',
        ],
    },
)
