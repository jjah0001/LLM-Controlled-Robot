"""Full robot bring-up: sensors, motion controller, Claude agent, motor driver.

    ros2 launch claude_ros robot.launch.py
    ros2 launch claude_ros robot.launch.py use_agent:=false   # sensors + controller only
    ros2 launch claude_ros robot.launch.py mission:="Find the red ball" model:=claude-haiku-5-5
    ros2 launch claude_ros robot.launch.py surface:=carpet     # more motor power
    ros2 launch claude_ros robot.launch.py min_duty:=85 kick_s:=0.25
    ros2 launch claude_ros robot.launch.py use_edge_sensor:=true   # downward drop sensor fitted

Optional hardware packages (m1ct_d2 LiDAR driver, camera_ros) are skipped with a
warning when they are not installed, so the rest of the stack still starts.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _has_package(name):
    try:
        get_package_share_directory(name)
        return True
    except PackageNotFoundError:
        return False


MOTOR_ARGS = ('min_duty', 'max_duty', 'kick_duty', 'kick_s')


def _truthy(value):
    return value.lower() in ('1', 'true', 'yes', 'on')


def _setup(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    params = arg('params_file')
    actions = []

    if _truthy(arg('use_lidar')):
        if _has_package('m1ct_d2'):
            actions.append(Node(
                package='m1ct_d2', executable='m1ct_d2', name='lidar', output='screen',
                parameters=[params, {'port': arg('lidar_port')}]))
        else:
            actions.append(LogInfo(msg='[robot.launch] m1ct_d2 not found: LiDAR skipped. '
                                       'Clone jjah0001/D05_lidar into ros2_ws/src.'))

    if _truthy(arg('use_camera')):
        if _has_package('camera_ros'):
            actions.append(Node(
                package='camera_ros', executable='camera_node', name='camera',
                output='screen', parameters=[params]))
        else:
            actions.append(LogInfo(msg='[robot.launch] camera_ros not found: camera skipped.'))

    edge = _truthy(arg('use_edge_sensor'))
    if edge:
        # Second HC-SR04 pointing at the floor; its own pins come from robot.yaml.
        actions.append(Node(
            package='ultrasonic_control', executable='ultrasonic_publisher',
            name='edge_sensor', output='screen', parameters=[params],
            remappings=[('ultrasonic/range', 'edge/range')]))
    else:
        actions.append(LogInfo(msg='[robot.launch] No edge sensor (use_edge_sensor:=false): '
                                   'only the camera can spot drops and stairs.'))
    motor_overrides = {k: float(arg(k)) for k in MOTOR_ARGS if arg(k)}
    if arg('surface'):
        motor_overrides['surface'] = arg('surface')
    actions += [
        Node(package='ultrasonic_control', executable='ultrasonic_publisher',
             name='ultrasonic_publisher', output='screen', parameters=[params]),
        Node(package='motor_driver', executable='motor_driver_node',
             name='motor_driver', output='screen',
             parameters=[params, motor_overrides]),
        Node(package='claude_ros', executable='motion_controller_node',
             name='motion_controller', output='screen',
             parameters=[params, {'edge_sensor': edge}]),
    ]

    if _truthy(arg('use_agent')):
        overrides = {k: arg(k) for k in ('model', 'mission') if arg(k)}
        actions.append(Node(
            package='claude_ros', executable='claude_agent_node', name='claude_agent',
            output='screen', parameters=[params, overrides]))
    return actions


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('claude_ros'), 'config', 'robot.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('use_agent', default_value='true'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_edge_sensor', default_value='false',
                              description='true if the downward edge (drop) sensor is fitted'),
        DeclareLaunchArgument('surface', default_value='',
                              description='Motor power preset: hard_floor or carpet'),
        DeclareLaunchArgument('min_duty', default_value='',
                              description='Fixed motor min_duty %, overriding the preset'),
        DeclareLaunchArgument('max_duty', default_value='',
                              description='Override motor max_duty %'),
        DeclareLaunchArgument('kick_duty', default_value='',
                              description='Override motor start-kick duty %'),
        DeclareLaunchArgument('kick_s', default_value='',
                              description='Motor start-kick length in s (0 = off)'),
        DeclareLaunchArgument('model', default_value='',
                              description='Override the Claude model ID from robot.yaml'),
        DeclareLaunchArgument('mission', default_value='',
                              description='Override the mission text from robot.yaml'),
        OpaqueFunction(function=_setup),
    ])
