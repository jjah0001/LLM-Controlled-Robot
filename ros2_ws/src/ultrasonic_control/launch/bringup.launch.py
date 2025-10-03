# launch/bringup.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ultrasonic_control',
            executable='ultrasonic_publisher',
            name='ultrasonic_publisher',
            parameters=[{'rate_hz': 10.0}],
            output='screen'
        ),
        Node(
            package='ultrasonic_control',
            executable='motor_driver',
            name='motor_driver',
            parameters=[
                {'stop_distance_m': 0.2},
                {'hysteresis_m': 0.05},
                {'speed_percent': 80.0},
                {'pwm_hz': 1000.0},
            ],
            output='screen'
        ),
    ])
