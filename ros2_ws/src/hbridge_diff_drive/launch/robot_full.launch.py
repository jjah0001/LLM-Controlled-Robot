from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hbridge_diff_drive',
            executable='driver_gpio',
            name='motor_driver',
            parameters=[{
                'left_in1_pin': 17, 'left_in2_pin': 22,
                'right_in1_pin': 23, 'right_in2_pin': 24,
                'wheel_base_m': 0.16,
                'deadband': 0.05,
                'watchdog_timeout_s': 0.5,
                'cmd_vel_topic': '/cmd_vel',
                'motor_command_topic': 'motor_command',
            }]
        ),
        Node(
            package='hbridge_diff_drive',
            executable='hcsr04',
            name='hcsr04_front',
            parameters=[{
                'trig_pin': 27, 'echo_pin': 4,
                'range_topic': '/ultrasonic/front',
                'publish_hz': 15, 'median_window': 3
            }]
        ),
        Node(
            package='hbridge_diff_drive',
            executable='forward_until_obstacle',
            name='forward_until_obstacle',
            parameters=[{
                'range_topic': '/ultrasonic/front',
                'cmd_vel_topic': '/cmd_vel',
                'stop_distance_m': 0.25,
                'slowdown_distance_m': 0.50,
                'forward_speed_mps': 0.15,
                'control_hz': 20,
                'no_data_behavior': 'stop',
                'stale_timeout_s': 0.5,
            }]
        ),
    ])
