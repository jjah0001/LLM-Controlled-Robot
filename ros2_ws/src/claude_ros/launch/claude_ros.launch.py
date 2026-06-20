from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='claude-haiku-4-5-20251001'),
        DeclareLaunchArgument('call_rate_hz', default_value='0.5'),
        DeclareLaunchArgument('max_linear', default_value='0.5'),
        DeclareLaunchArgument('max_angular', default_value='0.5'),

        # Ultrasonic sensor
        Node(
            package='ultrasonic_control',
            executable='ultrasonic_publisher',
            name='ultrasonic_publisher',
            parameters=[{'rate_hz': 10.0}],
            output='screen',
        ),

        # Motor driver (listens to /cmd_vel)
        Node(
            package='motor_driver',
            executable='motor_driver_node',
            name='motor_driver',
            output='screen',
        ),

        # Claude autonomous agent
        Node(
            package='claude_ros',
            executable='claude_agent_node',
            name='claude_agent',
            output='screen',
            parameters=[{
                'model':         LaunchConfiguration('model'),
                'call_rate_hz':  LaunchConfiguration('call_rate_hz'),
                'max_linear':    LaunchConfiguration('max_linear'),
                'max_angular':   LaunchConfiguration('max_angular'),
                'image_topic':   '/camera/image_raw',
                'image_width':   320,
                'image_height':  240,
                'jpeg_quality':  70,
            }],
        ),
    ])
