
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='robotsim1'
        ),
        DeclareLaunchArgument('x'),
        DeclareLaunchArgument('y'),
        DeclareLaunchArgument('theta'),
        DeclareLaunchArgument(
            'timeout', default_value='120'
        ),

        Node(
            package='random_nav',
            executable='nav_demo',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[
                LaunchConfiguration('x'),
                LaunchConfiguration('y'),
                LaunchConfiguration('theta'),
                '--timeout',
                LaunchConfiguration('timeout'),
            ],
        ),
    ])

