import launch
from launch import LaunchDescription, conditions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone0'),
        
        Node(
            package="ignition_platform",
            executable="ignition_platform_node",
            name="platform",
            namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
            parameters=[],
            remappings=[("sensor_measurements/odometry", "self_localization/odom")],
        ),
    ])
