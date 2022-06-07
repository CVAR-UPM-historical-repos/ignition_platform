from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('ignition_platform'),
        'config', 'control_modes.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value='drone_sim_rafa_0'),
        DeclareLaunchArgument('mass', default_value='1.5'),
        DeclareLaunchArgument('max_thrust', default_value='15.0'),
        DeclareLaunchArgument('min_thrust', default_value='0.15'),
        DeclareLaunchArgument('sensors', default_value='none'),

        DeclareLaunchArgument('control_modes_file', default_value=config),
        
        Node(
            package="ignition_platform",
            executable="ignition_platform_node",
            namespace=LaunchConfiguration('drone_id'),
            output="screen",
            emulate_tty=True,
            parameters=[
                {"control_modes_file": LaunchConfiguration('control_modes_file'),
                "simulation_mode": True,
                "mass": LaunchConfiguration('mass'),
                "max_thrust": LaunchConfiguration('max_thrust'),
                "min_thrust":  LaunchConfiguration('min_thrust'),
                "sensors": LaunchConfiguration('sensors')
                }],
            remappings=[("sensor_measurements/odometry", "self_localization/odom")]
        ),
    ])
