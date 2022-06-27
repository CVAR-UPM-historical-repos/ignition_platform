from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare

import subprocess


def get_sensors(drone_namespace):
    sensors  = set()

    cmd = f"ign topic -l"
    output = subprocess.run(cmd.split(), capture_output=True, text=True)
    for line in output.stdout.split('\n'):
        if f"{drone_namespace}/model" in line:
            tokens = line.split('/')
            sensors.add(f"{tokens[2]},{tokens[4]},{tokens[6]},{tokens[8]},{tokens[10]}")
    return repr(sensors).replace("', '", ":")[2:-2] if len(sensors) else ""


def get_platform_node(context, *args, **kwargs):
    drone_namespace = LaunchConfiguration('drone_id').perform(context)

    node = Node(
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
            "sensors": get_sensors(drone_namespace)
            }],
        remappings=[("sensor_measurements/odometry", "self_localization/odom")]
    )
    return [node]


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('ignition_platform'),
        'config', 'control_modes.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable('AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('mass', default_value='1.5'),
        DeclareLaunchArgument('max_thrust', default_value='15.0'),
        DeclareLaunchArgument('min_thrust', default_value='0.15'),
        DeclareLaunchArgument('control_modes_file', default_value=config),
        
        OpaqueFunction(function=get_platform_node)
    ])
