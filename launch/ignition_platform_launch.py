from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare


def get_platform_node(context, *args, **kwargs):

    node = Node(
        package="ignition_platform",
        executable="ignition_platform_node",
        namespace=LaunchConfiguration('drone_id'),
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": LaunchConfiguration('use_sim_time'),
            "control_modes_file": LaunchConfiguration('control_modes_file'),
            "simulation_mode": True,
            "cmd_vel_topic": LaunchConfiguration('cmd_vel_topic'),
            "arm_topic": LaunchConfiguration('arm_topic')
        }]
    )
    return [node]


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('ignition_platform'),
        'config', 'control_modes.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('drone_id', default_value=EnvironmentVariable(
            'AEROSTACK2_SIMULATION_DRONE_ID')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('control_modes_file', default_value=config),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel'),
        DeclareLaunchArgument('arm_topic', default_value='arm'),

        OpaqueFunction(function=get_platform_node)
    ])
