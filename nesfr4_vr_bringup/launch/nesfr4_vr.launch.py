import os

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression, PathJoinSubstitution)

import ament_index_python.packages
def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    hostname = os.uname().nodename.replace('-', '_')
    namespace_launch_arg = DeclareLaunchArgument(
            'namespace',
            #default_value=EnvironmentVariable(name='HOSTNAME')
            default_value=hostname
            )

    nesfr_vr_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nesfr_vr_ros2'), 'launch',
                    'nesfr_vr.launch.py'
                    ])
                ]),
            launch_arguments={
                'namespace': namespace
                }.items()
            )

    config_directory = os.path.join(
            ament_index_python.packages.get_package_share_directory('nesfr4_node'),
            'config')
    nesfr4_params = os.path.join(config_directory, 'nesfr4-params.yaml')
    nesfr4_node = Node(package='nesfr4_node',
            executable='nesfr4_node',
            output='both',
            namespace=namespace,
            parameters=[nesfr4_params])

    return LaunchDescription([
        namespace_launch_arg,
        nesfr4_node,
        nesfr_vr_launch,
    ])
