import os
import re

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

from launch.events import Shutdown
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    hostname = os.uname().nodename.replace('-', '_')
    namespace_launch_arg = DeclareLaunchArgument(
            'namespace',
            #default_value=EnvironmentVariable(name='HOSTNAME')
            default_value=hostname
            )

    #
    # nesfr_vr_ros2
    #
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

    #
    # robot_state_publisher_node
    #
    nesfr_arm_params = PathJoinSubstitution(
        [FindPackageShare("nesfr_arm_description"), "config", "nesfr7_arm.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("nesfr_arm_description"), "urdf", "nesfr_arm.urdf.xacro"]),
            " ",
            "nesfr_arm_params:=",
            nesfr_arm_params,
            " ",
            "prefix:=",
            hostname + '/',
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[robot_description],
    )


    nesfr7_arm_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nesfr_arm_bringup'), 'launch',
                    'nesfr7_arm_common.launch.py'
                    ])
                ]),
            launch_arguments={
                'namespace': namespace
                }.items()
            )


    nesfr4_params = PathJoinSubstitution([
            FindPackageShare('nesfr4_node'),
            'config', 'nesfr4-params.yaml'])

    nesfr4_node = Node(package='nesfr4_node',
            executable='nesfr4_node',
            output='both',
            namespace=namespace,
            parameters=[nesfr4_params])

    # nesfr7_XX
    if re.search("nesfr7_[0-9]+$", hostname):
        return LaunchDescription([
            namespace_launch_arg,
            nesfr_vr_launch,
        ])

    elif re.search("nesfr7_arm_[0-9]+$", hostname):
        return LaunchDescription([
            namespace_launch_arg,
            robot_state_publisher_node,
            nesfr7_arm_launch,
            nesfr_vr_launch,
        ])
    elif re.search("nesfr4$", hostname):
        return LaunchDescription([
            namespace_launch_arg,
            nesfr4_node,
            nesfr_vr_launch,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=nesfr4_node,
                    on_exit=[
                        LogInfo(msg=(' closed nesfr4_node')),
                        EmitEvent(event=Shutdown(reason='Window closed'))
                        ]
                    )
                ),
        ])
    else:
        #
        # references:
        #  - https://github.com/ros2/launch/blob/foxy/launch/doc/source/architecture.rst#id49
        #  - https://docs.ros.org/en/galactic/Tutorials/Intermediate/Launch/Using-Event-Handlers.html
        #
        return LaunchDescription([
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[LogInfo(
                        msg=['This system({}) is unsupported by [nesfr7_vr_bringup]! Check your system or hostname'.format(hostname)]
                    )]
            )
        ),
        ])
