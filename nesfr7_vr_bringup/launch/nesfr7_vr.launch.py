
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

from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)

os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

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

#    #
#    # robot_state_publisher_node
#    #
#    nesfr_arm_params = PathJoinSubstitution(
#        [FindPackageShare("nesfr_arm_description"), "config", "nesfr7_arm.yaml"]
#    )
#
#    robot_description_content = Command(
#        [
#            PathJoinSubstitution([FindExecutable(name="xacro")]),
#            " ",
#            PathJoinSubstitution([FindPackageShare("nesfr_arm_description"), "urdf", "nesfr_arm.urdf.xacro"]),
#            " ",
#            "nesfr_arm_params:=",
#            nesfr_arm_params,
#            " ",
#            "prefix:=",
#            hostname + '/',
#            " ",
#        ]
#    )
#    robot_description = {"robot_description": robot_description_content}
#    robot_state_publisher_node = Node(
#        package="robot_state_publisher",
#        executable="robot_state_publisher",
#        namespace=namespace,
#        output="both",
#        parameters=[robot_description],
#    )
#
#    #
#    # nesfr_arm_node
#    #
#    robot_config_file = LaunchConfiguration('robot_config_file', default=[namespace, '.yaml'])
#    nesfr7_arm_params = PathJoinSubstitution(
#            [FindPackageShare("nesfr_arm_bringup"), "config", robot_config_file]
#            )
#
#    #
#    # references
#    #  - https://answers.ros.org/question/311471/selecting-log-level-in-ros2-launch-file/
#    #  - https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html
#    #
#    nesfr7_arm_only_node = Node(
#        package='nesfr_arm_only_node_py',
#        executable='nesfr_arm_only_node',
#        namespace=namespace,
#        name='nesfr7_arm_only_node',
#        parameters=[nesfr7_arm_params],
#        #parameters=[{"param0": 1, "param1": 2}],
#        arguments=['--ros-args', '--log-level', [namespace, '.nesfr7_arm_only_node:=info'],],
#        output='both',
#    )

    nesfr7_arm_common_launch = IncludeLaunchDescription(
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

    nesfr7_arm_only_common_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('nesfr_arm_bringup'), 'launch',
                    'nesfr7_arm_only_common.launch.py'
                    ])
                ]),
            launch_arguments={
                'namespace': namespace
                }.items()
            )


    # nesfr7_XX
    if re.search("nesfr7_[0-9]+$", hostname):
        return LaunchDescription([
            namespace_launch_arg,
            nesfr_vr_launch,
        ])
    elif re.search("nesfr7_one_[0-9]+$", hostname):
        return LaunchDescription([
            namespace_launch_arg,
            nesfr7_arm_common_launch,
            nesfr_vr_launch,
        ])

    elif re.search("nesfr7_arm_[0-9]+$", hostname):
        return LaunchDescription([
            namespace_launch_arg,
            nesfr7_arm_only_common_launch,
            nesfr_vr_launch,
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
