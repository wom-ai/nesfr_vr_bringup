import os
import re
import json
import netifaces

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

os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

import logging

logging.basicConfig(level=logging.INFO, format="[%(levelname)s] [%(module)s]: %(message)s")

#
# references
#  - https://pypi.org/project/netifaces/
#  - https://github.com/eclipse-cyclonedds/cyclonedds
#
def set_cyclonedds():
    # FIXME
    with open("/etc/nesfrvr/config/hw_config.json", "r") as json_file:
        json_data = json.load(json_file)
        logging.info(json_data['network']['mac_addr'])

    for iface in netifaces.interfaces():
        ifaddresses = netifaces.ifaddresses(iface)
        ret = ifaddresses.get(netifaces.AF_LINK, None)
        if ret is not None and len(ret) > 0:
            ret = ret[0].get('addr', None)
            if ret is not None and ret == json_data['network']['mac_addr']:
                os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><NetworkInterfaceAddress>{}</></></></>'.format(iface)
                logging.info("interface_name={} mac_addr={}".format(iface, ret))

    logging.info('export CYCLONEDDS_URI=', os.environ['CYCLONEDDS_URI'])

def get_configuration():
    try:
        with open("/etc/nesfrvr/config/hw_config.json", "r") as json_hw_config_file:
            json_hw_config_data = json.load(json_hw_config_file)
    except Exception as e:
        logging.error('No hardware configuration file ({})'.format(e))
        json_hw_config_data = None
    try:
        with open("/etc/nesfrvr/config/sw_config.json", "r") as json_sw_config_file:
            json_sw_config_data = json.load(json_sw_config_file)
    except Exception as e:
        logging.error('No software configuration file ({})'.format(e))
        json_sw_config_data = None

    return json_hw_config_data, json_sw_config_data

def set_cyclonedds(json_hw_config_data):
    for iface in netifaces.interfaces():
        ifaddresses = netifaces.ifaddresses(iface)
        ret = ifaddresses.get(netifaces.AF_LINK, None)
        if ret is not None and len(ret) > 0:
            ret = ret[0].get('addr', None)
            if ret is not None and ret == json_hw_config_data['network']['mac_addr']:
                os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><NetworkInterfaceAddress>{}</></></></>'.format(iface)
                logging.info("interface_name={} mac_addr={}".format(iface, ret))

def generate_launch_description():

    json_hw_config_data, json_sw_config_data = get_configuration()

    if json_hw_config_data:
        set_cyclonedds(json_hw_config_data)

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

    joy_switch_node = Node(
        package='nesfr_vr_ros2',
        namespace=namespace,
        executable='nesfr_vr_joy_switch.py',
        output='both')

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

    #
    # nesfr4
    #
    nesfr4_params = PathJoinSubstitution([
            FindPackageShare('nesfr4_node'),
            'config', 'nesfr4-params.yaml'])

    nesfr4_node = Node(package='nesfr4_node',
            executable='nesfr4_node',
            output='both',
            namespace=namespace,
            parameters=[nesfr4_params])

    launch_list = [
            namespace_launch_arg,
            nesfr_vr_launch,
        ]

    if json_hw_config_data:
        if json_hw_config_data.get('base_robot'):
            logging.info("base_robot is found on hw_config.json")
            if json_hw_config_data['base_robot']['type'] == "NESFR7_ROS2":
                logging.info("NESFR7_ROS2 is found on hw_config.json")
                pass
            elif json_hw_config_data['base_robot']['type'] == "NESFR7_Arm_Only_ROS2":
                logging.info("NESFR7_Arm_Only_ROS2 is found on hw_config.json")
                launch_list.append(nesfr7_arm_only_common_launch)
            elif json_hw_config_data['base_robot']['type'] == "NESFR4_ROS2":
                logging.info("NESFR4_ROS2 is found on hw_config.json")
                launch_list.append(TimerAction(period=1.0, actions=[nesfr4_node,]))
                launch_list.append( RegisterEventHandler(
                                        OnProcessExit(
                                            target_action=nesfr4_node,
                                            on_exit=[
                                                LogInfo(msg=('nesfr4_node closed')),
                                                EmitEvent(event=Shutdown(reason='Window closed'))
                                                ]
                                            )
                                        )
                                    )
        else:
            logging.info("base_robot is not found on hw_config.json")

        return LaunchDescription(launch_list)

    #
    # default setup
    #
    logging.warn('Launch with Default Configuration')

    # nesfr7_XX
    if re.search("nesfr7_[0-9]+$", hostname):
        return LaunchDescription([
            namespace_launch_arg,
            nesfr_vr_launch,
        ])
    # nesfr7_one_XX
    elif re.search("nesfr7_one_[0-9]+$", hostname):
        return LaunchDescription([
            namespace_launch_arg,
            nesfr7_arm_common_launch,
            nesfr_vr_launch,
        ])
    # nesfr7_arm_XX
    elif re.search("nesfr7_arm_[0-9]+$", hostname):
        return LaunchDescription([
            namespace_launch_arg,
            nesfr7_arm_only_common_launch,
            nesfr_vr_launch,
            joy_switch_node,
        ])
    # nesfr4_XX
    elif re.search("nesfr4$", hostname):
        return LaunchDescription([
            namespace_launch_arg,
            nesfr_vr_launch,
            joy_switch_node,
            TimerAction(period=1.0, actions=[nesfr4_node,]),
#            RegisterEventHandler(
#                OnProcessExit(
#                    target_action=nesfr4_node,
#                    on_exit=[
#                        LogInfo(msg=('nesfr4_node closed')),
#                        EmitEvent(event=Shutdown(reason='Window closed'))
#                        ]
#                    )
#                ),
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
