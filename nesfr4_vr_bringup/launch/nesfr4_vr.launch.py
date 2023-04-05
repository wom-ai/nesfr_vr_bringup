import os
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

import ament_index_python.packages

os.environ['RCUTILS_COLORIZED_OUTPUT'] = '1'

#
# references
#  - https://pypi.org/project/netifaces/
#  - https://github.com/eclipse-cyclonedds/cyclonedds
#
def set_cyclonedds():
    # FIXME
    with open("/etc/nesfrvr/config/hw_config.json", "r") as json_file:
        json_data = json.load(json_file)
        print(json_data['network']['mac_addr'])

    for iface in netifaces.interfaces():
        ifaddresses = netifaces.ifaddresses(iface)
        ret = ifaddresses.get(netifaces.AF_LINK, None)
        if ret is not None and len(ret) > 0:
            ret = ret[0].get('addr', None)
            if ret is not None and ret == json_data['network']['mac_addr']:
                os.environ['CYCLONEDDS_URI'] = '<CycloneDDS><Domain><General><NetworkInterfaceAddress>{}</></></></>'.format(iface)
                print("[INFO] interface_name={} mac_addr={}".format(iface, ret))

    print('[INFO] export CYCLONEDDS_URI=', os.environ['CYCLONEDDS_URI'])
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
