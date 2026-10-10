# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""VDA 5050 connector for Rover A1: MQTT bridge, controller and rover adapter."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ('true', '1', 'yes', 'on')


def _launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    rover_namespace = arg('namespace').strip('/')
    # The connector's nodes live beside the rover's, not among them: <namespace>/vda5050.
    connector_namespace = f'{rover_namespace}/vda5050' if rover_namespace else 'vda5050'

    manufacturer = arg('manufacturer')
    serial_number = arg('serial_number')
    use_sim_time = _as_bool(arg('use_sim_time'))
    params_file = arg('params_file')

    identity = {
        'manufacturer_name': manufacturer,
        'serial_number': serial_number,
        'use_sim_time': use_sim_time,
    }

    mqtt_bridge = Node(
        package='vda5050_connector',
        executable='mqtt_bridge.py',
        name='mqtt_bridge',
        namespace=connector_namespace,
        output='screen',
        parameters=[
            params_file,
            {
                **identity,
                'mqtt_address': arg('broker_host'),
                'mqtt_port': int(arg('broker_port')),
                'mqtt_username': arg('broker_username'),
                'mqtt_password': arg('broker_password'),
            },
        ],
    )

    controller = Node(
        package='vda5050_connector',
        executable='vda5050_controller.py',
        name='controller',
        namespace=connector_namespace,
        output='screen',
        parameters=[params_file, {**identity, 'robot_name': serial_number}],
    )

    adapter = Node(
        package='rover_vda5050_adapter',
        executable='adapter_node',
        name='adapter',
        namespace=connector_namespace,
        output='screen',
        parameters=[
            params_file,
            {
                **identity,
                'robot_name': serial_number,
                'rover.namespace': rover_namespace,
                'rover.map_frame': arg('map_frame'),
            },
        ],
    )

    return [mqtt_bridge, controller, adapter]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('rover_vda5050_bringup'), 'config', 'connector.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value=os.environ.get('ROVER_SYSTEM_NAMESPACE', 'rover'),
            description='Rover namespace; the connector runs under <namespace>/vda5050.'),
        DeclareLaunchArgument(
            'broker_host', default_value='127.0.0.1', description='MQTT broker host.'),
        DeclareLaunchArgument(
            'broker_port', default_value='1883', description='MQTT broker port.'),
        DeclareLaunchArgument(
            'broker_username', default_value='',
            description='MQTT user; non-empty also enables TLS (CA from '
                        'VDA5050_CONNECTOR_TLS_CA_CERT).'),
        DeclareLaunchArgument(
            'broker_password', default_value='', description='MQTT password.'),
        DeclareLaunchArgument(
            'manufacturer', default_value='MechatronicsAcademy',
            description='VDA 5050 manufacturer (MQTT topic level).'),
        DeclareLaunchArgument(
            'serial_number', default_value='rover_a1',
            description='VDA 5050 serialNumber (MQTT topic level): A-Z a-z 0-9 _ . : -'),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description="Nav 2 global frame the order coordinates are in ('odom' with "
                        'localization_source odom); prefixed with <namespace>/.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use /clock (Gazebo).'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Connector parameters (config/connector.yaml).'),
        OpaqueFunction(function=_launch_setup),
    ])
