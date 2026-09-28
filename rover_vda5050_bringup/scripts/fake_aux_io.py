#!/usr/bin/env python3
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

"""
Stand-in for the rover hardware interface's aux IO, for testing the VDA 5050 aux actions
where there is no safety PLC (Gazebo, WSL): gz_ros2_control replaces RoverA1System, so the
simulation has neither the services nor the state topic.

    ros2 run rover_vda5050_bringup fake_aux_io.py --ros-args -r __ns:=/rover
    ros2 run rover_vda5050_bringup fake_aux_io.py --ros-args -r __ns:=/rover -p fail_output:=3

Serves hardware_interface/aux_output_<0..5>/set (std_srvs/SetBool) and publishes
hardware_interface/aux_io_state (rover_msgs/AuxIoState) at 20 Hz, with the same QoS as the
hardware interface. fail_output (1..6, 0 = none) makes that output's writes fail.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rover_msgs.msg import AuxIoState
from std_srvs.srv import SetBool

AUX_OUTPUT_COUNT = 6


class FakeAuxIo(Node):

    def __init__(self):
        super().__init__('fake_aux_io')
        self._fail_output = self.declare_parameter('fail_output', 0).value
        self._outputs = [False] * AUX_OUTPUT_COUNT
        self._set_services = [
            self.create_service(
                SetBool, f'hardware_interface/aux_output_{i}/set',
                lambda request, response, i=i: self._on_set(i, request, response))
            for i in range(AUX_OUTPUT_COUNT)
        ]
        self._state_pub = self.create_publisher(
            AuxIoState, 'hardware_interface/aux_io_state',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._state_timer = self.create_timer(0.05, self._publish_state)
        self.get_logger().info(f'Fake aux IO: {AUX_OUTPUT_COUNT} outputs, all off.')

    def _on_set(self, index: int, request, response):
        if index + 1 == self._fail_output:
            response.success = False
            response.message = f'fake_aux_io: output {index + 1} set to fail'
            self.get_logger().warning(response.message)
            return response
        self._outputs[index] = request.data
        response.success = True
        response.message = ''
        self.get_logger().info(
            f"Aux output {index + 1} (DIO0{index}) {'ON' if request.data else 'OFF'}; "
            f'outputs {self._outputs}')
        return response

    def _publish_state(self):
        now = self.get_clock().now().to_msg()
        msg = AuxIoState()
        msg.header.stamp = now
        msg.io_sample_time = now
        msg.outputs = list(self._outputs)
        msg.inputs = [False] * AUX_OUTPUT_COUNT
        msg.link_healthy = True
        self._state_pub.publish(msg)


def main():
    rclpy.init()
    node = FakeAuxIo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
