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

"""rover_vda5050: the bridge forwards the controller's factsheet to MQTT.

Upstream never subscribed to the factsheet topic, so a factsheetRequest was answered on ROS only.
"""

import json

from vda5050_connector_py.mqtt_bridge import MQTTBridge
from vda5050_msgs.msg import AGVAction, Factsheet


def test_bridge_subscribes_to_the_factsheet(setup_rclpy, mocker, mock_mqtt_client):
    mocker.patch.object(MQTTBridge, "create_subscription")
    mocker.patch.object(MQTTBridge, "create_publisher")
    bridge = MQTTBridge()

    bridge.create_subscription.assert_any_call(
        msg_type=Factsheet,
        topic="/uagv/v1/robots/robot_1/factsheet",
        callback=bridge._publish_factsheet,
        qos_profile=10,
    )


def test_factsheet_is_published_retained_in_vda5050_json(setup_rclpy, mock_mqtt_client):
    bridge = MQTTBridge()
    factsheet = Factsheet(
        header_id=3, version="2.0.0", manufacturer="robots", serial_number="robot_1")
    factsheet.protocol_features.agv_actions = [
        AGVAction(action_type="enableAuxOutput", action_scopes=["INSTANT", "NODE"])]

    bridge._publish_factsheet(factsheet)

    call = mock_mqtt_client.publish.call_args
    topic, payload = call.args[0], json.loads(call.args[1])
    assert topic == "uagv/v2/robots/robot_1/factsheet"
    assert call.kwargs == {"qos": 0, "retain": True}
    assert payload["headerId"] == 3
    assert payload["serialNumber"] == "robot_1"
    assert payload["protocolFeatures"]["agvActions"][0]["actionType"] == "enableAuxOutput"
    assert payload["protocolFeatures"]["agvActions"][0]["actionScopes"] == ["INSTANT", "NODE"]
    bridge.destroy_node()
