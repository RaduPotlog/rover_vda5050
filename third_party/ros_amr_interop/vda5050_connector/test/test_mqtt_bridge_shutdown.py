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

"""rover_vda5050: the bridge announces OFFLINE before it disconnects.

publish() only queues a message for paho's network thread. on_shutdown() used to disconnect right
after it, so the OFFLINE Connection message was dropped and the broker kept the retained ONLINE.
"""

import json

from unittest.mock import MagicMock

import pytest

from vda5050_connector_py.mqtt_bridge import MQTTBridge, SHUTDOWN_PUBLISH_TIMEOUT_S
from vda5050_msgs.msg import Connection


def _published_connection_states(mqtt_client):
    states = []
    for call in mqtt_client.publish.call_args_list:
        topic, payload = call.args[0], call.args[1]
        if topic.endswith("/connection"):
            states.append((json.loads(payload)["connectionState"], call.kwargs))
    return states


@pytest.fixture
def bridge(setup_rclpy, mock_mqtt_client):
    info = MagicMock()
    info.is_published.return_value = True
    mock_mqtt_client.publish = MagicMock(return_value=info)
    mock_mqtt_client.is_connected.return_value = True

    events = []
    info.wait_for_publish.side_effect = lambda timeout: events.append(("wait", timeout))
    mock_mqtt_client.disconnect.side_effect = lambda: events.append(("disconnect",))
    mock_mqtt_client.loop_stop.side_effect = lambda: events.append(("loop_stop",))

    node = MQTTBridge()
    yield node, mock_mqtt_client, info, events
    node.destroy_node()


def test_shutdown_publishes_offline_retained_with_qos1(bridge):
    node, mqtt_client, _, _ = bridge

    node.on_shutdown()

    states = _published_connection_states(mqtt_client)
    assert states[-1][0] == Connection.OFFLINE
    assert states[-1][1] == {"qos": 1, "retain": True}


def test_shutdown_waits_for_offline_before_disconnecting(bridge):
    node, _, _, events = bridge

    node.on_shutdown()

    assert events == [("wait", SHUTDOWN_PUBLISH_TIMEOUT_S), ("disconnect",), ("loop_stop",)]


def test_shutdown_does_not_wait_when_not_connected(bridge):
    node, mqtt_client, info, events = bridge
    mqtt_client.is_connected.return_value = False

    node.on_shutdown()

    info.wait_for_publish.assert_not_called()
    assert events == [("disconnect",), ("loop_stop",)]


def test_shutdown_still_disconnects_when_the_publish_fails(bridge):
    node, _, info, events = bridge
    info.wait_for_publish.side_effect = RuntimeError("message not queued")
    info.is_published.return_value = False

    node.on_shutdown()

    assert events == [("disconnect",), ("loop_stop",)]
