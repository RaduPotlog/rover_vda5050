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

"""rover_vda5050: VDA5050_CONNECTOR_TLS decides TLS independently of the broker user.

Upstream enables TLS whenever a user name is set. A broker reached through a VPN needs
user/password without TLS, so VDA5050_CONNECTOR_TLS=false turns it off; "auto" keeps upstream.
"""

import ssl

import pytest

from vda5050_connector_py.mqtt_bridge import MQTTBridge, use_tls

PARAMS = {
    "mqtt_address": "10.8.0.1",
    "mqtt_username": "rover_a1",
    "mqtt_password": "secret",
}


@pytest.fixture
def bridge_with_user(setup_rclpy, mocker, mock_mqtt_client):
    mocker.patch(
        "vda5050_connector_py.mqtt_bridge.read_str_parameter",
        side_effect=lambda node, name, alternative: PARAMS.get(name, alternative),
    )
    mock_mqtt_client.is_connected.return_value = False
    nodes = []

    def make():
        nodes.append(MQTTBridge())
        return nodes[-1].mqtt_client

    yield make
    for node in nodes:
        node.destroy_node()


def test_user_without_tls_when_disabled(bridge_with_user, monkeypatch):
    monkeypatch.setenv("VDA5050_CONNECTOR_TLS", "false")

    mqtt_client = bridge_with_user()

    mqtt_client.tls_set.assert_not_called()
    mqtt_client.username_pw_set.assert_called_once_with(username="rover_a1", password="secret")
    mqtt_client.connect_async.assert_called_with(host="10.8.0.1", port=1883)


def test_user_enables_tls_by_default(bridge_with_user, monkeypatch):
    monkeypatch.delenv("VDA5050_CONNECTOR_TLS", raising=False)
    monkeypatch.delenv("VDA5050_CONNECTOR_TLS_CA_CERT", raising=False)

    mqtt_client = bridge_with_user()

    mqtt_client.tls_set.assert_called_once_with(
        ca_certs="/etc/ssl/certs/ca-certificates.crt", tls_version=ssl.PROTOCOL_TLSv1_2
    )
    mqtt_client.username_pw_set.assert_called_once_with(username="rover_a1", password="secret")


@pytest.mark.parametrize(
    "username, mode, expected",
    [
        ("", "auto", False),
        ("user", "auto", True),
        ("user", "false", False),
        ("user", " OFF ", False),
        ("", "true", True),
        ("user", "1", True),
    ],
)
def test_use_tls(username, mode, expected):
    assert use_tls(username, mode) is expected


def test_use_tls_rejects_unknown_mode():
    with pytest.raises(ValueError):
        use_tls("user", "maybe")
