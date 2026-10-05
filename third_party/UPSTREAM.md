# Vendored upstream code

## ros_amr_interop

| | |
|---|---|
| Upstream | https://github.com/inorbit-ai/ros_amr_interop |
| Branch | `humble-devel` |
| Commit | `ad0674d516a59371a3c90594e72cc5b622ea4fc3` (2026-08-29) |
| License | BSD-3-Clause (`ros_amr_interop/vda5050_connector/LICENSE`) |
| VDA 5050 | 2.0.0 (and 1.1.0), MQTT topics `uagv/v2/<manufacturer>/<serialNumber>/<topic>` |
| Vendored as | `git subtree --squash` at `third_party/ros_amr_interop` |

Packages built here: `vda5050_msgs`, `vda5050_serializer`, `vda5050_connector`.
Not built (`COLCON_IGNORE`, left in place so `git subtree pull` stays clean):
`massrobotics_amr_sender_py`, `rmf_inorbit_fleet_adapter`.

### Local patches

Every change to upstream files is marked with a `rover_vda5050` comment. Keep this list in sync;
re-check each entry after a `git subtree pull`.

| File | Change | Why |
|---|---|---|
| `vda5050_connector/CMakeLists.txt` | `ament_target_dependencies()` → `target_link_libraries()` on imported targets; export the linked packages with `ament_export_dependencies()` | `ament_target_dependencies` is removed in lyrical; downstream plugin packages need the dependencies re-found |
| `vda5050_connector/test/adapter/CMakeLists.txt` | Same as above for the gtest targets | Same |
| `vda5050_connector/src/utils.cpp` | `tf2/LinearMath/Quaternion.h` → `.hpp` | The `.h` compatibility headers are removed in lyrical |
| `vda5050_connector/vda5050_connector_py/mqtt_bridge.py` | `mqtt_client.Client(CallbackAPIVersion.VERSION1)` | paho-mqtt 2.x (Ubuntu 26.04's `python3-paho-mqtt`) makes the callback API version mandatory |
| `vda5050_connector/vda5050_connector_py/mqtt_bridge.py` | Forward the controller's `factsheet` to MQTT, retained (+ `test/test_mqtt_bridge_factsheet.py`, one assert in `test_vda5050_mqtt_bridge.py`) | The bridge never subscribed to it, so `factsheetRequest` was answered on ROS only and master control never got a factsheet |
| `vda5050_connector/vda5050_connector_py/mqtt_bridge.py` | Publish `connection` with QoS 1, retained | VDA 5050 requires it, and the last will is retained: a master subscribing after a reconnect otherwise read a stale `CONNECTIONBROKEN` |
| `vda5050_connector/vda5050_connector_py/vda5050_controller.py` | Copy the adapter's `safety_state` into the published state | Upstream drops it, so an e-stop never reached master control |
| `vda5050_connector/vda5050_connector_py/vda5050_controller.py` | `_process_node` appends the node's actions to `_current_node_actions` instead of replacing them | One navigation feedback can pass several nodes; replacing dropped the NONE actions (e.g. `enableAuxOutput`) of all but the last |
| `vda5050_connector/vda5050_connector_py/mqtt_bridge.py`, `vda5050_connector/scripts/mqtt_bridge.py` | `on_shutdown` waits up to 2 s for the broker to acknowledge the OFFLINE `connection`, then `loop_stop()`s; the script maps SIGTERM to SIGINT (+ `test/test_mqtt_bridge_shutdown.py`, registered in `vda5050_connector/CMakeLists.txt`) | `disconnect()` dropped the queued OFFLINE, leaving the retained ONLINE on the broker; under rmw_zenoh_cpp a SIGTERM never ended `spin()`, so a container stop hung until SIGKILL |
| `vda5050_connector/vda5050_connector_py/mqtt_bridge.py` | `VDA5050_CONNECTOR_TLS` (`auto`/`true`/`false`) decides TLS; `auto` keeps upstream's "TLS when a user name is set" (+ `test/test_mqtt_bridge_tls.py`) | A broker reached through a VPN needs user/password without TLS |
| `vda5050_connector/vda5050_connector_py/mqtt_bridge.py`, `vda5050_connector_py/vda5050_controller.py` | `logger.warn(` → `logger.warning(` (all call sites; mechanical, not marked) | `warn` is gone from the lyrical rclpy logger, so these paths crashed |
| `vda5050_serializer/setup.cfg` | `script-dir`/`install-scripts` → `script_dir`/`install_scripts` | Dash-separated keys are rejected by current setuptools |
| `vda5050_connector/package.xml` | Second `<license>Apache-2.0</license>` | The local test files (`test/test_mqtt_bridge_factsheet.py`, `_shutdown.py`, `_tls.py`) are Apache-2.0, not BSD |
| `vda5050_msgs/LICENSE`, `vda5050_serializer/LICENSE` (new files) | Copies of `ros_amr_interop/LICENSE` (BSD) and of the Apache-2.0 text | Upstream ships no license text in these packages; `vda5050_serializer` declares Apache-2.0, unlike the rest of the repo |

### Updating

```bash
git subtree pull --squash --prefix third_party/ros_amr_interop \
  https://github.com/inorbit-ai/ros_amr_interop humble-devel
```
Resolve conflicts against the table above, then rebuild and run the tests.
