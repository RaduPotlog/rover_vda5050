# rover_vda5050

VDA 5050 fleet interface for Rover A1: a master control sends orders over MQTT, and the rover
drives them through `rover_mission_manager` and Nav 2. It runs as the `rover-a1-vda5050` container
of [`rover_docker`](https://github.com/RaduPotlog/rover_docker).

It is built on an existing implementation rather than written from scratch: InOrbit's
[`ros_amr_interop`](https://github.com/inorbit-ai/ros_amr_interop) connector (BSD-3), vendored
under `third_party/`. This repository adds only the rover-specific adapter and the bringup.

**Protocol version: VDA 5050 2.0.0** (MQTT topics `uagv/v2/<manufacturer>/<serialNumber>/<topic>`).
No mature open-source robot-side connector implements 3.0 yet (surveyed 2026-09-28). A 3.0 master
control needs a later upgrade.

## Packages

| Package | What |
|---------|------|
| `third_party/ros_amr_interop/vda5050_msgs` | VDA 5050 2.0 messages (upstream) |
| `third_party/ros_amr_interop/vda5050_serializer` | ROS ↔ JSON (snake_case ↔ camelCase) (upstream) |
| `third_party/ros_amr_interop/vda5050_connector` | `mqtt_bridge` (MQTT ↔ ROS), `controller` (order validation, base/horizon, stitching, action states, state/visualization/connection/factsheet) and the adapter node framework (pluginlib handlers) (upstream) |
| `rover_vda5050_adapter` | The rover's handler plugins for that adapter, plus its `adapter_node` executable |
| `rover_vda5050_bringup` | `vda5050.launch.py`, `config/connector.yaml`, `config/mosquitto.conf`, `scripts/fake_master.py` |
| `rover_vda5050` | Metapackage |

`third_party/UPSTREAM.md` lists the upstream commit and every local patch: the lyrical port, the
paho-mqtt 2 API, `connection` retained with QoS 1, and forwarding `safetyState`.

## How it drives the rover

```
master control ──MQTT── mosquitto ── mqtt_bridge ⇄ controller ──NavigateThroughNodes / ProcessVDAAction / GetState──► adapter
                                                                                                                         │ rover_vda5050_adapter plugins
                                             rover_mission_manager ◄── set_mission / run_mission / mission_state ───────┘
                                                      │ navigate_to_pose, one waypoint at a time
                                                    Nav 2
```

Orders go through `rover_mission_manager` rather than straight to Nav 2. That way every guard
the drive UI's GoTo has applies to them too: missions run only in the **Automatic** driving mode,
hold on the motion lock or a dead lidar, and abort on low battery. Nothing else sends Nav 2 goals
behind the manager's back.

| VDA 5050 | Rover |
|----------|-------|
| Released base segment (`NavigateThroughNodes`) | One `set_mission` with the segment's nodes as waypoints, mission id `vda5050-<n>` |
| Order stitching (update extending the base) | `set_mission` with the nodes not yet reached plus the new ones (the manager replaces the mission in place) |
| Node reached | `mission_state.current_index` advancing |
| `cancelOrder` | `run_mission false` |
| `startPause` / `stopPause` | `run_mission false` and keep the route / `set_mission` with the nodes not yet reached (the manager has no pause) |
| `stateRequest`, `factsheetRequest` | Answered by the upstream controller |
| `operatingMode` | Drive mode AUTOMATIC → `AUTOMATIC`; ASSISTED and MANUAL → `MANUAL` (an operator drives; `SEMIAUTOMATIC` would mean master control's orders run); no drive-mode manager → `SERVICE` |
| `paused` | `startPause`, or the mission manager holding the mission (motion lock, dead lidar) |
| `agvPosition` | TF `<ns>/<map_frame>` → `<ns>/base_link`; `positionInitialized` false when missing or older than 2 s. `mapId` = the indoor map in use, else `rover.map_id` |
| `velocity` | `odom` twist |
| `batteryState` | `rover_battery/battery_status` (charge 0 when the BMS does not know) |
| `safetyState.eStop` | `MANUAL` while the e-stop button is pressed or the safety PLC latch is set, else `NONE` |
| `safetyState.fieldViolation` | The drive mode's collision monitor is in its stop zone |
| `errors` (all `WARNING`) | `missionRefused` (why the manager refused the last order), `motionLocked`, `localizationUnavailable`, `safetyLinkDown` |

Rover errors are never `FATAL`: the upstream controller stops dispatching navigation while any
`FATAL` error is present. A failed or refused order still ends with the controller's own `FATAL`
`noRouteError`, and master control has to `cancelOrder` it before sending a new order.

### Decisions worth knowing

- **Node headings** (`rover.orientation_mode`, default `path`): the rover arrives at each node
  facing along the edge it drove in on, and the node's `theta` is ignored. A skid-steer rover
  turns in place poorly, and the ROS message cannot tell an omitted `theta` from `0.0`: honouring
  it would spin the rover to 0 rad at every node of a master control that leaves `theta` out. Set
  `node` to use each node's `theta`.
- **Frame:** node coordinates are taken to be in Nav 2's global frame (`rover.map_frame`, the
  container derives it from the localization source). The mission manager refuses missions in
  any other frame, with a message that ends up in `missionRefused`.
- **Refused orders** do not stop a mission the rover is already running for someone else (e.g.
  an operator's GoTo). Only a mission this connector dispatched, or one whose `set_mission` timed
  out, is cancelled.
- **Not supported:** node and edge actions other than the two pause actions, `initPosition`,
  edge `maxSpeed` and trajectories, loads, and the `allowedDeviation*` tolerances (the mission
  manager uses Nav 2's goal checker). An unsupported action is forwarded to the adapter, which
  rejects it, and it shows up as `FAILED` in `actionStates`.
- **Upstream limits:** order validation and the first-node deviation check are stubs, so any
  well-formed order and any start node are accepted. A stitch arriving just after a segment
  finished is acknowledged but not driven.
- A stitch re-sends the node being driven to, so Nav 2 re-plans once.

## Running

In the container, `ROVER_START_VDA5050=true` is all it takes (see rover_docker's README, "VDA
5050"). Outside it:

```bash
mosquitto -c $(ros2 pkg prefix rover_vda5050_bringup)/share/rover_vda5050_bringup/config/mosquitto.conf &
ros2 launch rover_vda5050_bringup vda5050.launch.py namespace:=rover map_frame:=map
```

Launch arguments: `namespace`, `broker_host`, `broker_port`, `broker_username`, `broker_password`,
`manufacturer`, `serial_number`, `map_frame`, `use_sim_time`, `params_file`.

`rover_mission_manager` must run with the same frame (`localization_source` gps/slam/amcl/indoor
→ `map`, odom → `odom`), and the rover must be in Automatic.

## Testing

```bash
colcon build --packages-up-to rover_vda5050 --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rover_vda5050_adapter && colcon test-result --verbose
```

Unit tests cover the route tracking (progress, stitching, pause/resume, other missions' states),
the command use case (refusals, timeouts, cancel) and the state mapping. A domain-purity check
fails the build if `domain/` includes a ROS header.

`scripts/fake_master.py` is a minimal master control for manual tests (needs `python3-paho-mqtt`):

```bash
ros2 run rover_vda5050_bringup fake_master.py watch                  # print connection / state
ros2 run rover_vda5050_bringup fake_master.py order 2,0 4,0 4,2      # from the current position
ros2 run rover_vda5050_bringup fake_master.py order --order-id <id> --update 1 --from-node n4 6,2  # stitch
ros2 run rover_vda5050_bringup fake_master.py pause | resume | cancel | factsheet
```

For a real master control, NVIDIA's
[Mission Dispatch](https://github.com/nvidia-isaac/isaac_mission_dispatch) and
[openTCS](https://github.com/openTCS/opentcs) both speak 2.0. The
[vda5050_visualizer](https://github.com/bekirbostanci/vda5050_visualizer) watches the traffic over
the broker's WebSockets port (9001).

Verified 2026-09-28 against the real `rover_mission_manager` with a stand-in for the rover (drive
mode, motion lock, TF, battery and a fake `navigate_to_pose`):
- a three-node order, driven node by node;
- `startPause` / `stopPause` mid-route, resuming at the next node;
- `cancelOrder`;
- in-flight stitching;
- the refusal in Assisted mode, including `fieldViolation`.

Not yet run in Gazebo or on the rover.
