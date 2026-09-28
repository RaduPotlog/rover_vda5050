// Copyright 2026 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rover_vda5050_adapter/infrastructure/vda_conversions.hpp"

namespace rover_vda5050_adapter::infrastructure
{

std::vector<domain::RouteNode> toRouteNodes(const std::vector<vda5050_msgs::msg::Node> & nodes)
{
    std::vector<domain::RouteNode> route;
    route.reserve(nodes.size());

    for (const auto & node : nodes) {
        route.push_back(
            domain::RouteNode{
                node.node_id, node.sequence_id,
                domain::Pose2D{
                    node.node_position.x, node.node_position.y, node.node_position.theta}});
    }

    return route;
}

vda5050_msgs::msg::AGVPosition toAgvPosition(const RoverSnapshot & snapshot)
{
    vda5050_msgs::msg::AGVPosition position;
    position.map_id = snapshot.map_id;
    position.position_initialized = snapshot.pose.has_value();

    if (snapshot.pose) {
        position.x = snapshot.pose->x;
        position.y = snapshot.pose->y;
        position.theta = snapshot.pose->theta;
    }

    return position;
}

vda5050_msgs::msg::Velocity toVelocity(const RoverSnapshot & snapshot)
{
    vda5050_msgs::msg::Velocity velocity;
    velocity.vx = snapshot.velocity.vx;
    velocity.vy = snapshot.velocity.vy;
    velocity.omega = snapshot.velocity.omega;

    return velocity;
}

vda5050_msgs::msg::BatteryState toBatteryState(const domain::VdaBattery & battery)
{
    vda5050_msgs::msg::BatteryState state;
    state.battery_charge = battery.charge_percent;
    state.battery_voltage = battery.voltage;
    state.charging = battery.charging;

    return state;
}

vda5050_msgs::msg::SafetyState toSafetyState(const domain::VdaSafety & safety)
{
    vda5050_msgs::msg::SafetyState state;
    state.e_stop = safety.e_stop;
    state.field_violation = safety.field_violation;

    return state;
}

vda5050_msgs::msg::Error toError(const domain::RoverError & error)
{
    vda5050_msgs::msg::Error msg;
    msg.error_type = error.type;
    msg.error_description = error.description;
    msg.error_level = error.fatal ? vda5050_msgs::msg::Error::FATAL :
                                    vda5050_msgs::msg::Error::WARNING;

    return msg;
}

}  // namespace rover_vda5050_adapter::infrastructure
