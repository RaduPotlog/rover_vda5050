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

#ifndef ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_VDA_CONVERSIONS_HPP_
#define ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_VDA_CONVERSIONS_HPP_

#include <vector>

#include "vda5050_msgs/msg/agv_position.hpp"
#include "vda5050_msgs/msg/battery_state.hpp"
#include "vda5050_msgs/msg/error.hpp"
#include "vda5050_msgs/msg/node.hpp"
#include "vda5050_msgs/msg/safety_state.hpp"
#include "vda5050_msgs/msg/velocity.hpp"

#include "rover_vda5050_adapter/domain/rover_status.hpp"
#include "rover_vda5050_adapter/domain/types.hpp"
#include "rover_vda5050_adapter/infrastructure/rover_link.hpp"

namespace rover_vda5050_adapter::infrastructure
{

/// Order nodes as route nodes. VDA 5050 positions are taken to be in the navigation frame.
std::vector<domain::RouteNode> toRouteNodes(const std::vector<vda5050_msgs::msg::Node> & nodes);

vda5050_msgs::msg::AGVPosition toAgvPosition(const RoverSnapshot & snapshot);
vda5050_msgs::msg::Velocity toVelocity(const RoverSnapshot & snapshot);
vda5050_msgs::msg::BatteryState toBatteryState(const domain::VdaBattery & battery);
vda5050_msgs::msg::SafetyState toSafetyState(const domain::VdaSafety & safety);
vda5050_msgs::msg::Error toError(const domain::RoverError & error);

}  // namespace rover_vda5050_adapter::infrastructure

#endif  // ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_VDA_CONVERSIONS_HPP_
