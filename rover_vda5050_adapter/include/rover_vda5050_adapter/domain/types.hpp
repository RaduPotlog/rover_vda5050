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

#ifndef ROVER_VDA5050_ADAPTER_DOMAIN_TYPES_HPP_
#define ROVER_VDA5050_ADAPTER_DOMAIN_TYPES_HPP_

#include <cstdint>
#include <string>

namespace rover_vda5050_adapter::domain
{

/** @brief Planar pose in the navigation frame: metres and radians. */
struct Pose2D
{
    double x{0.0};
    double y{0.0};
    double theta{0.0};
};

/** @brief A VDA 5050 order node the rover has to drive to. */
struct RouteNode
{
    std::string node_id;
    std::uint32_t sequence_id{0};
    Pose2D pose;
};

/** @brief How the heading of each waypoint is chosen. */
enum class OrientationMode
{
    /// Use the node's theta as given. VDA 5050 theta is optional on the wire, but the ROS message
    /// cannot tell "absent" from 0.0, so this makes a master that omits theta spin the rover to 0.
    kNode,
    /// Face along the edge the rover arrives on. Suits a skid-steer rover that turns in place
    /// poorly; the node's theta is used only when there is no edge to take a heading from.
    kPath,
};

/** @brief rover_mission_manager's view of the mission it runs (mirrors rover_msgs/MissionState). */
enum class MissionPhase
{
    kIdle,
    kRunning,
    kHeld,        ///< Suspended by the motion lock or a dead lidar; resumes on its own.
    kSucceeded,
    kFailed,
    kCancelled,
};

/** @brief One rover_msgs/MissionState sample. */
struct MissionProgress
{
    std::string mission_id;
    MissionPhase phase{MissionPhase::kIdle};
    std::uint32_t current_index{0};   ///< Waypoint being driven to; equals total once done.
    std::uint32_t total{0};
    std::string message;
};

}  // namespace rover_vda5050_adapter::domain

#endif  // ROVER_VDA5050_ADAPTER_DOMAIN_TYPES_HPP_
