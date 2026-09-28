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

#include "rover_vda5050_adapter/infrastructure/mission_manager_client.hpp"

#include <cmath>
#include <memory>
#include <utility>

namespace rover_vda5050_adapter::infrastructure
{

using application::CommandResult;

MissionManagerClient::MissionManagerClient(
    rclcpp::Node & node, const std::string & set_mission_service,
    const std::string & run_mission_service, std::string goal_frame,
    std::chrono::duration<double> availability_timeout,
    std::chrono::duration<double> response_timeout)
: node_(node),
  set_mission_client_(node.create_client<rover_msgs::srv::SetMission>(set_mission_service)),
  run_mission_client_(node.create_client<std_srvs::srv::SetBool>(run_mission_service)),
  goal_frame_(std::move(goal_frame)),
  availability_timeout_(availability_timeout),
  response_timeout_(response_timeout)
{
}

CommandResult MissionManagerClient::dispatch(
    const std::string & mission_id, const std::vector<domain::Pose2D> & waypoints)
{
    auto request = std::make_shared<rover_msgs::srv::SetMission::Request>();
    request->mission_id = mission_id;
    request->waypoints.reserve(waypoints.size());

    const auto stamp = node_.now();

    for (const auto & waypoint : waypoints) {
        geometry_msgs::msg::PoseStamped pose;
        // Explicit, so a manager running in another frame (e.g. odom-only navigation) refuses the
        // mission with a message instead of driving VDA map coordinates in the wrong frame.
        pose.header.frame_id = goal_frame_;
        pose.header.stamp = stamp;
        pose.pose.position.x = waypoint.x;
        pose.pose.position.y = waypoint.y;
        pose.pose.orientation.z = std::sin(waypoint.theta / 2.0);
        pose.pose.orientation.w = std::cos(waypoint.theta / 2.0);
        request->waypoints.push_back(pose);
    }

    return call<rover_msgs::srv::SetMission>(set_mission_client_, request);
}

CommandResult MissionManagerClient::cancel()
{
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;

    return call<std_srvs::srv::SetBool>(run_mission_client_, request);
}

template<typename ServiceT>
CommandResult MissionManagerClient::call(
    const typename rclcpp::Client<ServiceT>::SharedPtr & client,
    const typename ServiceT::Request::SharedPtr & request)
{
    const std::string service = client->get_service_name();

    if (!client->wait_for_service(availability_timeout_)) {
        return {false, "rover_mission_manager is not running ('" + service + "' unavailable).",
            false};
    }

    auto future = client->async_send_request(request);

    if (future.wait_for(response_timeout_) != std::future_status::ready) {
        client->remove_pending_request(future);
        return {false, "rover_mission_manager did not answer '" + service + "' in time.", true};
    }

    const auto response = future.get();

    return {response->success, response->message, false};
}

}  // namespace rover_vda5050_adapter::infrastructure
