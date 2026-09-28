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

#ifndef ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_MISSION_MANAGER_CLIENT_HPP_
#define ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_MISSION_MANAGER_CLIENT_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/srv/set_mission.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rover_vda5050_adapter/application/ports.hpp"

namespace rover_vda5050_adapter::infrastructure
{

/**
 * @brief MissionPort over rover_mission_manager's set_mission / run_mission services.
 *
 * Blocking: each call waits for the service and its answer, so it must run on a thread other than
 * the one spinning the node (the adapter runs every goal on its own thread).
 */
class MissionManagerClient : public application::MissionPort
{
public:
    MissionManagerClient(
        rclcpp::Node & node, const std::string & set_mission_service,
        const std::string & run_mission_service, std::string goal_frame,
        std::chrono::duration<double> availability_timeout,
        std::chrono::duration<double> response_timeout);

    application::CommandResult dispatch(
        const std::string & mission_id,
        const std::vector<domain::Pose2D> & waypoints) override;

    application::CommandResult cancel() override;

private:
    template<typename ServiceT>
    application::CommandResult call(
        const typename rclcpp::Client<ServiceT>::SharedPtr & client,
        const typename ServiceT::Request::SharedPtr & request);

    rclcpp::Node & node_;
    rclcpp::Client<rover_msgs::srv::SetMission>::SharedPtr set_mission_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr run_mission_client_;
    std::string goal_frame_;
    std::chrono::duration<double> availability_timeout_;
    std::chrono::duration<double> response_timeout_;
};

}  // namespace rover_vda5050_adapter::infrastructure

#endif  // ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_MISSION_MANAGER_CLIENT_HPP_
