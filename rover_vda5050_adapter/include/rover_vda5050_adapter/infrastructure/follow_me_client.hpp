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

#ifndef ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_FOLLOW_ME_CLIENT_HPP_
#define ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_FOLLOW_ME_CLIENT_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rover_vda5050_adapter/application/ports.hpp"

namespace rover_vda5050_adapter::infrastructure
{

/**
 * @brief FollowMePort over rover_follow_me's follow_me/start and follow_me/stop
 * (std_srvs/Trigger).
 *
 * Blocking: waits for the service and its answer, so it must run on a thread other than the one
 * spinning the node (the adapter runs every goal on its own thread).
 */
class FollowMeClient : public application::FollowMePort
{
public:
    FollowMeClient(
        rclcpp::Node & node, const std::string & start_service, const std::string & stop_service,
        std::chrono::duration<double> availability_timeout,
        std::chrono::duration<double> response_timeout);

    application::CommandResult start() override;
    application::CommandResult stop() override;

private:
    application::CommandResult call(rclcpp::Client<std_srvs::srv::Trigger> & client);

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
    std::chrono::duration<double> availability_timeout_;
    std::chrono::duration<double> response_timeout_;
};

}  // namespace rover_vda5050_adapter::infrastructure

#endif  // ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_FOLLOW_ME_CLIENT_HPP_
