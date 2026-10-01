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

#include "rover_vda5050_adapter/infrastructure/drive_mode_client.hpp"

#include <memory>

#include "rover_msgs/msg/drive_mode.hpp"

namespace rover_vda5050_adapter::infrastructure
{

using application::CommandResult;

DriveModeClient::DriveModeClient(
    rclcpp::Node & node, const std::string & service,
    std::chrono::duration<double> availability_timeout,
    std::chrono::duration<double> response_timeout)
: client_(node.create_client<rover_msgs::srv::SetDriveMode>(service)),
  availability_timeout_(availability_timeout),
  response_timeout_(response_timeout)
{
}

CommandResult DriveModeClient::set(domain::DriveMode mode)
{
    using rover_msgs::msg::DriveMode;

    auto request = std::make_shared<rover_msgs::srv::SetDriveMode::Request>();

    switch (mode) {
        case domain::DriveMode::kManual:
            request->mode = DriveMode::MANUAL;
            break;
        case domain::DriveMode::kAssisted:
            request->mode = DriveMode::ASSISTED;
            break;
        case domain::DriveMode::kAutomatic:
            request->mode = DriveMode::AUTOMATIC;
            break;
        case domain::DriveMode::kUnknown:
            return {false, "no drive mode to request.", false};
    }

    const std::string service = client_->get_service_name();

    if (!client_->wait_for_service(availability_timeout_)) {
        return {false, "'" + service + "' unavailable (is rover_drive_mode running?).", false};
    }

    auto future = client_->async_send_request(request);

    if (future.wait_for(response_timeout_) != std::future_status::ready) {
        client_->remove_pending_request(future);
        return {false, "'" + service + "' did not answer in time.", true};
    }

    const auto response = future.get();

    if (!response->success) {
        return {false, response->message.empty() ? "'" + service + "' refused the mode." :
            response->message, false};
    }

    return {true, response->message, false};
}

}  // namespace rover_vda5050_adapter::infrastructure
