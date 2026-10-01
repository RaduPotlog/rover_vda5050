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

#ifndef ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_DRIVE_MODE_CLIENT_HPP_
#define ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_DRIVE_MODE_CLIENT_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/srv/set_drive_mode.hpp"

#include "rover_vda5050_adapter/application/ports.hpp"

namespace rover_vda5050_adapter::infrastructure
{

/**
 * @brief DriveModePort over drive_mode_manager's set_drive_mode service (rover_msgs/SetDriveMode).
 *
 * Blocking: waits for the service and its answer, so it must run on a thread other than the one
 * spinning the node (the adapter runs every goal on its own thread).
 */
class DriveModeClient : public application::DriveModePort
{
public:
    DriveModeClient(
        rclcpp::Node & node, const std::string & service,
        std::chrono::duration<double> availability_timeout,
        std::chrono::duration<double> response_timeout);

    application::CommandResult set(domain::DriveMode mode) override;

private:
    rclcpp::Client<rover_msgs::srv::SetDriveMode>::SharedPtr client_;
    std::chrono::duration<double> availability_timeout_;
    std::chrono::duration<double> response_timeout_;
};

}  // namespace rover_vda5050_adapter::infrastructure

#endif  // ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_DRIVE_MODE_CLIENT_HPP_
