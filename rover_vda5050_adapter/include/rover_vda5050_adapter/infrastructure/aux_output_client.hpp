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

#ifndef ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_AUX_OUTPUT_CLIENT_HPP_
#define ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_AUX_OUTPUT_CLIENT_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rover_vda5050_adapter/application/ports.hpp"

namespace rover_vda5050_adapter::infrastructure
{

/**
 * @brief AuxOutputPort over the hardware interface's `<prefix><i>/set` SetBool services.
 *
 * The service answers once the safety PLC has acknowledged the write. Blocking: each call waits
 * for the service and its answer, so it must run on a thread other than the one spinning the
 * node (the adapter runs every goal on its own thread).
 */
class AuxOutputClient : public application::AuxOutputPort
{
public:
    /// @param service_prefix Resolved name up to the index (/rover/hardware_interface/aux_output_).
    AuxOutputClient(
        rclcpp::Node & node, const std::string & service_prefix, int count,
        std::chrono::duration<double> availability_timeout,
        std::chrono::duration<double> response_timeout);

    application::CommandResult set(int index, bool enabled) override;

private:
    std::vector<rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr> clients_;
    std::chrono::duration<double> availability_timeout_;
    std::chrono::duration<double> response_timeout_;
};

}  // namespace rover_vda5050_adapter::infrastructure

#endif  // ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_AUX_OUTPUT_CLIENT_HPP_
