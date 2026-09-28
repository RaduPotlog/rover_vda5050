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

#include "rover_vda5050_adapter/infrastructure/aux_output_client.hpp"

#include <memory>

namespace rover_vda5050_adapter::infrastructure
{

using application::CommandResult;

AuxOutputClient::AuxOutputClient(
    rclcpp::Node & node, const std::string & service_prefix, int count,
    std::chrono::duration<double> availability_timeout,
    std::chrono::duration<double> response_timeout)
: availability_timeout_(availability_timeout),
  response_timeout_(response_timeout)
{
    for (int i = 0; i < count; ++i) {
        clients_.push_back(
            node.create_client<std_srvs::srv::SetBool>(
                service_prefix + std::to_string(i) + "/set"));
    }
}

CommandResult AuxOutputClient::set(int index, bool enabled)
{
    if (index < 0 || index >= static_cast<int>(clients_.size())) {
        return {false, "no aux output with index " + std::to_string(index) + ".", false};
    }

    const auto & client = clients_[index];
    const std::string service = client->get_service_name();

    if (!client->wait_for_service(availability_timeout_)) {
        return {false, "'" + service + "' unavailable (is the rover hardware interface running?).",
            false};
    }

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enabled;

    auto future = client->async_send_request(request);

    if (future.wait_for(response_timeout_) != std::future_status::ready) {
        client->remove_pending_request(future);
        return {false, "'" + service + "' did not answer in time.", true};
    }

    const auto response = future.get();

    if (!response->success && response->message.empty()) {
        return {false, "'" + service + "' refused the write.", false};
    }

    return {response->success, response->message, false};
}

}  // namespace rover_vda5050_adapter::infrastructure
