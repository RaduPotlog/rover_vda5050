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

#include "rover_vda5050_adapter/infrastructure/follow_me_client.hpp"

#include <memory>

namespace rover_vda5050_adapter::infrastructure
{

using application::CommandResult;

FollowMeClient::FollowMeClient(
    rclcpp::Node & node, const std::string & start_service, const std::string & stop_service,
    std::chrono::duration<double> availability_timeout,
    std::chrono::duration<double> response_timeout)
: start_client_(node.create_client<std_srvs::srv::Trigger>(start_service)),
  stop_client_(node.create_client<std_srvs::srv::Trigger>(stop_service)),
  availability_timeout_(availability_timeout),
  response_timeout_(response_timeout)
{
}

CommandResult FollowMeClient::start()
{
    return call(*start_client_);
}

CommandResult FollowMeClient::stop()
{
    return call(*stop_client_);
}

CommandResult FollowMeClient::call(rclcpp::Client<std_srvs::srv::Trigger> & client)
{
    const std::string service = client.get_service_name();

    if (!client.wait_for_service(availability_timeout_)) {
        return {false, "'" + service + "' unavailable (is rover-a1-follow-me running?).", false};
    }

    auto future = client.async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

    if (future.wait_for(response_timeout_) != std::future_status::ready) {
        client.remove_pending_request(future);
        return {false, "'" + service + "' did not answer in time.", true};
    }

    const auto response = future.get();

    if (!response->success) {
        return {false, response->message.empty() ? "'" + service + "' refused." : response->message,
            false};
    }

    return {true, response->message, false};
}

}  // namespace rover_vda5050_adapter::infrastructure
