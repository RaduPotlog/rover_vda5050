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

#ifndef ROVER_VDA5050_ADAPTER_PLUGINS_INSTANT_ROVER_ACTION_HPP_
#define ROVER_VDA5050_ADAPTER_PLUGINS_INSTANT_ROVER_ACTION_HPP_

#include <exception>
#include <memory>
#include <string>

#include "vda5050_connector/vda_action.hpp"

#include "rover_vda5050_adapter/application/ports.hpp"
#include "rover_vda5050_adapter/infrastructure/rover_link.hpp"

namespace rover_vda5050_adapter::plugins
{

/**
 * @brief Base for the actions that complete in one blocking call.
 *
 * The connector's VDAAction::execute() loops over the action state until FINISHED or FAILED, on
 * a thread of its own. The goal is only ever terminated here: the connector's controller learns
 * an action's final status from the action result, which the upstream template never sends.
 */
class InstantRoverAction : public adapter::VDAAction
{
public:
    void configure() override { link_ = infrastructure::RoverLink::forNode(node_); }

    void initialize() override
    {
        application::CommandResult result;

        // The connector's action thread only catches std::out_of_range (and reports it as an
        // unsupported action): anything else escaping here would terminate the adapter.
        try {
            result = command();
        } catch (const std::exception & error) {
            result = {false, std::string("Internal error: ") + error.what(), false};
        } catch (...) {
            result = {false, "Internal error.", false};
        }

        current_action_msg_.result_description = result.message;
        update_action_state(result.ok ? STATES::FINISHED : STATES::FAILED);
    }

    void finish() override { goal_handle_->succeed(result_); }
    void fail() override { goal_handle_->abort(result_); }

protected:
    virtual application::CommandResult command() = 0;

    std::shared_ptr<infrastructure::RoverLink> link_;
};

}  // namespace rover_vda5050_adapter::plugins

#endif  // ROVER_VDA5050_ADAPTER_PLUGINS_INSTANT_ROVER_ACTION_HPP_
