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

#include <string>

#include "pluginlib/class_list_macros.hpp"

#include "rover_vda5050_adapter/plugins/instant_rover_action.hpp"

namespace rover_vda5050_adapter::plugins
{

/**
 * @brief Switch the aux outputs named by the `outputs` parameter (1..6, or a list of them).
 *
 * One plugin instance serves every action of its type, and the connector rejects a second one
 * while the first runs: switch several outputs with one action's list, not with several actions.
 */
class AuxOutputAction : public InstantRoverAction
{
public:
    explicit AuxOutputAction(bool enabled)
    : enabled_(enabled)
    {
    }

protected:
    application::CommandResult command() override
    {
        for (const auto & parameter : action_msg_.action_parameters) {
            if (parameter.key == kOutputsKey) {
                return link_->auxOutputs().apply(parameter.value, enabled_);
            }
        }

        return {false, std::string("Parameter '") + kOutputsKey + "' missing (1..6 or a list).",
            false};
    }

private:
    static constexpr const char * kOutputsKey = "outputs";

    bool enabled_;
};

/** @brief enableAuxOutput: switch aux outputs on. */
class EnableAuxOutput : public AuxOutputAction
{
public:
    EnableAuxOutput()
    : AuxOutputAction(true)
    {
    }
};

/** @brief disableAuxOutput: switch aux outputs off. */
class DisableAuxOutput : public AuxOutputAction
{
public:
    DisableAuxOutput()
    : AuxOutputAction(false)
    {
    }
};

}  // namespace rover_vda5050_adapter::plugins

PLUGINLIB_EXPORT_CLASS(rover_vda5050_adapter::plugins::EnableAuxOutput, adapter::VDAAction)
PLUGINLIB_EXPORT_CLASS(rover_vda5050_adapter::plugins::DisableAuxOutput, adapter::VDAAction)
