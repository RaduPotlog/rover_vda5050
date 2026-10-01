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
 * @brief setDriveMode: switch the rover to the drive mode named by `mode` (MANUAL, AUTOMATIC).
 *
 * A custom instant action: VDA 5050 has no master-control command for operatingMode. It is
 * served in every operating mode, MANUAL included, against the letter of the VDA 5050
 * operating-mode table, because handing a Manual rover back to fleet control is its purpose.
 */
class SetDriveMode : public InstantRoverAction
{
protected:
    application::CommandResult command() override
    {
        for (const auto & parameter : action_msg_.action_parameters) {
            if (parameter.key == kModeKey) {
                return link_->driveMode().request(parameter.value);
            }
        }

        return {false, std::string("Parameter '") + kModeKey + "' missing (MANUAL or AUTOMATIC).",
            false};
    }

private:
    static constexpr const char * kModeKey = "mode";
};

}  // namespace rover_vda5050_adapter::plugins

PLUGINLIB_EXPORT_CLASS(rover_vda5050_adapter::plugins::SetDriveMode, adapter::VDAAction)
