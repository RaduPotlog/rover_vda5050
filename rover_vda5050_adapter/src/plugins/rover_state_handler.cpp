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

#include <memory>

#include "pluginlib/class_list_macros.hpp"
#include "vda5050_connector/state_handler.hpp"

#include "rover_vda5050_adapter/domain/rover_status.hpp"
#include "rover_vda5050_adapter/infrastructure/rover_link.hpp"
#include "rover_vda5050_adapter/infrastructure/vda_conversions.hpp"

namespace rover_vda5050_adapter::plugins
{

/**
 * @brief Fills the rover-specific part of the VDA 5050 state: position, velocity, battery,
 *        operating mode, pause, safety and errors.
 *
 * Runs on the executor thread for every GetState request, so it only reads cached data. `driving`
 * is left to the navigation handler, which owns it in the connector's design.
 */
class RoverStateHandler : public adapter::StateHandler
{
public:
    void configure() override { link_ = infrastructure::RoverLink::forNode(node_); }

    void execute() override
    {
        using OrderState = vda5050_msgs::msg::OrderState;

        const auto snapshot = link_->snapshot();

        current_state_->set_parameter(
            &OrderState::agv_position, infrastructure::toAgvPosition(snapshot));
        current_state_->set_parameter(&OrderState::velocity, infrastructure::toVelocity(snapshot));
        current_state_->set_parameter(
            &OrderState::battery_state,
            infrastructure::toBatteryState(domain::batteryFor(snapshot.battery)));
        current_state_->set_parameter(
            &OrderState::operating_mode,
            domain::toVdaString(domain::operatingModeFor(snapshot.health.drive_mode)));
        current_state_->set_parameter(&OrderState::paused, snapshot.paused);
        current_state_->set_parameter(
            &OrderState::safety_state,
            infrastructure::toSafetyState(domain::safetyFor(snapshot.health)));

        for (const auto & error : domain::errorsFor(snapshot.health)) {
            current_state_->add_error(infrastructure::toError(error));
        }
    }

private:
    std::shared_ptr<infrastructure::RoverLink> link_;
};

}  // namespace rover_vda5050_adapter::plugins

PLUGINLIB_EXPORT_CLASS(rover_vda5050_adapter::plugins::RoverStateHandler, adapter::StateHandler)
