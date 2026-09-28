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

#include "rover_vda5050_adapter/domain/rover_status.hpp"

#include <algorithm>

namespace rover_vda5050_adapter::domain
{

OperatingMode operatingModeFor(DriveMode drive_mode)
{
    switch (drive_mode) {
        case DriveMode::kAutomatic:
            return OperatingMode::kAutomatic;
        case DriveMode::kAssisted:
        case DriveMode::kManual:
            return OperatingMode::kManual;
        case DriveMode::kUnknown:
            break;
    }

    return OperatingMode::kService;
}

std::string toVdaString(OperatingMode mode)
{
    switch (mode) {
        case OperatingMode::kAutomatic:
            return "AUTOMATIC";
        case OperatingMode::kSemiautomatic:
            return "SEMIAUTOMATIC";
        case OperatingMode::kManual:
            return "MANUAL";
        case OperatingMode::kService:
            break;
    }

    return "SERVICE";
}

VdaSafety safetyFor(const RoverHealth & health)
{
    VdaSafety safety;
    safety.e_stop = "NONE";

    if (health.safety && (health.safety->e_stop_pressed || health.safety->latch_active)) {
        safety.e_stop = "MANUAL";
    }

    safety.field_violation = health.guard == Guard::kStopped;

    return safety;
}

std::vector<RoverError> errorsFor(const RoverHealth & health)
{
    std::vector<RoverError> errors;

    if (health.safety && !health.safety->link_healthy) {
        errors.push_back(
            {"safetyLinkDown",
             "The link to the safety PLC is down; the reported e-stop state is last-known-good.",
             false});
    }

    if (health.motion_locked.value_or(false)) {
        errors.push_back(
            {"motionLocked",
             "Motion is locked (e-stop, stale safety data or a lock request); the rover holds "
             "position until it clears.",
             false});
    }

    if (!health.pose_known) {
        errors.push_back(
            {"localizationUnavailable",
             "No pose in the navigation frame; the position is not initialized.", false});
    }

    if (health.mission_refusal) {
        errors.push_back({"missionRefused", *health.mission_refusal, false});
    }

    return errors;
}

VdaBattery batteryFor(const BatteryReading & reading)
{
    VdaBattery battery;
    battery.charge_percent = std::clamp(reading.fraction.value_or(0.0), 0.0, 1.0) * 100.0;
    battery.voltage = reading.voltage.value_or(0.0);
    battery.charging = reading.charging;

    return battery;
}

}  // namespace rover_vda5050_adapter::domain
