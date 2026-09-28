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

#ifndef ROVER_VDA5050_ADAPTER_DOMAIN_ROVER_STATUS_HPP_
#define ROVER_VDA5050_ADAPTER_DOMAIN_ROVER_STATUS_HPP_

#include <optional>
#include <string>
#include <vector>

namespace rover_vda5050_adapter::domain
{

/** @brief rover_drive_mode's mode (rover_msgs/DriveMode); kUnknown until the first message. */
enum class DriveMode
{
    kUnknown,
    kManual,
    kAssisted,
    kAutomatic,
};

/** @brief State of the collision monitor guarding the active drive mode. */
enum class Guard
{
    kUnknown,
    kBypassed,
    kClear,
    kSlowing,
    kStopped,
    kNoData,
};

/** @brief VDA 5050 operatingMode. */
enum class OperatingMode
{
    kAutomatic,
    kSemiautomatic,
    kManual,
    kService,
};

/** @brief The rover_msgs/SafetyStatus fields that matter to master control. */
struct SafetyReading
{
    bool e_stop_pressed{false};   ///< Physical E-Stop button.
    bool latch_active{false};     ///< Safety PLC latch holds a stop until an explicit reset.
    bool link_healthy{true};      ///< false: the two fields above are last-known-good.
};

/** @brief Everything the error and safety reports are derived from. */
struct RoverHealth
{
    DriveMode drive_mode{DriveMode::kUnknown};
    Guard guard{Guard::kUnknown};
    std::optional<SafetyReading> safety;
    std::optional<bool> motion_locked;          ///< Latest fresh motion_lock value, if any.
    bool pose_known{false};
    std::optional<std::string> mission_refusal; ///< Why the mission manager refused the last order.
};

/** @brief VDA 5050 safetyState. */
struct VdaSafety
{
    std::string e_stop;          ///< AUTOACK / MANUAL / REMOTE / NONE
    bool field_violation{false};
};

/** @brief One VDA 5050 error object. */
struct RoverError
{
    std::string type;
    std::string description;
    bool fatal{false};
};

/** @brief sensor_msgs/BatteryState, reduced to what VDA 5050 reports. */
struct BatteryReading
{
    std::optional<double> fraction;   ///< State of charge 0..1; unset when the BMS does not know.
    std::optional<double> voltage;
    bool charging{false};
};

/** @brief VDA 5050 batteryState. */
struct VdaBattery
{
    double charge_percent{0.0};
    double voltage{0.0};
    bool charging{false};
};

/**
 * @brief Operating mode reported to master control.
 *
 * AUTOMATIC is the only mode in which rover_mission_manager accepts missions. ASSISTED and
 * MANUAL are both an operator on the joystick (ASSISTED only keeps the collision monitor in the
 * loop), so master control is not in control: MANUAL. Not SEMIAUTOMATIC, which in VDA 5050 means
 * master control's orders are executed. Without rover_drive_mode nobody can hand the rover over at
 * all, which is SERVICE.
 */
OperatingMode operatingModeFor(DriveMode drive_mode);

/** @brief The exact VDA 5050 enum string. */
std::string toVdaString(OperatingMode mode);

/**
 * @brief VDA 5050 safetyState.
 *
 * A pressed button or an active PLC latch is an e-stop that has to be reset on the rover (MANUAL).
 * The collision monitor's stop zone is the protective field.
 */
VdaSafety safetyFor(const RoverHealth & health);

/**
 * @brief Errors for the VDA 5050 state.
 *
 * All of them are WARNING: the connector stops dispatching navigation while any FATAL error is
 * present, and each condition here either clears by itself or is already enforced by the rover
 * (the mission manager holds on the motion lock and refuses missions outside AUTOMATIC).
 */
std::vector<RoverError> errorsFor(const RoverHealth & health);

/** @brief VDA 5050 batteryState; an unknown charge or voltage is reported as 0. */
VdaBattery batteryFor(const BatteryReading & reading);

}  // namespace rover_vda5050_adapter::domain

#endif  // ROVER_VDA5050_ADAPTER_DOMAIN_ROVER_STATUS_HPP_
