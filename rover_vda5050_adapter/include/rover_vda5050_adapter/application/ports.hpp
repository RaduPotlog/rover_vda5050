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

#ifndef ROVER_VDA5050_ADAPTER_APPLICATION_PORTS_HPP_
#define ROVER_VDA5050_ADAPTER_APPLICATION_PORTS_HPP_

#include <optional>
#include <string>
#include <vector>

#include "rover_vda5050_adapter/domain/rover_status.hpp"
#include "rover_vda5050_adapter/domain/types.hpp"

namespace rover_vda5050_adapter::application
{

/** @brief Outcome of a command; @c message says why when @c ok is false. */
struct CommandResult
{
    bool ok{true};
    std::string message;
    /// Failed without an answer (timeout): the mission manager may have acted on it anyway.
    bool outcome_unknown{false};
};

/**
 * @brief Outbound port to rover_mission_manager.
 *
 * Implemented by a set_mission / run_mission service client. Both calls block until the manager
 * answers or a timeout expires, so they must never be made from the thread that services the
 * client's responses.
 */
class MissionPort
{
public:
    virtual ~MissionPort() = default;

    /** @brief Replace the active mission with these waypoints and start it. */
    virtual CommandResult dispatch(
        const std::string & mission_id, const std::vector<domain::Pose2D> & waypoints) = 0;

    /** @brief Cancel the active mission, if any. */
    virtual CommandResult cancel() = 0;
};

/** @brief Outbound port reading where the rover is in the navigation frame. */
class PosePort
{
public:
    virtual ~PosePort() = default;

    /** @brief Current pose, or nothing when localization is unavailable. Non-blocking. */
    virtual std::optional<domain::Pose2D> currentPose() const = 0;
};

/**
 * @brief Outbound port to the aux outputs on the safety PLC.
 *
 * Implemented by the hardware interface's aux_output_<i>/set service clients. Blocks until the
 * PLC has acknowledged the write or a timeout expires, so never call it from the thread that
 * services the client's responses.
 */
class AuxOutputPort
{
public:
    virtual ~AuxOutputPort() = default;

    /** @brief Switch one output; @p index is 0-based (0 = DIO00). */
    virtual CommandResult set(int index, bool enabled) = 0;
};

/**
 * @brief Outbound port to rover_drive_mode's drive_mode_manager.
 *
 * Implemented by a set_drive_mode service client. Blocks until the manager answers or a timeout
 * expires, so never call it from the thread that services the client's responses.
 */
class DriveModePort
{
public:
    virtual ~DriveModePort() = default;

    /**
     * @brief Ask for @p mode (kManual or kAutomatic).
     *
     * Not ok when the manager refuses (its reason in @c message, e.g. no mission manager for
     * Automatic) or does not answer.
     */
    virtual CommandResult set(domain::DriveMode mode) = 0;
};

}  // namespace rover_vda5050_adapter::application

#endif  // ROVER_VDA5050_ADAPTER_APPLICATION_PORTS_HPP_
