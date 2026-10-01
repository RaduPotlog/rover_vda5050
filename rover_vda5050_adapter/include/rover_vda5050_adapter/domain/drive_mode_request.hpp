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

#ifndef ROVER_VDA5050_ADAPTER_DOMAIN_DRIVE_MODE_REQUEST_HPP_
#define ROVER_VDA5050_ADAPTER_DOMAIN_DRIVE_MODE_REQUEST_HPP_

#include <string>

#include "rover_vda5050_adapter/domain/rover_status.hpp"

namespace rover_vda5050_adapter::domain
{

/** @brief The drive mode a setDriveMode action asks for, or why its parameter was refused. */
struct DriveModeRequest
{
    /// kManual or kAutomatic when accepted; kUnknown when @c error is set.
    DriveMode mode{DriveMode::kUnknown};
    std::string error;

    bool ok() const { return error.empty(); }
};

/**
 * @brief Parse the `mode` parameter of setDriveMode: `MANUAL` or `AUTOMATIC`.
 *
 * Case-insensitive; surrounding blanks and quotes are ignored (the connector hands the value
 * over as Python's str() of the JSON value). Fleet control may stop automation (MANUAL) or hand
 * the rover to it (AUTOMATIC); ASSISTED stays on the rover's drive UI, where someone watches the
 * rover. Never throws.
 */
DriveModeRequest parseDriveModeRequest(const std::string & value);

/** @brief "Manual", "Assisted", "Automatic" or "unknown", as the drive UI names the modes. */
std::string driveModeName(DriveMode mode);

}  // namespace rover_vda5050_adapter::domain

#endif  // ROVER_VDA5050_ADAPTER_DOMAIN_DRIVE_MODE_REQUEST_HPP_
