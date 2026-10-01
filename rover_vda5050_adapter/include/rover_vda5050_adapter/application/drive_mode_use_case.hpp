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

#ifndef ROVER_VDA5050_ADAPTER_APPLICATION_DRIVE_MODE_USE_CASE_HPP_
#define ROVER_VDA5050_ADAPTER_APPLICATION_DRIVE_MODE_USE_CASE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rover_vda5050_adapter/application/ports.hpp"

namespace rover_vda5050_adapter::application
{

/**
 * @brief setDriveMode: fleet control switches the rover to Manual or Automatic.
 *
 * The drive mode manager stays the authority: it may refuse (Automatic without a mission
 * manager), and the new mode reaches master control through the state's operatingMode, as when
 * the drive UI switches it. Blocks on DriveModePort: never call it from the executor thread.
 */
class DriveModeUseCase
{
public:
    explicit DriveModeUseCase(std::shared_ptr<DriveModePort> drive_mode);

    /** @brief Switch to the mode named by @p mode_value (the `mode` action parameter). */
    CommandResult request(const std::string & mode_value);

private:
    std::shared_ptr<DriveModePort> drive_mode_;
    std::mutex mutex_;
};

}  // namespace rover_vda5050_adapter::application

#endif  // ROVER_VDA5050_ADAPTER_APPLICATION_DRIVE_MODE_USE_CASE_HPP_
