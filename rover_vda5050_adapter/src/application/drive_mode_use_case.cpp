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

#include "rover_vda5050_adapter/application/drive_mode_use_case.hpp"

#include <stdexcept>
#include <utility>

#include "rover_vda5050_adapter/domain/drive_mode_request.hpp"

namespace rover_vda5050_adapter::application
{

DriveModeUseCase::DriveModeUseCase(std::shared_ptr<DriveModePort> drive_mode)
: drive_mode_(std::move(drive_mode))
{
    if (!drive_mode_) {
        throw std::invalid_argument("DriveModeUseCase needs a DriveModePort");
    }
}

CommandResult DriveModeUseCase::request(const std::string & mode_value)
{
    const auto request = domain::parseDriveModeRequest(mode_value);

    if (!request.ok()) {
        return {false, request.error, false};
    }

    const std::string name = domain::driveModeName(request.mode);

    std::lock_guard<std::mutex> lock(mutex_);
    const auto result = drive_mode_->set(request.mode);

    if (result.ok) {
        return {true, "Drive mode " + name + ".", false};
    }

    return {false, name + " refused: " + result.message, result.outcome_unknown};
}

}  // namespace rover_vda5050_adapter::application
