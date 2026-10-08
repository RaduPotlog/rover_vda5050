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

#include "rover_vda5050_adapter/application/follow_me_use_case.hpp"

#include <stdexcept>
#include <utility>

namespace rover_vda5050_adapter::application
{

FollowMeUseCase::FollowMeUseCase(std::shared_ptr<FollowMePort> follow_me)
: follow_me_(std::move(follow_me))
{
    if (!follow_me_) {
        throw std::invalid_argument("FollowMeUseCase needs a FollowMePort");
    }
}

CommandResult FollowMeUseCase::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto result = follow_me_->start();

    if (result.ok) {
        return {true, "Following started.", false};
    }

    return {false, "startFollowing refused: " + result.message, result.outcome_unknown};
}

CommandResult FollowMeUseCase::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto result = follow_me_->stop();

    if (result.ok) {
        return {true, "Following stopped.", false};
    }

    return {false, "stopFollowing refused: " + result.message, result.outcome_unknown};
}

}  // namespace rover_vda5050_adapter::application
