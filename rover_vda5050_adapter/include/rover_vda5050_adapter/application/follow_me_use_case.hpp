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

#ifndef ROVER_VDA5050_ADAPTER_APPLICATION_FOLLOW_ME_USE_CASE_HPP_
#define ROVER_VDA5050_ADAPTER_APPLICATION_FOLLOW_ME_USE_CASE_HPP_

#include <memory>
#include <mutex>

#include "rover_vda5050_adapter/application/ports.hpp"

namespace rover_vda5050_adapter::application
{

/**
 * @brief startFollowing / stopFollowing: fleet control starts or stops follow-me.
 *
 * The rover's follow_me node stays the authority: it refuses a start outside Automatic, during a
 * mission or with nobody in front of the rover, and its reason becomes the action's result.
 * Following then runs outside VDA 5050 (Nav 2's Following server), so the action finishes as soon
 * as following has started or stopped; a later order stops following on the rover side. Blocks
 * on FollowMePort: never call it from the executor thread.
 */
class FollowMeUseCase
{
public:
    explicit FollowMeUseCase(std::shared_ptr<FollowMePort> follow_me);

    CommandResult start();
    CommandResult stop();

private:
    std::shared_ptr<FollowMePort> follow_me_;
    std::mutex mutex_;
};

}  // namespace rover_vda5050_adapter::application

#endif  // ROVER_VDA5050_ADAPTER_APPLICATION_FOLLOW_ME_USE_CASE_HPP_
