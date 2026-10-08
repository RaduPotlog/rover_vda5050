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

#include "pluginlib/class_list_macros.hpp"

#include "rover_vda5050_adapter/plugins/instant_rover_action.hpp"

namespace rover_vda5050_adapter::plugins
{

/**
 * @brief startFollowing / stopFollowing: start or stop follow-me (rover_follow_me's follow_me
 * node). Custom instant actions without parameters.
 *
 * FINISHED as soon as follow_me has started or stopped following; following itself runs outside
 * VDA 5050, on the rover (Nav 2's Following server), and an order sent meanwhile stops it there.
 * FAILED with follow_me's reason when it refuses (not Automatic, a mission running, nobody in
 * front of the rover).
 */
class FollowMeAction : public InstantRoverAction
{
protected:
    explicit FollowMeAction(bool start)
    : start_(start)
    {
    }

    application::CommandResult command() override
    {
        return start_ ? link_->followMe().start() : link_->followMe().stop();
    }

private:
    bool start_;
};

class StartFollowing : public FollowMeAction
{
public:
    StartFollowing()
    : FollowMeAction(true)
    {
    }
};

class StopFollowing : public FollowMeAction
{
public:
    StopFollowing()
    : FollowMeAction(false)
    {
    }
};

}  // namespace rover_vda5050_adapter::plugins

PLUGINLIB_EXPORT_CLASS(rover_vda5050_adapter::plugins::StartFollowing, adapter::VDAAction)
PLUGINLIB_EXPORT_CLASS(rover_vda5050_adapter::plugins::StopFollowing, adapter::VDAAction)
