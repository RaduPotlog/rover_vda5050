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

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rover_vda5050_adapter/application/follow_me_use_case.hpp"

using rover_vda5050_adapter::application::CommandResult;
using rover_vda5050_adapter::application::FollowMePort;
using rover_vda5050_adapter::application::FollowMeUseCase;

namespace
{

class FakeFollowMe : public FollowMePort
{
public:
    CommandResult start() override
    {
        calls.push_back("start");
        return answer;
    }

    CommandResult stop() override
    {
        calls.push_back("stop");
        return answer;
    }

    std::vector<std::string> calls;
    CommandResult answer{true, "", false};
};

struct Fixture : ::testing::Test
{
    std::shared_ptr<FakeFollowMe> follow_me{std::make_shared<FakeFollowMe>()};
    FollowMeUseCase use_case{follow_me};
};

}  // namespace

TEST_F(Fixture, StartsAndStops)
{
    const auto started = use_case.start();
    const auto stopped = use_case.stop();

    EXPECT_TRUE(started.ok) << started.message;
    EXPECT_EQ(started.message, "Following started.");
    EXPECT_TRUE(stopped.ok) << stopped.message;
    EXPECT_EQ(stopped.message, "Following stopped.");
    EXPECT_EQ(follow_me->calls, (std::vector<std::string>{"start", "stop"}));
}

TEST_F(Fixture, PassesTheRefusalOn)
{
    follow_me->answer = {false, "drive mode is ASSISTED; switch the rover to Automatic", false};

    const auto result = use_case.start();

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.message,
        "startFollowing refused: drive mode is ASSISTED; switch the rover to Automatic");
    EXPECT_FALSE(result.outcome_unknown);
}

TEST_F(Fixture, KeepsAnUnknownOutcome)
{
    follow_me->answer = {false, "'/rover/follow_me/stop' did not answer in time.", true};

    const auto result = use_case.stop();

    EXPECT_FALSE(result.ok);
    EXPECT_TRUE(result.outcome_unknown);
    EXPECT_EQ(result.message.rfind("stopFollowing refused: ", 0), 0u);
}

TEST(FollowMeUseCase, NeedsAPort)
{
    EXPECT_THROW(FollowMeUseCase{nullptr}, std::invalid_argument);
}
