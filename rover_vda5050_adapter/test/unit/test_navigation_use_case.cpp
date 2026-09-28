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
#include <string>
#include <vector>

#include "rover_vda5050_adapter/application/navigation_use_case.hpp"

using rover_vda5050_adapter::application::CommandResult;
using rover_vda5050_adapter::application::MissionPort;
using rover_vda5050_adapter::application::NavigationUseCase;
using rover_vda5050_adapter::application::PosePort;
using rover_vda5050_adapter::domain::MissionPhase;
using rover_vda5050_adapter::domain::MissionProgress;
using rover_vda5050_adapter::domain::OrientationMode;
using rover_vda5050_adapter::domain::Pose2D;
using rover_vda5050_adapter::domain::RouteNode;
using rover_vda5050_adapter::domain::RouteOutcome;

namespace
{

class FakeMissions : public MissionPort
{
public:
    CommandResult dispatch(const std::string & id, const std::vector<Pose2D> & waypoints) override
    {
        dispatched_ids.push_back(id);
        last_waypoints = waypoints;
        return next_dispatch;
    }

    CommandResult cancel() override
    {
        ++cancels;
        return {true, "Mission cancelled.", false};
    }

    CommandResult next_dispatch{true, "started", false};
    std::vector<std::string> dispatched_ids;
    std::vector<Pose2D> last_waypoints;
    int cancels{0};
};

class FakePose : public PosePort
{
public:
    std::optional<Pose2D> currentPose() const override { return Pose2D{}; }
};

std::vector<RouteNode> nodes(int count)
{
    std::vector<RouteNode> route;
    for (int i = 0; i < count; ++i) {
        route.push_back(
            RouteNode{"n" + std::to_string(i), static_cast<std::uint32_t>(2 * i),
                Pose2D{i + 1.0, 0.0, 0.0}});
    }
    return route;
}

struct Fixture : ::testing::Test
{
    std::shared_ptr<FakeMissions> missions{std::make_shared<FakeMissions>()};
    NavigationUseCase navigation{
        missions, std::make_shared<FakePose>(), "vda", OrientationMode::kNode};
};

}  // namespace

TEST_F(Fixture, StartSendsTheRouteAndProgressReachesTheGoal)
{
    ASSERT_TRUE(navigation.start(nodes(2)).ok);
    ASSERT_EQ(missions->dispatched_ids.size(), 1U);

    navigation.onMissionProgress({"vda-1", MissionPhase::kRunning, 1, 2, ""});
    navigation.onMissionProgress({"vda-1", MissionPhase::kSucceeded, 2, 2, ""});

    const auto updates = navigation.takeUpdates();
    ASSERT_EQ(updates.size(), 2U);
    EXPECT_EQ(updates[0].last_reached->node_id, "n0");
    EXPECT_EQ(updates[1].outcome, RouteOutcome::kSucceeded);
    EXPECT_TRUE(navigation.takeUpdates().empty());
    EXPECT_FALSE(navigation.routeActive());
}

TEST_F(Fixture, ARefusedRouteIsReportedAndLeavesOtherMissionsAlone)
{
    missions->next_dispatch = {false, "Drive mode is not AUTOMATIC", false};

    const auto result = navigation.start(nodes(1));

    EXPECT_FALSE(result.ok);
    EXPECT_EQ(navigation.lastRefusal().value_or(""), "Drive mode is not AUTOMATIC");
    EXPECT_FALSE(navigation.routeActive());
    // Whatever runs now (an operator's GoTo) is not ours to stop.
    EXPECT_EQ(missions->cancels, 0);

    missions->next_dispatch = {true, "started", false};
    ASSERT_TRUE(navigation.start(nodes(1)).ok);
    EXPECT_FALSE(navigation.lastRefusal());
}

TEST_F(Fixture, ATimedOutDispatchIsCancelledInCaseItStarted)
{
    missions->next_dispatch = {false, "no answer", true};

    EXPECT_FALSE(navigation.start(nodes(1)).ok);
    EXPECT_EQ(missions->cancels, 1);
}

TEST_F(Fixture, ARefusedStitchStopsOurRunningMission)
{
    ASSERT_TRUE(navigation.start(nodes(2)).ok);
    missions->next_dispatch = {false, "Waypoint 0 is in frame 'x'", false};

    EXPECT_FALSE(navigation.extend(nodes(1)).ok);
    EXPECT_EQ(missions->cancels, 1);
    EXPECT_FALSE(navigation.routeActive());
}

TEST_F(Fixture, PauseStopsTheMissionAndResumeSendsTheRest)
{
    ASSERT_TRUE(navigation.start(nodes(3)).ok);
    navigation.onMissionProgress({"vda-1", MissionPhase::kRunning, 1, 3, ""});

    ASSERT_TRUE(navigation.pause().ok);
    EXPECT_EQ(missions->cancels, 1);
    EXPECT_TRUE(navigation.paused());

    navigation.onMissionProgress({"vda-1", MissionPhase::kCancelled, 1, 3, ""});
    ASSERT_TRUE(navigation.resume().ok);

    ASSERT_EQ(missions->dispatched_ids.size(), 2U);
    EXPECT_EQ(missions->dispatched_ids[1], "vda-2");
    EXPECT_EQ(missions->last_waypoints.size(), 2U);
    EXPECT_FALSE(navigation.paused());
    EXPECT_TRUE(navigation.routeActive());
}

TEST_F(Fixture, CancelStopsOurMissionAndIgnoresTheResultingCancelled)
{
    ASSERT_TRUE(navigation.start(nodes(2)).ok);

    ASSERT_TRUE(navigation.cancel().ok);
    EXPECT_EQ(missions->cancels, 1);

    navigation.onMissionProgress({"vda-1", MissionPhase::kCancelled, 0, 2, ""});
    EXPECT_TRUE(navigation.takeUpdates().empty());

    // Nothing of ours runs any more.
    ASSERT_TRUE(navigation.cancel().ok);
    EXPECT_EQ(missions->cancels, 1);
}

TEST_F(Fixture, HeldByTheRoverCountsAsPaused)
{
    ASSERT_TRUE(navigation.start(nodes(1)).ok);

    navigation.onMissionProgress({"vda-1", MissionPhase::kHeld, 0, 1, ""});
    EXPECT_TRUE(navigation.paused());

    navigation.onMissionProgress({"vda-1", MissionPhase::kRunning, 0, 1, ""});
    EXPECT_FALSE(navigation.paused());
}
