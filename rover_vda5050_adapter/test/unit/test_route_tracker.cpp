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

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "rover_vda5050_adapter/domain/route_tracker.hpp"

using rover_vda5050_adapter::domain::MissionPhase;
using rover_vda5050_adapter::domain::MissionProgress;
using rover_vda5050_adapter::domain::OrientationMode;
using rover_vda5050_adapter::domain::Pose2D;
using rover_vda5050_adapter::domain::RouteNode;
using rover_vda5050_adapter::domain::RouteOutcome;
using rover_vda5050_adapter::domain::RouteTracker;

namespace
{

// Nodes along +x, 1 m apart, each with theta 1.0 so kNode and kPath are distinguishable.
std::vector<RouteNode> line(std::uint32_t first_sequence, int count, double start_x = 1.0)
{
    std::vector<RouteNode> nodes;
    for (int i = 0; i < count; ++i) {
        nodes.push_back(
            RouteNode{
                "n" + std::to_string(first_sequence + 2 * i), first_sequence + 2 * i,
                Pose2D{start_x + i, 0.0, 1.0}});
    }
    return nodes;
}

MissionProgress progress(
    const std::string & id, MissionPhase phase, std::uint32_t index, std::uint32_t total)
{
    return MissionProgress{id, phase, index, total, ""};
}

}  // namespace

TEST(RouteTracker, StartDispatchesEveryNodeWithAFreshMissionId)
{
    RouteTracker tracker("vda", OrientationMode::kNode);

    const auto dispatch = tracker.start(line(2, 3), Pose2D{0.0, 0.0, 0.0});

    ASSERT_TRUE(dispatch);
    EXPECT_EQ(dispatch->mission_id, "vda-1");
    ASSERT_EQ(dispatch->waypoints.size(), 3U);
    EXPECT_DOUBLE_EQ(dispatch->waypoints[2].x, 3.0);
    EXPECT_DOUBLE_EQ(dispatch->waypoints[0].theta, 1.0);
    EXPECT_TRUE(tracker.active());
    EXPECT_TRUE(tracker.dispatched());
}

TEST(RouteTracker, StartRejectsAnEmptyRoute)
{
    RouteTracker tracker("vda", OrientationMode::kNode);

    EXPECT_THROW(tracker.start({}, std::nullopt), std::invalid_argument);
}

TEST(RouteTracker, PathModeFacesAlongTheArrivalEdge)
{
    RouteTracker tracker("vda", OrientationMode::kPath);
    std::vector<RouteNode> nodes{
        {"a", 2, {1.0, 0.0, 1.0}}, {"b", 4, {1.0, 1.0, 1.0}}, {"c", 6, {1.0, 1.0, 1.0}}};

    const auto dispatch = tracker.start(nodes, Pose2D{0.0, 0.0, 0.0});

    ASSERT_TRUE(dispatch);
    EXPECT_NEAR(dispatch->waypoints[0].theta, 0.0, 1e-9);           // from the rover, along +x
    EXPECT_NEAR(dispatch->waypoints[1].theta, M_PI / 2.0, 1e-9);    // along +y
    EXPECT_NEAR(dispatch->waypoints[2].theta, M_PI / 2.0, 1e-9);    // zero-length edge keeps it
}

TEST(RouteTracker, PathModeWithoutAPoseUsesTheFirstNodesTheta)
{
    RouteTracker tracker("vda", OrientationMode::kPath);

    const auto dispatch = tracker.start(line(0, 1), std::nullopt);

    ASSERT_TRUE(dispatch);
    EXPECT_DOUBLE_EQ(dispatch->waypoints[0].theta, 1.0);
}

TEST(RouteTracker, ProgressReportsEachNewlyReachedNode)
{
    RouteTracker tracker("vda", OrientationMode::kNode);
    tracker.start(line(2, 3), std::nullopt);

    auto update = tracker.onProgress(progress("vda-1", MissionPhase::kRunning, 0, 3));
    EXPECT_FALSE(update.last_reached);

    update = tracker.onProgress(progress("vda-1", MissionPhase::kRunning, 2, 3));
    ASSERT_TRUE(update.last_reached);
    EXPECT_EQ(update.last_reached->sequence_id, 4U);    // skipped ahead: second node reached
    EXPECT_EQ(update.outcome, RouteOutcome::kInProgress);

    // A repeated sample reports nothing new.
    update = tracker.onProgress(progress("vda-1", MissionPhase::kRunning, 2, 3));
    EXPECT_FALSE(update.last_reached);

    update = tracker.onProgress(progress("vda-1", MissionPhase::kSucceeded, 3, 3));
    ASSERT_TRUE(update.last_reached);
    EXPECT_EQ(update.last_reached->sequence_id, 6U);
    EXPECT_EQ(update.outcome, RouteOutcome::kSucceeded);
    EXPECT_FALSE(tracker.active());
}

TEST(RouteTracker, ProgressOfAnotherMissionIsIgnored)
{
    RouteTracker tracker("vda", OrientationMode::kNode);
    tracker.start(line(2, 2), std::nullopt);

    // e.g. the latched state of an operator's GoTo, or of the mission a stitch replaced.
    const auto update = tracker.onProgress(progress("goto", MissionPhase::kFailed, 0, 1));

    EXPECT_EQ(update.outcome, RouteOutcome::kInProgress);
    EXPECT_TRUE(tracker.active());
}

TEST(RouteTracker, FailureAndRoverCancellationFailTheRoute)
{
    RouteTracker failed("vda", OrientationMode::kNode);
    failed.start(line(2, 2), std::nullopt);
    auto update = failed.onProgress(
        MissionProgress{"vda-1", MissionPhase::kFailed, 1, 2, "navigation failed"});
    EXPECT_EQ(update.outcome, RouteOutcome::kFailed);
    EXPECT_NE(update.message.find("navigation failed"), std::string::npos);
    EXPECT_FALSE(failed.active());

    RouteTracker cancelled("vda", OrientationMode::kNode);
    cancelled.start(line(2, 2), std::nullopt);
    update = cancelled.onProgress(
        MissionProgress{"vda-1", MissionPhase::kCancelled, 0, 2, "drive mode left AUTOMATIC"});
    EXPECT_EQ(update.outcome, RouteOutcome::kFailed);
    EXPECT_NE(update.message.find("AUTOMATIC"), std::string::npos);
}

TEST(RouteTracker, HeldIsReportedAndClearsOnRunning)
{
    RouteTracker tracker("vda", OrientationMode::kNode);
    tracker.start(line(2, 2), std::nullopt);

    tracker.onProgress(progress("vda-1", MissionPhase::kHeld, 0, 2));
    EXPECT_TRUE(tracker.heldByRover());

    tracker.onProgress(progress("vda-1", MissionPhase::kRunning, 0, 2));
    EXPECT_FALSE(tracker.heldByRover());
}

TEST(RouteTracker, ExtendRedispatchesOnlyTheNodesAhead)
{
    RouteTracker tracker("vda", OrientationMode::kNode);
    tracker.start(line(2, 2), std::nullopt);
    tracker.onProgress(progress("vda-1", MissionPhase::kRunning, 1, 2));   // n2 reached

    const auto dispatch = tracker.extend(line(6, 2, 3.0), std::nullopt);

    ASSERT_TRUE(dispatch);
    EXPECT_EQ(dispatch->mission_id, "vda-2");
    ASSERT_EQ(dispatch->waypoints.size(), 3U);   // n4, n6, n8
    EXPECT_DOUBLE_EQ(dispatch->waypoints[0].x, 2.0);

    // The replaced mission's progress no longer counts; the new one's index is offset.
    auto update = tracker.onProgress(progress("vda-1", MissionPhase::kSucceeded, 2, 2));
    EXPECT_FALSE(update.last_reached);

    update = tracker.onProgress(progress("vda-2", MissionPhase::kRunning, 1, 3));
    ASSERT_TRUE(update.last_reached);
    EXPECT_EQ(update.last_reached->sequence_id, 4U);
}

TEST(RouteTracker, ExtendWithoutARouteThrows)
{
    RouteTracker tracker("vda", OrientationMode::kNode);

    EXPECT_THROW(tracker.extend(line(2, 1), std::nullopt), std::logic_error);
}

TEST(RouteTracker, PauseDropsTheMissionAndResumeContinuesFromTheNextNode)
{
    RouteTracker tracker("vda", OrientationMode::kNode);
    tracker.start(line(2, 3), std::nullopt);
    tracker.onProgress(progress("vda-1", MissionPhase::kRunning, 1, 3));

    EXPECT_TRUE(tracker.pause());
    EXPECT_TRUE(tracker.paused());
    EXPECT_FALSE(tracker.dispatched());

    // The CANCELLED the pause itself causes must not fail the route.
    auto update = tracker.onProgress(progress("vda-1", MissionPhase::kCancelled, 1, 3));
    EXPECT_EQ(update.outcome, RouteOutcome::kInProgress);
    EXPECT_TRUE(tracker.active());

    const auto dispatch = tracker.resume(std::nullopt);
    ASSERT_TRUE(dispatch);
    EXPECT_EQ(dispatch->mission_id, "vda-2");
    ASSERT_EQ(dispatch->waypoints.size(), 2U);
    EXPECT_DOUBLE_EQ(dispatch->waypoints[0].x, 2.0);
    EXPECT_FALSE(tracker.paused());
}

TEST(RouteTracker, RoutesStartedWhilePausedWaitForResume)
{
    RouteTracker tracker("vda", OrientationMode::kNode);

    EXPECT_FALSE(tracker.pause());   // nothing to stop
    EXPECT_FALSE(tracker.start(line(2, 2), std::nullopt));
    EXPECT_TRUE(tracker.active());
    EXPECT_FALSE(tracker.dispatched());

    const auto dispatch = tracker.resume(std::nullopt);
    ASSERT_TRUE(dispatch);
    EXPECT_EQ(dispatch->waypoints.size(), 2U);
}

TEST(RouteTracker, ResumeWithoutARouteDispatchesNothing)
{
    RouteTracker tracker("vda", OrientationMode::kNode);
    tracker.pause();

    EXPECT_FALSE(tracker.resume(std::nullopt));
    EXPECT_FALSE(tracker.paused());
}
