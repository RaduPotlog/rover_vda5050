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

#ifndef ROVER_VDA5050_ADAPTER_DOMAIN_ROUTE_TRACKER_HPP_
#define ROVER_VDA5050_ADAPTER_DOMAIN_ROUTE_TRACKER_HPP_

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

#include "rover_vda5050_adapter/domain/types.hpp"

namespace rover_vda5050_adapter::domain
{

/** @brief A mission to hand to rover_mission_manager. */
struct Dispatch
{
    std::string mission_id;
    std::vector<Pose2D> waypoints;
};

enum class RouteOutcome
{
    kInProgress,
    kSucceeded,
    kFailed,
};

/** @brief What one mission_state sample meant for the route. */
struct RouteUpdate
{
    std::optional<RouteNode> last_reached;   ///< Set when the rover passed at least one more node.
    RouteOutcome outcome{RouteOutcome::kInProgress};
    std::string message;                     ///< Failure reason when outcome is kFailed.
};

/**
 * @brief Maps a VDA 5050 route (a NavigateThroughNodes goal) onto rover_mission_manager missions.
 *
 * The mission manager has no pause/resume and no way to append waypoints: set_mission replaces the
 * active mission and run_mission(false) cancels it. The tracker therefore owns the route and hands
 * out a fresh mission, with its own id, for whatever part of the route is still ahead: on start, on
 * an order stitch (extend) and on resume after a pause. Progress is attributed by mission id, so
 * the latched state of an older mission - including the CANCELLED that a pause itself produces -
 * never moves the route.
 *
 * Pause is a property of the vehicle, not of the route: a route started or extended while paused
 * is recorded but not dispatched until resume().
 */
class RouteTracker
{
public:
    RouteTracker(std::string mission_prefix, OrientationMode orientation_mode);

    /**
     * @brief Replace the route.
     * @param current_pose Where the rover is, for the heading to the first node (kPath).
     * @return The mission to dispatch, or nothing while paused.
     * @throws std::invalid_argument when @p nodes is empty.
     */
    std::optional<Dispatch> start(
        std::vector<RouteNode> nodes, const std::optional<Pose2D> & current_pose);

    /**
     * @brief Append nodes to the active route (VDA 5050 order stitching).
     * @return The mission covering every node not yet reached, or nothing while paused.
     * @throws std::logic_error when no route is active; std::invalid_argument when @p nodes is empty.
     */
    std::optional<Dispatch> extend(
        const std::vector<RouteNode> & nodes, const std::optional<Pose2D> & current_pose);

    /** @brief Enter pause. @return true when a dispatched mission has to be cancelled. */
    bool pause();

    /**
     * @brief Leave pause.
     * @return The mission covering the nodes not yet reached, or nothing when no route is active.
     */
    std::optional<Dispatch> resume(const std::optional<Pose2D> & current_pose);

    /** @brief Forget the route (cancelled, aborted or refused). The pause state is kept. */
    void clear();

    /** @brief Attribute one mission_state sample to the route. */
    RouteUpdate onProgress(const MissionProgress & progress);

    bool active() const { return active_; }
    bool paused() const { return paused_; }
    /// The mission manager suspended the running mission (motion lock or dead lidar).
    bool heldByRover() const { return held_; }
    /// A mission for the current route is with the mission manager.
    bool dispatched() const { return active_ && !paused_ && !mission_id_.empty(); }
    std::size_t reachedCount() const { return reached_; }
    const std::vector<RouteNode> & nodes() const { return nodes_; }

private:
    std::optional<Dispatch> dispatchRemaining(const std::optional<Pose2D> & current_pose);
    std::vector<Pose2D> waypointsFrom(
        std::size_t first, const std::optional<Pose2D> & current_pose) const;

    std::string mission_prefix_;
    OrientationMode orientation_mode_;

    std::vector<RouteNode> nodes_;
    std::size_t reached_{0};          ///< Nodes of nodes_ the rover has passed.
    std::size_t mission_offset_{0};   ///< Index in nodes_ of the dispatched mission's waypoint 0.
    std::string mission_id_;          ///< Dispatched mission; empty when none.
    unsigned long mission_count_{0};

    bool active_{false};
    bool paused_{false};
    bool held_{false};
};

}  // namespace rover_vda5050_adapter::domain

#endif  // ROVER_VDA5050_ADAPTER_DOMAIN_ROUTE_TRACKER_HPP_
