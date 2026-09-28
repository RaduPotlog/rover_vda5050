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

#include "rover_vda5050_adapter/domain/route_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace rover_vda5050_adapter::domain
{

namespace
{

// Closer than this, two points give no usable heading.
constexpr double kMinHeadingDistance = 1e-3;

std::optional<double> headingBetween(const Pose2D & from, const Pose2D & to)
{
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;

    if (std::hypot(dx, dy) < kMinHeadingDistance) {
        return std::nullopt;
    }

    return std::atan2(dy, dx);
}

}  // namespace

RouteTracker::RouteTracker(std::string mission_prefix, OrientationMode orientation_mode)
: mission_prefix_(std::move(mission_prefix)), orientation_mode_(orientation_mode)
{
}

std::optional<Dispatch> RouteTracker::start(
    std::vector<RouteNode> nodes, const std::optional<Pose2D> & current_pose)
{
    if (nodes.empty()) {
        throw std::invalid_argument("A route needs at least one node.");
    }

    nodes_ = std::move(nodes);
    reached_ = 0;
    mission_offset_ = 0;
    mission_id_.clear();
    held_ = false;
    active_ = true;

    return dispatchRemaining(current_pose);
}

std::optional<Dispatch> RouteTracker::extend(
    const std::vector<RouteNode> & nodes, const std::optional<Pose2D> & current_pose)
{
    if (!active_) {
        throw std::logic_error("No active route to extend.");
    }

    if (nodes.empty()) {
        throw std::invalid_argument("An extension needs at least one node.");
    }

    nodes_.insert(nodes_.end(), nodes.begin(), nodes.end());

    return dispatchRemaining(current_pose);
}

bool RouteTracker::pause()
{
    const bool had_mission = dispatched();

    paused_ = true;
    held_ = false;
    // The cancelled mission's progress no longer counts; resume() dispatches a new one.
    mission_id_.clear();

    return had_mission;
}

std::optional<Dispatch> RouteTracker::resume(const std::optional<Pose2D> & current_pose)
{
    paused_ = false;

    if (!active_) {
        return std::nullopt;
    }

    return dispatchRemaining(current_pose);
}

void RouteTracker::clear()
{
    nodes_.clear();
    reached_ = 0;
    mission_offset_ = 0;
    mission_id_.clear();
    held_ = false;
    active_ = false;
}

RouteUpdate RouteTracker::onProgress(const MissionProgress & progress)
{
    RouteUpdate update;

    if (!dispatched() || progress.mission_id != mission_id_) {
        return update;
    }

    held_ = progress.phase == MissionPhase::kHeld;

    const std::size_t index_in_mission = std::min(progress.current_index, progress.total);
    const std::size_t reached = std::min(mission_offset_ + index_in_mission, nodes_.size());

    if (reached > reached_) {
        reached_ = reached;
        update.last_reached = nodes_[reached_ - 1];
    }

    switch (progress.phase) {
        case MissionPhase::kSucceeded:
            if (reached_ >= nodes_.size()) {
                update.outcome = RouteOutcome::kSucceeded;
                clear();
            }
            break;

        case MissionPhase::kFailed:
            update.outcome = RouteOutcome::kFailed;
            update.message = progress.message.empty() ?
                std::string("Mission failed on the rover.") :
                "Mission failed on the rover: " + progress.message;
            clear();
            break;

        case MissionPhase::kCancelled:
            // Pause and cancel both forget the mission id before cancelling, so a CANCELLED that
            // still matches came from the rover itself, e.g. the drive mode left AUTOMATIC.
            update.outcome = RouteOutcome::kFailed;
            update.message = progress.message.empty() ?
                std::string("Mission cancelled on the rover.") :
                "Mission cancelled on the rover: " + progress.message;
            clear();
            break;

        case MissionPhase::kIdle:
        case MissionPhase::kRunning:
        case MissionPhase::kHeld:
            break;
    }

    return update;
}

std::optional<Dispatch> RouteTracker::dispatchRemaining(const std::optional<Pose2D> & current_pose)
{
    mission_id_.clear();

    if (paused_ || reached_ >= nodes_.size()) {
        return std::nullopt;
    }

    mission_offset_ = reached_;
    mission_id_ = mission_prefix_ + "-" + std::to_string(++mission_count_);
    held_ = false;

    return Dispatch{mission_id_, waypointsFrom(mission_offset_, current_pose)};
}

std::vector<Pose2D> RouteTracker::waypointsFrom(
    std::size_t first, const std::optional<Pose2D> & current_pose) const
{
    std::vector<Pose2D> waypoints;
    waypoints.reserve(nodes_.size() - first);

    for (std::size_t i = first; i < nodes_.size(); ++i) {
        Pose2D waypoint = nodes_[i].pose;

        if (orientation_mode_ == OrientationMode::kPath) {
            // Arrive facing along the edge the rover drives in on: from the previous node, or
            // from where the rover stands for the first one.
            std::optional<double> heading;

            if (i > 0) {
                heading = headingBetween(nodes_[i - 1].pose, nodes_[i].pose);
            } else if (current_pose) {
                heading = headingBetween(*current_pose, nodes_[i].pose);
            }

            // A zero-length edge (repeated node) keeps the heading the rover already has.
            if (!heading && !waypoints.empty()) {
                heading = waypoints.back().theta;
            }

            if (heading) {
                waypoint.theta = *heading;
            }
        }

        waypoints.push_back(waypoint);
    }

    return waypoints;
}

}  // namespace rover_vda5050_adapter::domain
