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

#ifndef ROVER_VDA5050_ADAPTER_APPLICATION_NAVIGATION_USE_CASE_HPP_
#define ROVER_VDA5050_ADAPTER_APPLICATION_NAVIGATION_USE_CASE_HPP_

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "rover_vda5050_adapter/application/ports.hpp"
#include "rover_vda5050_adapter/domain/route_tracker.hpp"
#include "rover_vda5050_adapter/domain/types.hpp"

namespace rover_vda5050_adapter::application
{

/**
 * @brief Drives a VDA 5050 route through rover_mission_manager: start, stitch, pause, resume,
 *        cancel, and progress.
 *
 * Shared by the navigation handler and the pause actions, which run on different threads.
 *
 * Threading contract: commands (start ... cancel) call MissionPort, which blocks until the
 * mission manager answers, so they must run off the executor thread. onMissionProgress() and the
 * queries run on the executor thread and never block on a command: commands are serialized by
 * their own mutex, and the tracker's mutex is never held across a MissionPort call. Without that
 * split, a mission_state message delivered while a set_mission call waits would block the only
 * thread that can deliver the call's response.
 */
class NavigationUseCase
{
public:
    NavigationUseCase(
        std::shared_ptr<MissionPort> missions, std::shared_ptr<PosePort> pose,
        std::string mission_prefix, domain::OrientationMode orientation_mode);

    /** @brief Replace the route and send it to the rover (held back while paused). */
    CommandResult start(std::vector<domain::RouteNode> nodes);

    /** @brief Append nodes to the active route (order stitching). */
    CommandResult extend(std::vector<domain::RouteNode> nodes);

    /** @brief Stop and forget the route. */
    CommandResult cancel();

    /** @brief VDA 5050 startPause: stop driving, keep the route. Valid without a route. */
    CommandResult pause();

    /** @brief VDA 5050 stopPause: continue the route from the first node not yet reached. */
    CommandResult resume();

    /** @brief Feed one mission_state sample. Executor thread; never blocks on a command. */
    void onMissionProgress(const domain::MissionProgress & progress);

    /** @brief Route updates since the last call, oldest first. */
    std::vector<domain::RouteUpdate> takeUpdates();

    bool routeActive() const;
    /// Paused by startPause, or held by the rover's own motion lock.
    bool paused() const;
    /// Why the mission manager refused the last mission; cleared by cancel() or the next accepted one.
    std::optional<std::string> lastRefusal() const;

private:
    /// @param ours_was_running A mission of this route was with the manager before this one.
    CommandResult send(const std::optional<domain::Dispatch> & dispatch, bool ours_was_running);

    std::shared_ptr<MissionPort> missions_;
    std::shared_ptr<PosePort> pose_;

    std::mutex command_mutex_;       ///< Serializes commands, held across MissionPort calls.
    mutable std::mutex state_mutex_; ///< Guards everything below; never held across a port call.
    domain::RouteTracker tracker_;
    std::vector<domain::RouteUpdate> updates_;
    std::optional<std::string> last_refusal_;
};

}  // namespace rover_vda5050_adapter::application

#endif  // ROVER_VDA5050_ADAPTER_APPLICATION_NAVIGATION_USE_CASE_HPP_
