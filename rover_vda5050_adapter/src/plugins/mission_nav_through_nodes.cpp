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

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "vda5050_connector/nav_through_nodes.hpp"

#include "rover_vda5050_adapter/infrastructure/rover_link.hpp"
#include "rover_vda5050_adapter/infrastructure/vda_conversions.hpp"

namespace rover_vda5050_adapter::plugins
{

using domain::RouteOutcome;

/**
 * @brief NavigateThroughNodes handler that drives the route through rover_mission_manager.
 *
 * The connector calls execute() on a thread of its own per goal, and cancel() and the
 * ExtendNavigation hook on the executor thread. Mission-manager calls block, so the executor-thread
 * entry points only queue a request and wake the goal thread, which does all the work.
 */
class MissionNavThroughNodes : public adapter::NavThroughNodes
{
public:
    ~MissionNavThroughNodes() override
    {
        if (link_) {
            link_->clearProgressListener();
        }
    }

    void configure() override
    {
        link_ = infrastructure::RoverLink::forNode(node_);
        link_->setProgressListener([this]() { wake(); });
        setupExtendNavigationService();
    }

    void execute() override
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cancel_requested_ = false;
            pending_extension_.clear();
            woken_ = false;
        }

        auto & navigation = link_->navigation();
        const auto [edges, nodes] = getNavigationSnapshot();

        const auto started = navigation.start(infrastructure::toRouteNodes(nodes));
        if (!started.ok) {
            RCLCPP_WARN(node_->get_logger(), "VDA 5050 route refused: %s", started.message.c_str());
            finish(false);
            return;
        }

        RCLCPP_INFO(
            node_->get_logger(), "VDA 5050 route of %zu node(s) started: %s", nodes.size(),
            started.message.c_str());

        while (rclcpp::ok()) {
            bool cancel_requested = false;
            std::vector<vda5050_msgs::msg::Node> extension;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wakeup_.wait_for(lock, kPollPeriod, [this]() { return woken_; });
                woken_ = false;
                cancel_requested = cancel_requested_;
                extension.swap(pending_extension_);
            }

            if (cancel_requested) {
                const auto result = navigation.cancel();
                RCLCPP_INFO(
                    node_->get_logger(), "VDA 5050 route cancelled: %s", result.message.c_str());
                update_driving_state(false);
                goal_handle_->canceled(result_);
                return;
            }

            if (!extension.empty()) {
                const auto result = navigation.extend(infrastructure::toRouteNodes(extension));
                if (!result.ok) {
                    RCLCPP_WARN(
                        node_->get_logger(), "VDA 5050 route extension refused: %s",
                        result.message.c_str());
                    finish(false);
                    return;
                }
            }

            for (const auto & update : navigation.takeUpdates()) {
                if (update.last_reached) {
                    publishReached(*update.last_reached);
                }

                if (update.outcome == RouteOutcome::kSucceeded) {
                    finish(true);
                    return;
                }

                if (update.outcome == RouteOutcome::kFailed) {
                    RCLCPP_WARN(node_->get_logger(), "VDA 5050 route failed: %s",
                        update.message.c_str());
                    finish(false);
                    return;
                }
            }

            update_driving_state(navigation.routeActive() && !navigation.paused());
        }
    }

    bool cancel() override
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            cancel_requested_ = true;
            woken_ = true;
        }
        wakeup_.notify_all();

        return true;
    }

protected:
    void onNavigationExtended(size_t old_edge_count) override
    {
        // Re-snapshot: the base class has appended the stitched nodes. edges[i] leads to nodes[i],
        // so the new nodes start where the old edges ended.
        const auto [edges, nodes] = getNavigationSnapshot();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            pending_extension_.insert(
                pending_extension_.end(), nodes.begin() + static_cast<std::ptrdiff_t>(old_edge_count),
                nodes.end());
            woken_ = true;
        }
        wakeup_.notify_all();
    }

private:
    static constexpr std::chrono::milliseconds kPollPeriod{200};

    void wake()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            woken_ = true;
        }
        wakeup_.notify_all();
    }

    void publishReached(const domain::RouteNode & reached)
    {
        const auto [edges, nodes] = getNavigationSnapshot();

        for (const auto & node : nodes) {
            if (node.sequence_id == reached.sequence_id) {
                feedback_->last_node = node;
                break;
            }
        }

        const auto snapshot = link_->snapshot();
        feedback_->position = infrastructure::toAgvPosition(snapshot);
        feedback_->velocity = infrastructure::toVelocity(snapshot);
        goal_handle_->publish_feedback(feedback_);
    }

    void finish(bool succeeded)
    {
        update_driving_state(false);

        if (succeeded) {
            goal_handle_->succeed(result_);
        } else {
            goal_handle_->abort(result_);
        }
    }

    std::shared_ptr<infrastructure::RoverLink> link_;

    std::mutex mutex_;
    std::condition_variable wakeup_;
    bool woken_{false};
    bool cancel_requested_{false};
    std::vector<vda5050_msgs::msg::Node> pending_extension_;
};

}  // namespace rover_vda5050_adapter::plugins

PLUGINLIB_EXPORT_CLASS(rover_vda5050_adapter::plugins::MissionNavThroughNodes, adapter::NavThroughNodes)
