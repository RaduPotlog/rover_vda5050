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

#ifndef ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_ROVER_LINK_HPP_
#define ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_ROVER_LINK_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/drive_mode.hpp"
#include "rover_msgs/msg/localization_state.hpp"
#include "rover_msgs/msg/mission_state.hpp"
#include "rover_msgs/msg/safety_status.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

#include "rover_vda5050_adapter/application/aux_output_use_case.hpp"
#include "rover_vda5050_adapter/application/drive_mode_use_case.hpp"
#include "rover_vda5050_adapter/application/follow_me_use_case.hpp"
#include "rover_vda5050_adapter/application/navigation_use_case.hpp"
#include "rover_vda5050_adapter/application/ports.hpp"
#include "rover_vda5050_adapter/domain/rover_status.hpp"
#include "rover_vda5050_adapter/domain/types.hpp"

namespace rover_vda5050_adapter::infrastructure
{

/** @brief The adapter's `rover.*` parameters, with names resolved against the rover namespace. */
struct RoverLinkConfig
{
    std::string rover_namespace;
    std::string map_frame;
    std::string base_frame;
    std::string map_id;
    std::string mission_prefix;
    domain::OrientationMode orientation_mode{domain::OrientationMode::kPath};

    std::string set_mission_service;
    std::string run_mission_service;
    std::string mission_state_topic;
    std::string drive_mode_topic;
    std::string drive_mode_service;
    std::string follow_me_start_service;
    std::string follow_me_stop_service;
    std::string battery_topic;
    std::string safety_status_topic;
    std::string motion_lock_topic;
    std::string localization_state_topic;
    std::string odom_topic;
    /// Resolved service name up to the output index: <prefix><0..5>/set.
    std::string aux_output_service_prefix;

    double service_availability_timeout{2.0};
    double service_response_timeout{3.0};
    double pose_timeout{2.0};
    double status_timeout{2.0};
    double aux_output_timeout{3.0};

    /** @brief Declare and read the parameters on @p node. */
    static RoverLinkConfig fromParameters(rclcpp::Node & node);
};

/** @brief Robot-frame velocity. */
struct Velocity2D
{
    double vx{0.0};
    double vy{0.0};
    double omega{0.0};
};

/** @brief Everything the VDA 5050 state reports about the rover, at one instant. */
struct RoverSnapshot
{
    std::optional<domain::Pose2D> pose;
    std::string map_id;
    Velocity2D velocity;
    domain::BatteryReading battery;
    domain::RoverHealth health;
    bool paused{false};
};

/** @brief PosePort over TF: navigation frame -> base frame. */
class TfPoseSource : public application::PosePort
{
public:
    TfPoseSource(
        rclcpp::Node & node, std::string map_frame, std::string base_frame, double max_age);

    std::optional<domain::Pose2D> currentPose() const override;

private:
    rclcpp::Node & node_;
    std::string map_frame_;
    std::string base_frame_;
    rclcpp::Duration max_age_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;
};

/**
 * @brief Everything the adapter plugins share on one adapter node.
 *
 * The connector loads each handler as its own plugin instance, but they drive one rover: the
 * pause actions and the navigation handler must see the same route, and every handler reads the
 * same rover state. forNode() hands all of them the same instance, created on first use.
 */
class RoverLink
{
public:
    static std::shared_ptr<RoverLink> forNode(rclcpp::Node * node);

    explicit RoverLink(rclcpp::Node & node);

    application::NavigationUseCase & navigation() { return *navigation_; }
    application::AuxOutputUseCase & auxOutputs() { return *aux_outputs_; }
    application::DriveModeUseCase & driveMode() { return *drive_mode_use_case_; }
    application::FollowMeUseCase & followMe() { return *follow_me_use_case_; }
    const RoverLinkConfig & config() const { return config_; }

    /** @brief Current state. Non-blocking; safe on the executor thread. */
    RoverSnapshot snapshot() const;

    /** @brief Called on the executor thread after every mission_state sample. */
    void setProgressListener(std::function<void()> listener);
    void clearProgressListener();

private:
    void onMissionState(const rover_msgs::msg::MissionState & msg);
    bool fresh(const rclcpp::Time & received) const;

    rclcpp::Node & node_;
    RoverLinkConfig config_;
    std::shared_ptr<TfPoseSource> pose_source_;
    std::unique_ptr<application::NavigationUseCase> navigation_;
    std::unique_ptr<application::AuxOutputUseCase> aux_outputs_;
    std::unique_ptr<application::DriveModeUseCase> drive_mode_use_case_;
    std::unique_ptr<application::FollowMeUseCase> follow_me_use_case_;

    mutable std::mutex mutex_;
    std::optional<rover_msgs::msg::DriveMode> drive_mode_;
    std::optional<sensor_msgs::msg::BatteryState> battery_;
    std::optional<rover_msgs::msg::SafetyStatus> safety_;
    rclcpp::Time safety_received_;
    std::optional<bool> motion_locked_;
    rclcpp::Time motion_lock_received_;
    std::optional<rover_msgs::msg::LocalizationState> localization_;
    std::optional<nav_msgs::msg::Odometry> odom_;
    rclcpp::Time odom_received_;
    std::function<void()> progress_listener_;

    rclcpp::Subscription<rover_msgs::msg::MissionState>::SharedPtr mission_state_sub_;
    rclcpp::Subscription<rover_msgs::msg::DriveMode>::SharedPtr drive_mode_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<rover_msgs::msg::SafetyStatus>::SharedPtr safety_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motion_lock_sub_;
    rclcpp::Subscription<rover_msgs::msg::LocalizationState>::SharedPtr localization_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace rover_vda5050_adapter::infrastructure

#endif  // ROVER_VDA5050_ADAPTER_INFRASTRUCTURE_ROVER_LINK_HPP_
