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

#include "rover_vda5050_adapter/infrastructure/rover_link.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>

#include "tf2/exceptions.hpp"

#include "rover_vda5050_adapter/infrastructure/mission_manager_client.hpp"

namespace rover_vda5050_adapter::infrastructure
{

namespace
{

std::string stripSlashes(const std::string & name)
{
    const auto first = name.find_first_not_of('/');
    if (first == std::string::npos) {
        return {};
    }

    const auto last = name.find_last_not_of('/');
    return name.substr(first, last - first + 1);
}

// Topic and service names are relative to the rover namespace, not to the adapter node's own
// (the connector runs under <namespace>/vda5050). Absolute names are used as given.
std::string resolveName(const std::string & rover_namespace, const std::string & name)
{
    if (!name.empty() && name.front() == '/') {
        return name;
    }

    const auto ns = stripSlashes(rover_namespace);
    return ns.empty() ? "/" + name : "/" + ns + "/" + name;
}

// Frames follow the rover's tf_prefix convention (<namespace>/map); a frame that already names a
// prefix is used as given.
std::string resolveFrame(const std::string & rover_namespace, const std::string & frame)
{
    const auto stripped = stripSlashes(frame);

    if (stripped.find('/') != std::string::npos) {
        return stripped;
    }

    const auto ns = stripSlashes(rover_namespace);
    return ns.empty() ? stripped : ns + "/" + stripped;
}

domain::OrientationMode parseOrientationMode(const std::string & mode)
{
    if (mode == "node") {
        return domain::OrientationMode::kNode;
    }

    if (mode == "path") {
        return domain::OrientationMode::kPath;
    }

    throw std::invalid_argument(
        "rover.orientation_mode must be 'path' or 'node', got '" + mode + "'");
}

domain::MissionPhase phaseFor(std::uint8_t state)
{
    using rover_msgs::msg::MissionState;

    switch (state) {
        case MissionState::RUNNING:
            return domain::MissionPhase::kRunning;
        case MissionState::HELD:
            return domain::MissionPhase::kHeld;
        case MissionState::SUCCEEDED:
            return domain::MissionPhase::kSucceeded;
        case MissionState::FAILED:
            return domain::MissionPhase::kFailed;
        case MissionState::CANCELLED:
            return domain::MissionPhase::kCancelled;
        default:
            return domain::MissionPhase::kIdle;
    }
}

domain::DriveMode driveModeFor(std::uint8_t mode)
{
    using rover_msgs::msg::DriveMode;

    switch (mode) {
        case DriveMode::MANUAL:
            return domain::DriveMode::kManual;
        case DriveMode::ASSISTED:
            return domain::DriveMode::kAssisted;
        case DriveMode::AUTOMATIC:
            return domain::DriveMode::kAutomatic;
        default:
            return domain::DriveMode::kUnknown;
    }
}

domain::Guard guardFor(std::uint8_t guard)
{
    using rover_msgs::msg::DriveMode;

    switch (guard) {
        case DriveMode::GUARD_BYPASSED:
            return domain::Guard::kBypassed;
        case DriveMode::GUARD_CLEAR:
            return domain::Guard::kClear;
        case DriveMode::GUARD_SLOWING:
            return domain::Guard::kSlowing;
        case DriveMode::GUARD_STOPPED:
            return domain::Guard::kStopped;
        case DriveMode::GUARD_NO_DATA:
            return domain::Guard::kNoData;
        default:
            return domain::Guard::kUnknown;
    }
}

domain::BatteryReading batteryReadingFor(const sensor_msgs::msg::BatteryState & msg)
{
    using sensor_msgs::msg::BatteryState;

    domain::BatteryReading reading;

    // Same rule as rover_mission_manager and rover_safety: an UNKNOWN-status report carries no
    // real charge.
    if (msg.power_supply_status != BatteryState::POWER_SUPPLY_STATUS_UNKNOWN &&
        std::isfinite(msg.percentage))
    {
        reading.fraction = msg.percentage;
    }

    if (std::isfinite(msg.voltage) && msg.voltage > 0.0F) {
        reading.voltage = msg.voltage;
    }

    reading.charging = msg.power_supply_status == BatteryState::POWER_SUPPLY_STATUS_CHARGING;

    return reading;
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

RoverLinkConfig RoverLinkConfig::fromParameters(rclcpp::Node & node)
{
    RoverLinkConfig config;

    config.rover_namespace = node.declare_parameter<std::string>("rover.namespace", "");
    config.map_frame = resolveFrame(
        config.rover_namespace, node.declare_parameter<std::string>("rover.map_frame", "map"));
    config.base_frame = resolveFrame(
        config.rover_namespace,
        node.declare_parameter<std::string>("rover.base_frame", "base_link"));
    config.map_id = node.declare_parameter<std::string>("rover.map_id", "");
    config.mission_prefix = node.declare_parameter<std::string>("rover.mission_prefix", "vda5050");
    config.orientation_mode = parseOrientationMode(
        node.declare_parameter<std::string>("rover.orientation_mode", "path"));

    const auto name = [&](const std::string & parameter, const std::string & default_name) {
        return resolveName(
            config.rover_namespace, node.declare_parameter<std::string>(parameter, default_name));
    };

    config.set_mission_service = name("rover.set_mission_service", "set_mission");
    config.run_mission_service = name("rover.run_mission_service", "run_mission");
    config.mission_state_topic = name("rover.mission_state_topic", "mission_state");
    config.drive_mode_topic = name("rover.drive_mode_topic", "drive_mode");
    config.battery_topic = name("rover.battery_topic", "rover_battery/battery_status");
    config.safety_status_topic =
        name("rover.safety_status_topic", "hardware_interface/safety_status");
    config.motion_lock_topic = name("rover.motion_lock_topic", "motion_lock");
    config.localization_state_topic =
        name("rover.localization_state_topic", "localization_state");
    config.odom_topic = name("rover.odom_topic", "odom");

    config.service_availability_timeout =
        node.declare_parameter<double>("rover.service_availability_timeout", 2.0);
    config.service_response_timeout =
        node.declare_parameter<double>("rover.service_response_timeout", 3.0);
    config.pose_timeout = node.declare_parameter<double>("rover.pose_timeout", 2.0);
    config.status_timeout = node.declare_parameter<double>("rover.status_timeout", 2.0);

    return config;
}

TfPoseSource::TfPoseSource(
    rclcpp::Node & node, std::string map_frame, std::string base_frame, double max_age)
: node_(node),
  map_frame_(std::move(map_frame)),
  base_frame_(std::move(base_frame)),
  max_age_(rclcpp::Duration::from_seconds(max_age)),
  buffer_(std::make_shared<tf2_ros::Buffer>(node.get_clock())),
  listener_(std::make_shared<tf2_ros::TransformListener>(*buffer_, &node, true))
{
}

std::optional<domain::Pose2D> TfPoseSource::currentPose() const
{
    geometry_msgs::msg::TransformStamped transform;

    try {
        transform = buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
        return std::nullopt;
    }

    // A chain whose dynamic links stopped publishing still resolves at its last common time.
    const rclcpp::Time stamp(transform.header.stamp, node_.get_clock()->get_clock_type());
    if (stamp.nanoseconds() != 0 && node_.now() - stamp > max_age_) {
        return std::nullopt;
    }

    return domain::Pose2D{
        transform.transform.translation.x, transform.transform.translation.y,
        yawOf(transform.transform.rotation)};
}

std::shared_ptr<RoverLink> RoverLink::forNode(rclcpp::Node * node)
{
    if (node == nullptr) {
        throw std::invalid_argument("RoverLink needs the adapter node");
    }

    static std::mutex registry_mutex;
    static std::map<rclcpp::Node *, std::weak_ptr<RoverLink>> registry;

    std::lock_guard<std::mutex> lock(registry_mutex);
    auto & entry = registry[node];

    if (auto link = entry.lock()) {
        return link;
    }

    auto link = std::make_shared<RoverLink>(*node);
    entry = link;

    return link;
}

RoverLink::RoverLink(rclcpp::Node & node)
: node_(node),
  config_(RoverLinkConfig::fromParameters(node)),
  safety_received_(0, 0, node.get_clock()->get_clock_type()),
  motion_lock_received_(0, 0, node.get_clock()->get_clock_type()),
  odom_received_(0, 0, node.get_clock()->get_clock_type())
{
    using std::placeholders::_1;

    pose_source_ = std::make_shared<TfPoseSource>(
        node_, config_.map_frame, config_.base_frame, config_.pose_timeout);

    auto missions = std::make_shared<MissionManagerClient>(
        node_, config_.set_mission_service, config_.run_mission_service, config_.map_frame,
        std::chrono::duration<double>(config_.service_availability_timeout),
        std::chrono::duration<double>(config_.service_response_timeout));

    navigation_ = std::make_unique<application::NavigationUseCase>(
        missions, pose_source_, config_.mission_prefix, config_.orientation_mode);

    // QoS matches each publisher: the latched topics are transient-local depth 1.
    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    mission_state_sub_ = node_.create_subscription<rover_msgs::msg::MissionState>(
        config_.mission_state_topic, latched,
        [this](const rover_msgs::msg::MissionState & msg) { onMissionState(msg); });

    drive_mode_sub_ = node_.create_subscription<rover_msgs::msg::DriveMode>(
        config_.drive_mode_topic, latched, [this](const rover_msgs::msg::DriveMode & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            drive_mode_ = msg;
        });

    localization_sub_ = node_.create_subscription<rover_msgs::msg::LocalizationState>(
        config_.localization_state_topic, latched,
        [this](const rover_msgs::msg::LocalizationState & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            localization_ = msg;
        });

    battery_sub_ = node_.create_subscription<sensor_msgs::msg::BatteryState>(
        config_.battery_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::BatteryState & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            battery_ = msg;
        });

    safety_sub_ = node_.create_subscription<rover_msgs::msg::SafetyStatus>(
        config_.safety_status_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](const rover_msgs::msg::SafetyStatus & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            safety_ = msg;
            safety_received_ = node_.now();
        });

    motion_lock_sub_ = node_.create_subscription<std_msgs::msg::Bool>(
        config_.motion_lock_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
        [this](const std_msgs::msg::Bool & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            motion_locked_ = msg.data;
            motion_lock_received_ = node_.now();
        });

    odom_sub_ = node_.create_subscription<nav_msgs::msg::Odometry>(
        config_.odom_topic, rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry & msg) {
            std::lock_guard<std::mutex> lock(mutex_);
            odom_ = msg;
            odom_received_ = node_.now();
        });

    RCLCPP_INFO_STREAM(
        node_.get_logger(), "VDA 5050 rover link: missions via '"
                                << config_.set_mission_service << "', poses in '"
                                << config_.map_frame << "' -> '" << config_.base_frame << "'.");
}

RoverSnapshot RoverLink::snapshot() const
{
    RoverSnapshot snapshot;
    snapshot.pose = pose_source_->currentPose();
    snapshot.paused = navigation_->paused();
    snapshot.health.mission_refusal = navigation_->lastRefusal();

    // Taken only after the use case's own lock is released: the two never nest.
    std::lock_guard<std::mutex> lock(mutex_);

    snapshot.map_id = config_.map_id;
    if (localization_ && !localization_->map_name.empty()) {
        snapshot.map_id = localization_->map_name;
    }

    if (odom_ && fresh(odom_received_)) {
        snapshot.velocity.vx = odom_->twist.twist.linear.x;
        snapshot.velocity.vy = odom_->twist.twist.linear.y;
        snapshot.velocity.omega = odom_->twist.twist.angular.z;
    }

    if (battery_) {
        snapshot.battery = batteryReadingFor(*battery_);
    }

    auto & health = snapshot.health;
    health.pose_known = snapshot.pose.has_value();

    if (drive_mode_) {
        health.drive_mode = driveModeFor(drive_mode_->mode);
        health.guard = guardFor(drive_mode_->guard);
    }

    if (safety_) {
        health.safety = domain::SafetyReading{
            safety_->hw_e_stop_user_button, safety_->latch_active,
            // Stale data is as untrustworthy as data the PLC link itself flags.
            safety_->link_healthy && fresh(safety_received_)};
    }

    if (motion_locked_ && fresh(motion_lock_received_)) {
        health.motion_locked = *motion_locked_;
    }

    return snapshot;
}

void RoverLink::setProgressListener(std::function<void()> listener)
{
    std::lock_guard<std::mutex> lock(mutex_);
    progress_listener_ = std::move(listener);
}

void RoverLink::clearProgressListener()
{
    std::lock_guard<std::mutex> lock(mutex_);
    progress_listener_ = nullptr;
}

void RoverLink::onMissionState(const rover_msgs::msg::MissionState & msg)
{
    navigation_->onMissionProgress(
        domain::MissionProgress{
            msg.mission_id, phaseFor(msg.state), msg.current_index, msg.total, msg.message});

    std::function<void()> listener;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        listener = progress_listener_;
    }

    if (listener) {
        listener();
    }
}

bool RoverLink::fresh(const rclcpp::Time & received) const
{
    return received.nanoseconds() != 0 &&
           (node_.now() - received).seconds() <= config_.status_timeout;
}

}  // namespace rover_vda5050_adapter::infrastructure
