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

#include <algorithm>
#include <string>
#include <vector>

#include "rover_vda5050_adapter/domain/rover_status.hpp"

using namespace rover_vda5050_adapter::domain;  // NOLINT(build/namespaces)

namespace
{

bool hasError(const std::vector<RoverError> & errors, const std::string & type)
{
    return std::any_of(errors.begin(), errors.end(), [&](const RoverError & e) {
        return e.type == type;
    });
}

}  // namespace

TEST(RoverStatus, OperatingModeFollowsTheDriveMode)
{
    EXPECT_EQ(toVdaString(operatingModeFor(DriveMode::kAutomatic)), "AUTOMATIC");
    // An operator drives in ASSISTED; SEMIAUTOMATIC would tell master control it may send orders.
    EXPECT_EQ(toVdaString(operatingModeFor(DriveMode::kAssisted)), "MANUAL");
    EXPECT_EQ(toVdaString(OperatingMode::kSemiautomatic), "SEMIAUTOMATIC");
    EXPECT_EQ(toVdaString(operatingModeFor(DriveMode::kManual)), "MANUAL");
    EXPECT_EQ(toVdaString(operatingModeFor(DriveMode::kUnknown)), "SERVICE");
}

TEST(RoverStatus, EStopButtonOrLatchIsAManualEStop)
{
    RoverHealth health;
    EXPECT_EQ(safetyFor(health).e_stop, "NONE");

    health.safety = SafetyReading{true, false, true};
    EXPECT_EQ(safetyFor(health).e_stop, "MANUAL");

    health.safety = SafetyReading{false, true, true};
    EXPECT_EQ(safetyFor(health).e_stop, "MANUAL");

    health.safety = SafetyReading{false, false, true};
    EXPECT_EQ(safetyFor(health).e_stop, "NONE");
}

TEST(RoverStatus, CollisionMonitorStopZoneIsAFieldViolation)
{
    RoverHealth health;
    health.guard = Guard::kSlowing;
    EXPECT_FALSE(safetyFor(health).field_violation);

    health.guard = Guard::kStopped;
    EXPECT_TRUE(safetyFor(health).field_violation);
}

TEST(RoverStatus, ErrorsNameEachProblemAndAreNeverFatal)
{
    RoverHealth health;
    health.pose_known = true;
    EXPECT_TRUE(errorsFor(health).empty());

    health.pose_known = false;
    health.safety = SafetyReading{false, false, false};
    health.motion_locked = true;
    health.mission_refusal = "Drive mode is not AUTOMATIC";

    const auto errors = errorsFor(health);
    EXPECT_TRUE(hasError(errors, "localizationUnavailable"));
    EXPECT_TRUE(hasError(errors, "safetyLinkDown"));
    EXPECT_TRUE(hasError(errors, "motionLocked"));
    EXPECT_TRUE(hasError(errors, "missionRefused"));
    // A FATAL error would stop the connector dispatching navigation at all.
    EXPECT_TRUE(std::none_of(errors.begin(), errors.end(), [](const RoverError & e) {
        return e.fatal;
    }));
}

TEST(RoverStatus, BatteryIsReportedInPercentAndUnknownAsZero)
{
    const auto known = batteryFor(BatteryReading{0.42, 25.2, true});
    EXPECT_DOUBLE_EQ(known.charge_percent, 42.0);
    EXPECT_DOUBLE_EQ(known.voltage, 25.2);
    EXPECT_TRUE(known.charging);

    const auto unknown = batteryFor(BatteryReading{});
    EXPECT_DOUBLE_EQ(unknown.charge_percent, 0.0);
    EXPECT_DOUBLE_EQ(unknown.voltage, 0.0);

    EXPECT_DOUBLE_EQ(batteryFor(BatteryReading{1.7, std::nullopt, false}).charge_percent, 100.0);
}
