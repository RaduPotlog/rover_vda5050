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

#include "rover_vda5050_adapter/application/drive_mode_use_case.hpp"

using rover_vda5050_adapter::application::CommandResult;
using rover_vda5050_adapter::application::DriveModePort;
using rover_vda5050_adapter::application::DriveModeUseCase;
using rover_vda5050_adapter::domain::DriveMode;

namespace
{

class FakeDriveMode : public DriveModePort
{
public:
    CommandResult set(DriveMode mode) override
    {
        requests.push_back(mode);
        return answer;
    }

    std::vector<DriveMode> requests;
    CommandResult answer{true, "", false};
};

struct Fixture : ::testing::Test
{
    std::shared_ptr<FakeDriveMode> drive_mode{std::make_shared<FakeDriveMode>()};
    DriveModeUseCase use_case{drive_mode};
};

}  // namespace

TEST_F(Fixture, SwitchesToManual)
{
    const auto result = use_case.request("MANUAL");

    EXPECT_TRUE(result.ok) << result.message;
    EXPECT_EQ(result.message, "Drive mode Manual.");
    EXPECT_EQ(drive_mode->requests, std::vector<DriveMode>{DriveMode::kManual});
}

TEST_F(Fixture, ReportsTheManagersRefusal)
{
    drive_mode->answer = {false, "AUTOMATIC needs rover_mission_manager", false};

    const auto result = use_case.request("AUTOMATIC");

    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.outcome_unknown);
    EXPECT_EQ(result.message, "Automatic refused: AUTOMATIC needs rover_mission_manager");
}

TEST_F(Fixture, KeepsAnUnknownOutcome)
{
    drive_mode->answer = {false, "'/rover/set_drive_mode' did not answer in time.", true};

    const auto result = use_case.request("MANUAL");

    EXPECT_FALSE(result.ok);
    EXPECT_TRUE(result.outcome_unknown);
}

TEST_F(Fixture, BadParameterAsksNothing)
{
    const auto result = use_case.request("ASSISTED");

    EXPECT_FALSE(result.ok);
    EXPECT_TRUE(drive_mode->requests.empty());
}

TEST(DriveModeUseCase, NeedsAPort)
{
    EXPECT_THROW(DriveModeUseCase(nullptr), std::invalid_argument);
}
