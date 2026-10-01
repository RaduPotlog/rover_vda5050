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

#include <string>

#include "rover_vda5050_adapter/domain/drive_mode_request.hpp"

using rover_vda5050_adapter::domain::DriveMode;
using rover_vda5050_adapter::domain::driveModeName;
using rover_vda5050_adapter::domain::parseDriveModeRequest;

TEST(DriveModeRequest, AcceptsManualAndAutomaticAsTheConnectorHandsThemOver)
{
    for (const std::string value : {"MANUAL", "manual", " Manual ", "'MANUAL'", "\"MANUAL\""}) {
        const auto request = parseDriveModeRequest(value);
        EXPECT_TRUE(request.ok()) << value << ": " << request.error;
        EXPECT_EQ(request.mode, DriveMode::kManual) << value;
    }

    const auto automatic = parseDriveModeRequest("AUTOMATIC");
    EXPECT_TRUE(automatic.ok());
    EXPECT_EQ(automatic.mode, DriveMode::kAutomatic);
}

TEST(DriveModeRequest, AssistedStaysOnTheDriveUi)
{
    const auto request = parseDriveModeRequest("ASSISTED");
    EXPECT_FALSE(request.ok());
    EXPECT_EQ(request.mode, DriveMode::kUnknown);
    EXPECT_NE(request.error.find("drive UI"), std::string::npos);
}

TEST(DriveModeRequest, RefusesAnythingElse)
{
    for (const std::string value : {"", "3", "AUTO", "SEMIAUTOMATIC", "MANUAL AUTOMATIC"}) {
        const auto request = parseDriveModeRequest(value);
        EXPECT_FALSE(request.ok()) << value;
        EXPECT_EQ(request.mode, DriveMode::kUnknown) << value;
    }
}

TEST(DriveModeRequest, NamesModesAsTheDriveUi)
{
    EXPECT_EQ(driveModeName(DriveMode::kManual), "Manual");
    EXPECT_EQ(driveModeName(DriveMode::kAssisted), "Assisted");
    EXPECT_EQ(driveModeName(DriveMode::kAutomatic), "Automatic");
    EXPECT_EQ(driveModeName(DriveMode::kUnknown), "unknown");
}
