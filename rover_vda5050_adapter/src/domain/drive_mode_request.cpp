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

#include "rover_vda5050_adapter/domain/drive_mode_request.hpp"

#include <algorithm>
#include <cctype>

namespace rover_vda5050_adapter::domain
{

namespace
{

std::string normalized(const std::string & value)
{
    const auto blank_or_quote = [](unsigned char c) {
        return std::isspace(c) || c == '"' || c == '\'';
    };

    auto begin = value.begin();
    auto end = value.end();

    while (begin != end && blank_or_quote(*begin)) {
        ++begin;
    }

    while (end != begin && blank_or_quote(*(end - 1))) {
        --end;
    }

    std::string text(begin, end);
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::toupper(c));
    });

    return text;
}

}  // namespace

DriveModeRequest parseDriveModeRequest(const std::string & value)
{
    const std::string mode = normalized(value);

    if (mode == "MANUAL") {
        return {DriveMode::kManual, ""};
    }

    if (mode == "AUTOMATIC") {
        return {DriveMode::kAutomatic, ""};
    }

    if (mode == "ASSISTED") {
        return {DriveMode::kUnknown,
            "Assisted can only be chosen on the rover's drive UI; use MANUAL or AUTOMATIC."};
    }

    return {DriveMode::kUnknown,
        "Unknown drive mode '" + value + "'; use MANUAL or AUTOMATIC."};
}

std::string driveModeName(DriveMode mode)
{
    switch (mode) {
        case DriveMode::kManual:
            return "Manual";
        case DriveMode::kAssisted:
            return "Assisted";
        case DriveMode::kAutomatic:
            return "Automatic";
        case DriveMode::kUnknown:
            break;
    }

    return "unknown";
}

}  // namespace rover_vda5050_adapter::domain
