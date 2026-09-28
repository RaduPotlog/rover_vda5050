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

#include "rover_vda5050_adapter/domain/aux_outputs.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <string_view>
#include <system_error>
#include <utility>

namespace rover_vda5050_adapter::domain
{

namespace
{

std::string_view trim(std::string_view text, std::string_view characters = " \t\r\n")
{
    const auto first = text.find_first_not_of(characters);
    if (first == std::string_view::npos) {
        return {};
    }

    const auto last = text.find_last_not_of(characters);
    return text.substr(first, last - first + 1);
}

std::string_view unquote(std::string_view text)
{
    if (text.size() >= 2 && (text.front() == '"' || text.front() == '\'') &&
        text.back() == text.front())
    {
        return trim(text.substr(1, text.size() - 2));
    }

    return text;
}

AuxOutputSelection rejected(std::string error) { return {{}, std::move(error)}; }

std::string rangeText()
{
    return "1.." + std::to_string(kAuxOutputCount);
}

}  // namespace

AuxOutputSelection parseAuxOutputs(const std::string & value)
{
    std::string_view list = trim(value);

    if (list.size() >= 2 && list.front() == '[' && list.back() == ']') {
        list = trim(list.substr(1, list.size() - 2));
    }

    if (list.empty()) {
        return rejected("No aux output given; expected " + rangeText() + " or a list of them.");
    }

    AuxOutputSelection selection;

    while (true) {
        const auto comma = list.find(',');
        const auto token = unquote(trim(list.substr(0, comma)));

        double number = 0.0;
        const auto * const end = token.data() + token.size();
        const auto [parsed_end, error] = std::from_chars(token.data(), end, number);

        if (token.empty() || error != std::errc() || parsed_end != end) {
            return rejected(
                "Aux output '" + std::string(token) + "' is not a number; expected " +
                rangeText() + ".");
        }

        if (!std::isfinite(number) || number != std::floor(number) || number < 1.0 ||
            number > kAuxOutputCount)
        {
            return rejected(
                "Aux output '" + std::string(token) + "' does not exist; expected " +
                rangeText() + ".");
        }

        selection.indices.push_back(static_cast<int>(number) - 1);

        if (comma == std::string_view::npos) {
            break;
        }

        list = list.substr(comma + 1);
    }

    std::sort(selection.indices.begin(), selection.indices.end());
    selection.indices.erase(
        std::unique(selection.indices.begin(), selection.indices.end()), selection.indices.end());

    return selection;
}

}  // namespace rover_vda5050_adapter::domain
