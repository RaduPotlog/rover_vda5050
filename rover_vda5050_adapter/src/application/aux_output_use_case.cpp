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

#include "rover_vda5050_adapter/application/aux_output_use_case.hpp"

#include <stdexcept>
#include <utility>
#include <vector>

#include "rover_vda5050_adapter/domain/aux_outputs.hpp"

namespace rover_vda5050_adapter::application
{

namespace
{

// "Aux output 3" / "Aux outputs 1, 3", numbered 1..6 as VDA 5050 and the drive UI name them.
std::string outputsText(const std::vector<int> & indices)
{
    std::string text = indices.size() == 1 ? "Aux output " : "Aux outputs ";

    for (std::size_t i = 0; i < indices.size(); ++i) {
        text += (i == 0 ? "" : ", ") + std::to_string(indices[i] + 1);
    }

    return text;
}

}  // namespace

AuxOutputUseCase::AuxOutputUseCase(std::shared_ptr<AuxOutputPort> outputs)
: outputs_(std::move(outputs))
{
    if (!outputs_) {
        throw std::invalid_argument("AuxOutputUseCase needs an AuxOutputPort");
    }
}

CommandResult AuxOutputUseCase::apply(const std::string & outputs_value, bool enabled)
{
    const auto selection = domain::parseAuxOutputs(outputs_value);

    if (!selection.ok()) {
        return {false, selection.error, false};
    }

    const std::string verb = enabled ? "enabled" : "disabled";

    std::vector<int> switched;
    std::string failures;
    bool outcome_unknown = false;

    std::lock_guard<std::mutex> lock(mutex_);

    for (const int index : selection.indices) {
        const auto result = outputs_->set(index, enabled);

        if (result.ok) {
            switched.push_back(index);
            continue;
        }

        failures += (failures.empty() ? "" : "; ") + outputsText({index}) + " not " + verb +
                    ": " + result.message;
        outcome_unknown = outcome_unknown || result.outcome_unknown;
    }

    if (failures.empty()) {
        return {true, outputsText(switched) + " " + verb + ".", false};
    }

    if (!switched.empty()) {
        failures += "; " + outputsText(switched) + " " + verb;
    }

    return {false, failures, outcome_unknown};
}

}  // namespace rover_vda5050_adapter::application
