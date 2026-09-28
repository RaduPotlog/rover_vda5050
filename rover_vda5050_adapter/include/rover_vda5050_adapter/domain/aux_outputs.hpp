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

#ifndef ROVER_VDA5050_ADAPTER_DOMAIN_AUX_OUTPUTS_HPP_
#define ROVER_VDA5050_ADAPTER_DOMAIN_AUX_OUTPUTS_HPP_

#include <string>
#include <vector>

namespace rover_vda5050_adapter::domain
{

/// Aux outputs on the safety PLC (DIO00..DIO05). VDA 5050 numbers them 1..6, as the drive UI does.
constexpr int kAuxOutputCount = 6;

/** @brief The aux outputs one action names, or why its parameter was not understood. */
struct AuxOutputSelection
{
    /// 0-based (output 1 = index 0 = DIO00), ascending, without duplicates.
    std::vector<int> indices;
    /// Non-empty when the parameter was rejected; @c indices is then empty.
    std::string error;

    bool ok() const { return error.empty(); }
};

/**
 * @brief Parse the `outputs` action parameter.
 *
 * The connector hands every actionParameters value over as Python's str() of the JSON value, so
 * `3`, `3.0`, `"3"`, `[1, 3]` and `['1', '3']` all arrive as text; `1,3` is accepted as well.
 * Every number must be a whole number in 1..kAuxOutputCount. Never throws.
 */
AuxOutputSelection parseAuxOutputs(const std::string & value);

}  // namespace rover_vda5050_adapter::domain

#endif  // ROVER_VDA5050_ADAPTER_DOMAIN_AUX_OUTPUTS_HPP_
