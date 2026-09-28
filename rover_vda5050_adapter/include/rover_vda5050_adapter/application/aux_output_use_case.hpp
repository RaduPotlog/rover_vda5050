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

#ifndef ROVER_VDA5050_ADAPTER_APPLICATION_AUX_OUTPUT_USE_CASE_HPP_
#define ROVER_VDA5050_ADAPTER_APPLICATION_AUX_OUTPUT_USE_CASE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rover_vda5050_adapter/application/ports.hpp"

namespace rover_vda5050_adapter::application
{

/**
 * @brief VDA 5050 enableAuxOutput / disableAuxOutput: switch the aux outputs an action names.
 *
 * Shared by both action plugins, which run on threads of their own; calls are serialized, so the
 * outputs end up in the order the actions were started. Blocks on AuxOutputPort: never call it
 * from the executor thread.
 */
class AuxOutputUseCase
{
public:
    explicit AuxOutputUseCase(std::shared_ptr<AuxOutputPort> outputs);

    /**
     * @brief Switch every output named by @p outputs_value (the `outputs` action parameter).
     *
     * An unparsable parameter writes nothing. Otherwise every named output is written, in
     * ascending order, even after one fails; the result is ok only when all of them succeeded.
     */
    CommandResult apply(const std::string & outputs_value, bool enabled);

private:
    std::shared_ptr<AuxOutputPort> outputs_;
    std::mutex mutex_;
};

}  // namespace rover_vda5050_adapter::application

#endif  // ROVER_VDA5050_ADAPTER_APPLICATION_AUX_OUTPUT_USE_CASE_HPP_
