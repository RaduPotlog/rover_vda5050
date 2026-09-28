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

#include <map>
#include <stdexcept>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rover_vda5050_adapter/application/aux_output_use_case.hpp"

using rover_vda5050_adapter::application::AuxOutputPort;
using rover_vda5050_adapter::application::AuxOutputUseCase;
using rover_vda5050_adapter::application::CommandResult;

namespace
{

class FakeAuxOutputs : public AuxOutputPort
{
public:
    CommandResult set(int index, bool enabled) override
    {
        writes.emplace_back(index, enabled);
        const auto failure = failures.find(index);
        return failure == failures.end() ? CommandResult{true, "", false} : failure->second;
    }

    std::vector<std::pair<int, bool>> writes;
    std::map<int, CommandResult> failures;
};

struct Fixture : ::testing::Test
{
    std::shared_ptr<FakeAuxOutputs> outputs{std::make_shared<FakeAuxOutputs>()};
    AuxOutputUseCase use_case{outputs};
};

}  // namespace

TEST_F(Fixture, EnablesEveryNamedOutputInOrder)
{
    const auto result = use_case.apply("[5, 1]", true);

    EXPECT_TRUE(result.ok) << result.message;
    EXPECT_EQ(result.message, "Aux outputs 1, 5 enabled.");
    EXPECT_EQ(outputs->writes, (std::vector<std::pair<int, bool>>{{0, true}, {4, true}}));
}

TEST_F(Fixture, DisablesOneOutput)
{
    const auto result = use_case.apply("3", false);

    EXPECT_TRUE(result.ok) << result.message;
    EXPECT_EQ(result.message, "Aux output 3 disabled.");
    EXPECT_EQ(outputs->writes, (std::vector<std::pair<int, bool>>{{2, false}}));
}

TEST_F(Fixture, BadParameterWritesNothing)
{
    const auto result = use_case.apply("[1, 7]", true);

    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.outcome_unknown);
    EXPECT_TRUE(outputs->writes.empty());
}

TEST_F(Fixture, FailedWriteStillWritesTheRest)
{
    outputs->failures[1] = {false, "PLC link down", false};

    const auto result = use_case.apply("[1, 2, 3]", true);

    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.outcome_unknown);
    EXPECT_EQ(outputs->writes.size(), 3U);
    EXPECT_EQ(result.message, "Aux output 2 not enabled: PLC link down; Aux outputs 1, 3 enabled");
}

TEST_F(Fixture, TimeoutMakesTheOutcomeUnknown)
{
    outputs->failures[0] = {false, "did not answer in time.", true};

    const auto result = use_case.apply("1", false);

    EXPECT_FALSE(result.ok);
    EXPECT_TRUE(result.outcome_unknown);
    EXPECT_EQ(result.message, "Aux output 1 not disabled: did not answer in time.");
}

TEST(AuxOutputUseCase, NeedsAPort)
{
    EXPECT_THROW(AuxOutputUseCase(nullptr), std::invalid_argument);
}
