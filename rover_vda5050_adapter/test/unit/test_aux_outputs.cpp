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
#include <vector>

#include "rover_vda5050_adapter/domain/aux_outputs.hpp"

using rover_vda5050_adapter::domain::parseAuxOutputs;

namespace
{

std::vector<int> indicesOf(const std::string & value)
{
    const auto selection = parseAuxOutputs(value);
    EXPECT_TRUE(selection.ok()) << value << ": " << selection.error;
    return selection.indices;
}

}  // namespace

TEST(AuxOutputs, SingleNumberIsOneBased)
{
    EXPECT_EQ(indicesOf("1"), std::vector<int>({0}));
    EXPECT_EQ(indicesOf("6"), std::vector<int>({5}));
    EXPECT_EQ(indicesOf(" 3 "), std::vector<int>({2}));
}

TEST(AuxOutputs, AcceptsWhatTheConnectorHandsOver)
{
    // Python str() of the JSON values 3.0, "3", [1, 3] and ["1", "3"].
    EXPECT_EQ(indicesOf("3.0"), std::vector<int>({2}));
    EXPECT_EQ(indicesOf("'3'"), std::vector<int>({2}));
    EXPECT_EQ(indicesOf("[1, 3]"), std::vector<int>({0, 2}));
    EXPECT_EQ(indicesOf("['1', '3']"), std::vector<int>({0, 2}));
    EXPECT_EQ(indicesOf("[5]"), std::vector<int>({4}));
    EXPECT_EQ(indicesOf("1,3"), std::vector<int>({0, 2}));
}

TEST(AuxOutputs, SortsAndDropsDuplicates)
{
    EXPECT_EQ(indicesOf("[6, 2, 6, 2]"), std::vector<int>({1, 5}));
    EXPECT_EQ(indicesOf("[3, 3]"), std::vector<int>({2}));
}

TEST(AuxOutputs, RejectsWithoutThrowing)
{
    for (const std::string value :
        {"", "  ", "[]", "0", "7", "-1", "2.5", "abc", "[1, x]", "1,", ",1", "[1, 3",
            "nan", "inf", "1e9", "True", "99999999999999999999"})
    {
        EXPECT_NO_THROW({
            const auto selection = parseAuxOutputs(value);
            EXPECT_FALSE(selection.ok()) << "accepted '" << value << "'";
            EXPECT_TRUE(selection.indices.empty()) << value;
        });
    }
}

TEST(AuxOutputs, ErrorNamesTheBadValue)
{
    const auto selection = parseAuxOutputs("[1, 7]");
    ASSERT_FALSE(selection.ok());
    EXPECT_NE(selection.error.find("'7'"), std::string::npos) << selection.error;
    EXPECT_NE(selection.error.find("1..6"), std::string::npos) << selection.error;
}
