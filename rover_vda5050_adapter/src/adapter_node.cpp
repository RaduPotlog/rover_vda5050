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

// Composition root: the connector's own adapter node, which loads this package's handler
// plugins (plugins.xml) by the names in config/connector.yaml of rover_vda5050_bringup.
// vda5050_connector ships the node as a library only, so every robot provides this main().

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_connector/adapter.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Name and namespace come from the launch file (__node / __ns remaps).
    auto node = std::make_shared<adapter::AdapterNode>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
