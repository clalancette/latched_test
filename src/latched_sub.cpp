// Copyright 2018 Toyota Research Institute.  All rights reserved.
//
// IF WE RELEASE THIS CODE, WE MAY USE THE FOLLOWING BOILERPLATE:
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

// C++ includes

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Local includes
#include "latched_test/latched_sub.hpp"

namespace MMBO {

LatchedSub::LatchedSub() : rclcpp::Node("latched_sub_node")
{
  test_sub_ = this->create_subscription<std_msgs::msg::String>("/latched_test", std::bind(&LatchedSub::recv, this, std::placeholders::_1),
                                                               rmw_qos_profile_default);
}

void LatchedSub::recv(const std_msgs::msg::String::SharedPtr msg)
{
  fprintf(stderr, "Saw msg '%s'\n", msg->data.c_str());
}

LatchedSub::~LatchedSub()
{
}

} // namespace MMBO
