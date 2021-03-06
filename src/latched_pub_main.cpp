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
#include <memory>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>

// Local includes
#include "latched_test/latched_pub.hpp"

int main(int argc, char const* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MMBO::LatchedPub>());

  rclcpp::shutdown();

  return 0;
}
