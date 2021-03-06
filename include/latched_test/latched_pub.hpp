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

#pragma once

// C++ includes
#include <future>
#include <thread>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Local includes

namespace MMBO {

class LatchedPub final : public rclcpp::Node
{
public:
  LatchedPub();

  ~LatchedPub();

private:
  std::shared_future<void>                                                future_;
  std::promise<void>                                                      exit_signal_;
  std::thread                                                             pub_thread_;
  void publishThread(std::shared_future<void> local_future);

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_pub_;

};

} // namespace MMBO
