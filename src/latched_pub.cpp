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
#include <chrono>
#include <future>
#include <thread>

// ROS 2 includes
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

// Local includes
#include "latched_test/latched_pub.hpp"

namespace MMBO {

LatchedPub::LatchedPub() : rclcpp::Node("latched_pub_node")
{
  future_ = exit_signal_.get_future();

  test_pub_ =
    this->create_publisher<std_msgs::msg::String>("/latched_test",
                                                       rmw_qos_profile_default);

  pub_thread_ = std::thread(&LatchedPub::publishThread, this, future_);
}

void LatchedPub::publishThread(std::shared_future<void> local_future)
{
  std::future_status status;

  do {
    status = local_future.wait_for(std::chrono::milliseconds(100));
  } while (status == std::future_status::timeout);
}

LatchedPub::~LatchedPub()
{
  exit_signal_.set_value();
  pub_thread_.join();
}

} // namespace MMBO
