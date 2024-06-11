// Copyright 2024 Sony Group Corporation.
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

#include <array>
#include <chrono>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "std_msgs/msg/int64.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{

// Create a RateControlPublisher class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class RateControlPublisher final : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit RateControlPublisher(const rclcpp::NodeOptions & options)
  : Node("rate_control_publisher", options)
  {
    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
        msg_ = std::make_unique<std_msgs::msg::Int64>();
        msg_->data = counter_++;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", msg_->data);
        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(std::move(msg_));
      };
    // Create a publisher with a custom Quality of Service profile.
    rclcpp::QoS qos(rclcpp::KeepLast{10});
    pub_ = this->create_publisher<std_msgs::msg::Int64>("counter", qos);

    // Publisher sends message with 10 Hz (100 msec)
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), publish_message);
  }

private:
  std::atomic<int64_t> counter_ {0};
  std::unique_ptr<std_msgs::msg::Int64> msg_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::RateControlPublisher)
