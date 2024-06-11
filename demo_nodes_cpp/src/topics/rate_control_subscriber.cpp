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
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rcpputils/join.hpp"

#include "std_msgs/msg/int64.hpp"

#include "demo_nodes_cpp/visibility_control.h"

namespace demo_nodes_cpp
{
// Rate-Control const parameters, 0 division remainder, 5 divisor (slower rate)
constexpr std::array<int64_t, 2> RATE_CONTROL_CONFIG {0, 5};

// Create a RateControlSubscriber class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class RateControlSubscriber : public rclcpp::Node
{
public:
  DEMO_NODES_CPP_PUBLIC
  explicit RateControlSubscriber(const rclcpp::NodeOptions & options)
  : Node("rate_control_subscriber", options)
  {
    // Create a callback function for when messages are received.
    auto callback =
      [this](const std_msgs::msg::Int64 & msg) -> void
      {
        RCLCPP_INFO(
          this->get_logger(),
          "I receive an message data: [%ld]", msg.data);
      };

    // Initialize a subscription with a content filter to receive data with
    // one-fifth 1/5 frequency on the subscription. (2 Hz but publisher 10 Hz)
    rclcpp::SubscriptionOptions sub_options;
    sub_options.content_filter_options.filter_expression = "%0 == data % %1";
    sub_options.content_filter_options.expression_parameters = {
      std::to_string(RATE_CONTROL_CONFIG[0]),
      std::to_string(RATE_CONTROL_CONFIG[1])
    };

    sub_ = create_subscription<std_msgs::msg::Int64>("counter", 10, callback, sub_options);

    if (!sub_->is_cft_enabled()) {
      RCLCPP_WARN(
        this->get_logger(), "Content filter is not enabled since it's not supported");
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "subscribed to topic \"%s\" with content filter options \"%s, {%s}\"",
        sub_->get_topic_name(),
        sub_options.content_filter_options.filter_expression.c_str(),
        rcpputils::join(sub_options.content_filter_options.expression_parameters, ", ").c_str());
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_;
};

}  // namespace demo_nodes_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(demo_nodes_cpp::RateControlSubscriber)
