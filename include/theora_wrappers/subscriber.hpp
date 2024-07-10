/**
 * Theora subscriber wrapper, based on image_transport.
 *
 * Roberto Masocco <r.masocco@dotxautomation.com>
 *
 * June 18, 2023
 */

/**
 * Copyright 2024 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THEORA_WRAPPERS__SUBSCRIBER_HPP_
#define THEORA_WRAPPERS__SUBSCRIBER_HPP_

#include "visibility_control.h"

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <std_srvs/srv/trigger.hpp>

namespace TheoraWrappers
{

typedef std::function<void (const sensor_msgs::msg::Image::ConstSharedPtr &)> Callback;

/**
 * Theora subscriber wrapper: invokes publisher reset to trigger header retransmissions.
 */
class THEORA_WRAPPERS_PUBLIC Subscriber
{
public:
  Subscriber(
    rclcpp::Node * node,
    const std::string & base_topic,
    Callback && callback,
    rmw_qos_profile_t qos,
    bool spin = true);
  virtual ~Subscriber();

  void shutdown();

private:
  rclcpp::Node * node_;
  std::string base_topic_;
  rmw_qos_profile_t qos_;
  std::shared_ptr<image_transport::Subscriber> sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
};

} // namespace TheoraWrappers

#endif // THEORA_WRAPPERS__SUBSCRIBER_HPP_
