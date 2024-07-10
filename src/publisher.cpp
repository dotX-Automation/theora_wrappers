/**
 * Theora publisher wrapper implementation.
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

#define UNUSED(arg) (void)(arg)

#include <theora_wrappers/publisher.hpp>

namespace TheoraWrappers
{

/**
 * @brief Constructor.
 *
 * @param node The ROS 2 node.
 * @param base_topic The base topic name.
 * @param qos The QoS profile.
 */
Publisher::Publisher(
  rclcpp::Node * node,
  const std::string & base_topic,
  rmw_qos_profile_t qos)
: node_(node),
  base_topic_(base_topic),
  qos_(qos)
{
  // Instantiate reset service
  reset_service_ = node_->create_service<std_srvs::srv::Trigger>(
    base_topic_ + "/stream/reset",
    std::bind(
      &Publisher::reset_callback_,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  // Instantiate publisher
  pub_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      node_,
      base_topic_ + "/stream",
      qos_));
}

/**
 * @brief Destructor.
 */
Publisher::~Publisher()
{
  if (pub_) {
    pub_->shutdown();
    pub_.reset();
  }
}

/**
 * @brief Get the number of subscribers.
 *
 * @return The number of subscribers.
 */
size_t Publisher::getNumSubscribers()
{
  if (pub_lock_.try_lock()) {
    if (pub_) {
      size_t num = pub_->getNumSubscribers();
      pub_lock_.unlock();
      return num;
    }
    pub_lock_.unlock();
  }
  return 0;
}

/**
 * @brief Publish a message.
 *
 * @param message The message to publish.
 */
void Publisher::publish(const sensor_msgs::msg::Image & message)
{
  if (pub_lock_.try_lock()) {
    if (pub_) {
      pub_->publish(message);
    }
    pub_lock_.unlock();
  }
}

/**
 * @brief Publish a message.
 *
 * @param message The message to publish.
 */
void Publisher::publish(const sensor_msgs::msg::Image::ConstSharedPtr & message)
{
  if (pub_lock_.try_lock()) {
    if (pub_) {
      pub_->publish(message);
    }
    pub_lock_.unlock();
  }
}

/**
 * @brief Reset service callback.
 *
 * @param req The request message.
 * @param res The response message.
 */
void Publisher::reset_callback_(
  const std_srvs::srv::Trigger::Request::SharedPtr req,
  const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  UNUSED(req);
  std::unique_lock<std::mutex> lock(pub_lock_);

  if (pub_) {
    pub_.reset();
  }
  pub_ = std::make_shared<image_transport::Publisher>(
    image_transport::create_publisher(
      node_,
      base_topic_ + "/stream",
      qos_));

  res->set__success(true);
  res->set__message("Publisher reset");
}

} // namespace TheoraWrappers
