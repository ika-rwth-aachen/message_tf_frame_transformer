/*
==============================================================================
MIT License

Copyright 2023 Institute for Automotive Engineering of RWTH Aachen University.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
==============================================================================
*/


#pragma once

#include <memory>
#include <string>
#include <type_traits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace message_tf_frame_transformer {


// definitions for distinguishing between ROS messages with/without header
// based on SFINAE (https://fekir.info/post/detect-member-variables/)
using HasRosHeaderNo = std::false_type;
using HasRosHeaderYes = std::true_type;

template <typename T, typename = std_msgs::msg::Header>
struct HasRosHeader : HasRosHeaderNo {};

template <typename T>
struct HasRosHeader<T, decltype(T::header)> : HasRosHeaderYes {};


class MessageTfFrameTransformer : public rclcpp::Node {

 public:

  MessageTfFrameTransformer();

 protected:

  void loadParameters();

  void setup();

  void detectMessageType();

  void transformGeneric(const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg);

  template <typename T>
  void transform(const T& msg);

  template <typename T>
  void transform(const T& msg, const HasRosHeaderYes&);

  template <typename T>
  void transform(const T& msg, const HasRosHeaderNo&);

 protected:

  static const std::string kInputTopic;

  static const std::string kOutputTopic;

  static const std::string kSourceFrameIdParam;

  static const std::string kTargetFrameIdParam;

  std::string source_frame_id_;

  std::string target_frame_id_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr detect_message_type_timer_;

  rclcpp::GenericSubscription::SharedPtr subscriber_;
  
  rclcpp::PublisherBase::SharedPtr publisher_;

  std::string msg_type_;
};


template <typename T>
void MessageTfFrameTransformer::transform(const T& msg) {
  this->transform<T>(msg, HasRosHeader<T>());
}

template <typename T>
void MessageTfFrameTransformer::transform(const T& msg, const HasRosHeaderYes&) {

  // transform
  T tf_msg;
  try {
    tf_buffer_->transform(msg, tf_msg, target_frame_id_);
  } catch (tf2::LookupException &e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to lookup transform from '%s' to '%s': %s", msg.header.frame_id.c_str(), target_frame_id_.c_str(), e.what()
    );
    return;
  }

  // publish
  RCLCPP_DEBUG(
    this->get_logger(),
    "Publishing data transformed from '%s' to '%s'", msg.header.frame_id.c_str(), target_frame_id_.c_str()
  );
  std::static_pointer_cast<rclcpp::Publisher<T>>(publisher_)->publish(tf_msg);
}

template <typename T>
void MessageTfFrameTransformer::transform(const T& msg, const HasRosHeaderNo&) {

  if (source_frame_id_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Transforming messages without an 'std_msgs/Header' requires the '%s' parameter to be set", kSourceFrameIdParam.c_str()
    );
    return;
  }

  // lookup transform
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(target_frame_id_, source_frame_id_, tf2::TimePointZero);
  } catch (tf2::LookupException &e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed to lookup transform from '%s' to '%s': %s", source_frame_id_.c_str(), target_frame_id_.c_str(), e.what()
    );
    return;
  }

  // transform
  T tf_msg;
  tf2::doTransform(msg, tf_msg, tf);

  // publish
  RCLCPP_DEBUG(
    this->get_logger(),
    "Publishing data transformed from '%s' to '%s'", source_frame_id_.c_str(), target_frame_id_.c_str()
  );
  std::static_pointer_cast<rclcpp::Publisher<T>>(publisher_)->publish(tf_msg);
}


}  // namespace message_tf_frame_transformer
