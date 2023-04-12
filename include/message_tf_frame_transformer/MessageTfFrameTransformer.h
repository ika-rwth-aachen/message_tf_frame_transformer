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

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace message_tf_frame_transformer {


// definitions for distinguishing between ROS messages with/without header
// based on SFINAE (https://fekir.info/post/detect-member-variables/)
using HasRosHeaderNo = std::false_type;
using HasRosHeaderYes = std::true_type;

template <typename T, typename = std_msgs::Header>
struct HasRosHeader : HasRosHeaderNo {};

template <typename T>
struct HasRosHeader<T, decltype(T::header)> : HasRosHeaderYes {};


class MessageTfFrameTransformer : public nodelet::Nodelet {

 protected:

  virtual void onInit() override;

  void loadParameters();

  void setup();

  void detectMessageType(const topic_tools::ShapeShifter::ConstPtr& generic_msg);

  template <typename T>
  void transform(const typename T::ConstPtr& msg);

  template <typename T>
  void transform(const typename T::ConstPtr& msg, const HasRosHeaderYes&);

  template <typename T>
  void transform(const typename T::ConstPtr& msg, const HasRosHeaderNo&);

 protected:

  static const std::string kInputTopic;

  static const std::string kOutputTopic;

  static const std::string kSourceFrameIdParam;

  static const std::string kTargetFrameIdParam;

  std::string source_frame_id_;

  std::string target_frame_id_;

  ros::NodeHandle node_handle_;

  ros::NodeHandle private_node_handle_;

  tf2_ros::Buffer tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Subscriber subscriber_;

  ros::Publisher publisher_;
};


template <typename T>
void MessageTfFrameTransformer::transform(const typename T::ConstPtr& msg) {
  this->transform<T>(msg, HasRosHeader<T>());
}

template <typename T>
void MessageTfFrameTransformer::transform(const typename T::ConstPtr& msg, const HasRosHeaderYes&) {

  // transform
  T tf_msg;
  try {
    tf_buffer_.transform(*msg, tf_msg, target_frame_id_);
  } catch (tf2::LookupException &e) {
    NODELET_ERROR("Failed to lookup transform from '%s' to '%s': %s", msg->header.frame_id.c_str(), target_frame_id_.c_str(), e.what());
    return;
  }

  // publish
  NODELET_DEBUG("Publishing data transformed from '%s' to '%s'", msg->header.frame_id.c_str(), target_frame_id_.c_str());
  publisher_.publish(tf_msg);
}

template <typename T>
void MessageTfFrameTransformer::transform(const typename T::ConstPtr& msg, const HasRosHeaderNo&) {

  if (source_frame_id_.empty()) {
    NODELET_ERROR("Transforming messages without an 'std_msgs/Header' requires the '%s' parameter to be set", kSourceFrameIdParam.c_str());
    return;
  }

  // lookup transform
  geometry_msgs::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(target_frame_id_, source_frame_id_, ros::Time(0));
  } catch (tf2::LookupException &e) {
    NODELET_ERROR("Failed to lookup transform from '%s' to '%s': %s", source_frame_id_.c_str(), target_frame_id_.c_str(), e.what());
    return;
  }

  // transform
  T tf_msg;
  tf2::doTransform(*msg, tf_msg, tf);

  // publish
  NODELET_DEBUG("Publishing data transformed from '%s' to '%s'", source_frame_id_.c_str(), target_frame_id_.c_str());
  publisher_.publish(tf_msg);
}


}  // namespace message_tf_frame_transformer
