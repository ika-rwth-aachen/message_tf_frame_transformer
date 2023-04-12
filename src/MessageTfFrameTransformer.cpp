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


#include <message_tf_frame_transformer/MessageTfFrameTransformer.h>
#include <message_tf_frame_transformer/message_types.h>
#include <pluginlib/class_list_macros.h>
#include <ros/message_traits.h>


PLUGINLIB_EXPORT_CLASS(message_tf_frame_transformer::MessageTfFrameTransformer, nodelet::Nodelet)


namespace message_tf_frame_transformer {


const std::string MessageTfFrameTransformer::kInputTopic = "input";

const std::string MessageTfFrameTransformer::kOutputTopic = "transformed";

const std::string MessageTfFrameTransformer::kSourceFrameIdParam = "source_frame_id";

const std::string MessageTfFrameTransformer::kTargetFrameIdParam = "target_frame_id";


void MessageTfFrameTransformer::onInit() {

  node_handle_ = this->getMTNodeHandle();
  private_node_handle_ = this->getMTPrivateNodeHandle();

  loadParameters();
  setup();
}


void MessageTfFrameTransformer::loadParameters() {

  private_node_handle_.getParam(kSourceFrameIdParam, source_frame_id_);

  bool found = private_node_handle_.getParam(kTargetFrameIdParam, target_frame_id_);
  if (!found) {
    NODELET_FATAL("Parameter '%s' is required", kTargetFrameIdParam.c_str());
    exit(EXIT_FAILURE);
  }

  NODELET_INFO("Transforming data on topic '%s' to frame '%s' published on topic '%s'",
               ros::names::resolve("~" + kInputTopic).c_str(), target_frame_id_.c_str(),
               ros::names::resolve("~" + kOutputTopic).c_str());
}


void MessageTfFrameTransformer::setup() {

  // listen to tf
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  // setup subscriber to detect message type
  subscriber_ = private_node_handle_.subscribe(kInputTopic, 10, &MessageTfFrameTransformer::detectMessageType, this);
}


void MessageTfFrameTransformer::detectMessageType(const topic_tools::ShapeShifter::ConstPtr& generic_msg) {

  const std::string msg_type = generic_msg->getDataType();
  const std::string& msg_type_md5 = generic_msg->getMD5Sum();
  NODELET_DEBUG("Detected message type '%s'", msg_type.c_str());

  // detect message type based on md5 hash
  if (false) {}
#define MESSAGE_TYPE(TYPE)                                                     \
  else if (msg_type_md5 == ros::message_traits::MD5Sum<TYPE>::value()) {       \
                                                                               \
    /* instantiate generic message as message of concrete type */              \
    TYPE::ConstPtr msg = generic_msg->instantiate<TYPE>();                     \
                                                                               \
    /* setup publisher and pass message to transform callback */               \
    publisher_ = private_node_handle_.advertise<TYPE>(kOutputTopic, 10);       \
    this->transform<TYPE>(msg);                                                \
                                                                               \
    /* re-initialize concrete subscriber */                                    \
    subscriber_.shutdown();                                                    \
    subscriber_ = private_node_handle_.subscribe(                              \
      kInputTopic,                                                             \
      10,                                                                      \
      &MessageTfFrameTransformer::transform<TYPE>,                                      \
      this                                                                     \
    );                                                                         \
                                                                               \
  }
#include "message_tf_frame_transformer/message_types.macro"
#undef MESSAGE_TYPE
  else {
    NODELET_ERROR("Transforming message type '%s' is not supported",
                  msg_type.c_str());
    return;
  }
}


}  // namespace message_tf_frame_transformer
