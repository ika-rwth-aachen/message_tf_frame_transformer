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


#include <generic_transform/GenericTransform.h>
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/message_traits.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


PLUGINLIB_EXPORT_CLASS(generic_transform::GenericTransform, nodelet::Nodelet)


namespace generic_transform {


const std::string GenericTransform::kInputTopic = "input";

const std::string GenericTransform::kOutputTopic = "transformed";

const std::string GenericTransform::kFrameIdParam = "frame_id";


void GenericTransform::onInit() {

  private_node_handle_ = this->getMTPrivateNodeHandle();

  loadParameters();
  setup();
}


void GenericTransform::loadParameters() {

  bool found = private_node_handle_.getParam(kFrameIdParam, frame_id_);
  if (!found) NODELET_ERROR("Parameter '%s' is missing", kFrameIdParam.c_str());

  NODELET_INFO("Transforming data on topic '%s' to frame '%s' published on topic '%s'",
               ros::names::resolve("~" + kInputTopic).c_str(), frame_id_.c_str(),
               ros::names::resolve("~" + kOutputTopic).c_str());
}


void GenericTransform::setup() {

  // listen to tf
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

  // setup subscriber to detect message type
  subscriber_ = private_node_handle_.subscribe(kInputTopic, 10, &GenericTransform::detectMessageType, this);
}


void GenericTransform::detectMessageType(const topic_tools::ShapeShifter::ConstPtr& generic_msg) {

  std::string msg_type;
  const std::string& msg_type_md5 = generic_msg->getMD5Sum();

  // detect message type based on md5 hash
  if (msg_type_md5 == ros::message_traits::MD5Sum<sensor_msgs::Image>::value()) {

    // instantiate generic message as message of concrete type
    msg_type = "sensor_msgs::PointCloud2";
    sensor_msgs::Image::ConstPtr msg = generic_msg->instantiate<sensor_msgs::Image>();

    // setup publisher and pass message to transform callback
    publisher_ = private_node_handle_.advertise<sensor_msgs::Image>(kOutputTopic, 10);
    this->transform<sensor_msgs::Image>(msg);

    // re-initialize concrete subscriber
    subscriber_.shutdown();
    subscriber_ = private_node_handle_.subscribe(kInputTopic, 10, &GenericTransform::transform<sensor_msgs::PointCloud2>, this);

  } else if (msg_type_md5 == ros::message_traits::MD5Sum<sensor_msgs::PointCloud2>::value()) {
    
    // instantiate generic message as message of concrete type
    msg_type = "sensor_msgs::Image";
    sensor_msgs::PointCloud2::ConstPtr msg = generic_msg->instantiate<sensor_msgs::PointCloud2>();

    // setup publisher and pass message to transform callback
    publisher_ = private_node_handle_.advertise<sensor_msgs::PointCloud2>(kOutputTopic, 10);
    this->transform<sensor_msgs::PointCloud2>(msg);

    // re-initialize concrete subscriber
    subscriber_.shutdown();
    subscriber_ = private_node_handle_.subscribe(kInputTopic, 10, &GenericTransform::transform<sensor_msgs::PointCloud2>, this);
  }

  NODELET_DEBUG("Detected message type '%s'", msg_type.c_str());
}


}  // namespace generic_transform
