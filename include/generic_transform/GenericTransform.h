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

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <topic_tools/shape_shifter.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>


namespace generic_transform {

class GenericTransform : public nodelet::Nodelet {

 protected:

  virtual void onInit() override;

  void loadParameters();

  void setup();

  void transform(const sensor_msgs::PointCloud2::ConstPtr& msg);

 protected:

  static const std::string kInputTopic;

  static const std::string kOutputTopic;

  static const std::string kFrameIdParam;

  std::string frame_id_;

  ros::NodeHandle private_node_handle_;

  tf2_ros::Buffer tf_buffer_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ros::Subscriber subscriber_;

  ros::Publisher publisher_;
};

}  // namespace generic_transform
