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


#include <generic_transform/GenericTransform.ros2.hpp>
#include <generic_transform/message_types.ros2.hpp>


namespace generic_transform {


const std::string GenericTransform::kInputTopic = "~/input";

const std::string GenericTransform::kOutputTopic = "~/transformed";

const std::string GenericTransform::kFrameIdParam = "frame_id";


GenericTransform::GenericTransform() : Node("generic_transform") {

  loadParameters();
  setup();
}


void GenericTransform::loadParameters() {

  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.description = "Target frame ID to transform to";
  this->declare_parameter(kFrameIdParam, rclcpp::ParameterType::PARAMETER_STRING, param_desc);

  try {
    frame_id_ = this->get_parameter(kFrameIdParam).as_string();
  } catch (rclcpp::exceptions::ParameterUninitializedException&) {
    RCLCPP_FATAL(get_logger(), "Parameter '%s' is required", kFrameIdParam.c_str());
    exit(EXIT_FAILURE);
  }
}


void GenericTransform::setup() {

  // listen to tf
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // setup subscriber to detect message type
  subscriber_ = this->create_generic_subscription(
    kInputTopic,
    "sensor_msgs/msg/PointCloud2",
    10,
    std::bind(&GenericTransform::detectMessageType, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(),
    "Transforming data on topic '%s' to frame '%s'",
    subscriber_->get_topic_name(),
    frame_id_.c_str()
  );
}


void GenericTransform::detectMessageType(const std::shared_ptr<rclcpp::SerializedMessage>& serialized_msg) {

  // TODO: find way to determine message type

  msg_type_ = "sensor_msgs::msg::PointCloud2";

  // setup publisher
  if (msg_type_.empty()) {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(kOutputTopic, 10);
  }

  // instantiate generic message as message of concrete type
  sensor_msgs::msg::PointCloud2 msg;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
  serializer.deserialize_message(serialized_msg.get(), &msg);

  // pass message to transform callback
  this->transform<sensor_msgs::msg::PointCloud2>(msg);

  RCLCPP_DEBUG(this->get_logger(), "Detected message type '%s'", msg_type_.c_str());
}


}  // namespace generic_transform


int main(int argc, char *argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<generic_transform::GenericTransform>());
  rclcpp::shutdown();

  return 0;
}
