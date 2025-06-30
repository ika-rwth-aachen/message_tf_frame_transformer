# message_tf_frame_transformer

<p align="center">
  <img src="https://img.shields.io/github/v/release/ika-rwth-aachen/message_tf_frame_transformer"/>
  <img src="https://img.shields.io/github/license/ika-rwth-aachen/message_tf_frame_transformer"/>
  <a href="https://github.com/ika-rwth-aachen/message_tf_frame_transformer/actions/workflows/industrial_ci.yml"><img src="https://github.com/ika-rwth-aachen/message_tf_frame_transformer/actions/workflows/industrial_ci.yml/badge.svg"/></a>
  <a href="https://github.com/ika-rwth-aachen/message_tf_frame_transformer/actions/workflows/docker-ros.yml"><img src="https://github.com/ika-rwth-aachen/message_tf_frame_transformer/actions/workflows/docker-ros.yml/badge.svg"/></a>
  <img src="https://img.shields.io/badge/ROS-humble|jazzy|kilted|rolling-293754"/>
  <a href="https://github.com/ika-rwth-aachen/message_tf_frame_transformer"><img src="https://img.shields.io/github/stars/ika-rwth-aachen/message_tf_frame_transformer?style=social"/></a>
</p>

The *message_tf_frame_transformer* package provides a ROS node to transform messages of arbitrary type to a different coordinate frame. This can be helpful if you cannot or do not want to modify the source code of other ROS nodes that require your data to be valid in a specific coordinate frame. Simply launch the *message_tf_frame_transformer* node and transform arbitrary messages to a target coordinate frame.

- [Installation](#installation)
  - [docker-ros](#docker-ros)
- [Usage](#usage)
- [Supported Message Types](#supported-message-types)
- [Nodes](#nodes)
- [Acknowledgements](#acknowledgements)


## Installation

The *message_tf_frame_transformer* package is released as an official ROS package and can easily be installed via a package manager.

```bash
sudo apt install ros-$ROS_DISTRO-message-tf-frame-transformer
```

If you would like to install *message_tf_frame_transformer* from source, simply clone this repository into your ROS workspace. All dependencies that are listed in the ROS [`package.xml`](./package.xml) can be installed using [*rosdep*](http://wiki.ros.org/rosdep).

```bash
# message_tf_frame_transformer$
rosdep install -r --ignore-src --from-paths .

# workspace$
colcon build --packages-up-to message_tf_frame_transformer --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### docker-ros

*message_tf_frame_transformer* is also available as a Docker image, containerized through [*docker-ros*](https://github.com/ika-rwth-aachen/docker-ros).

```bash
docker run --rm ghcr.io/ika-rwth-aachen/message_tf_frame_transformer:latest # or distro-specific tags, e.g., :rolling
```


## Usage

In order to transform messages on topic `$INPUT_TOPIC` to frame `$TARGET_FRAME_ID` and publish them to topic `$OUTPUT_TOPIC`, the *message_tf_frame_transformer* node can be started with the following topic remappings and parameter setting. Only the `target_frame_id` parameter is required. The `source_frame_id` parameter is only required for non-stamped messages without an [`std_msgs/Header`](https://docs.ros.org/en/api/std_msgs/html/msg/Header.html). The topics default to `~/input` and `~/transformed` in the node's private namespace.

```bash
ros2 run message_tf_frame_transformer message_tf_frame_transformer --ros-args \
  -r \~/input:=$INPUT_TOPIC \
  -r \~/transformed:=$OUTPUT_TOPIC \
  -p source_frame_id:=$SOURCE_FRAME_ID \
  -p target_frame_id:=$TARGET_FRAME_ID
```

The provided launch file enables you to directly launch a [`tf2_ros/static_transform_publisher`](http://wiki.ros.org/tf2_ros) alongside the *message_tf_frame_transformer* node. This way you can transform a topic to a new coordinate frame with a single command.

```bash
ros2 launch message_tf_frame_transformer message_tf_frame_transformer.launch.xml \
  input_topic:=$INPUT_TOPIC \
  output_topic:=$OUTPUT_TOPIC \
  source_frame_id:=$SOURCE_FRAME_ID \
  target_frame_id:=$TARGET_FRAME_ID \
  x:=$X \
  y:=$Y \
  z:=$Z \
  roll:=$ROLL \
  pitch:=$PITCH \
  yaw:=$YAW
```


## Supported Message Types

The *message_tf_frame_transformer* package is able to support any ROS message type that integrates with [`tf2::doTransform`](http://wiki.ros.org/tf2/Tutorials/Transforming%20your%20own%20datatypes). Currently, the following message types are explicitly supported.

| ROS Message | Remarks |
| --- | --- |
| [`geometry_msgs/msg/Point`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Point.html) |  |
| [`geometry_msgs/msg/PointStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PointStamped.html) |  |
| [`geometry_msgs/msg/Pose`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Pose.html) |  |
| [`geometry_msgs/msg/PoseStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html) |  |
| [`geometry_msgs/msg/PoseWithCovariance`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseWithCovariance.html) |  |
| [`geometry_msgs/msg/PoseWithCovarianceStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseWithCovarianceStamped.html) |  |
| [`geometry_msgs/msg/Quaternion`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Quaternion.html) |  |
| [`geometry_msgs/msg/QuaternionStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/QuaternionStamped.html) |  |
| [`geometry_msgs/msg/Transform`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Transform.html) |  |
| [`geometry_msgs/msg/TransformStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/TransformStamped.html) |  |
| [`geometry_msgs/msg/Vector3`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Vector3.html) |  |
| [`geometry_msgs/msg/Vector3Stamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Vector3Stamped.html) |  |
| [`geometry_msgs/msg/Wrench`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Wrench.html) |  |
| [`geometry_msgs/msg/WrenchStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/WrenchStamped.html) |  |
| [`perception_msgs/msg/EgoData`](https://github.com/ika-rwth-aachen/perception_interfaces/blob/main/perception_msgs/msg/EgoData.msg) |  |
| [`perception_msgs/msg/Object`](https://github.com/ika-rwth-aachen/perception_interfaces/blob/main/perception_msgs/msg/Object.msg) |  |
| [`perception_msgs/msg/ObjectList`](https://github.com/ika-rwth-aachen/perception_interfaces/blob/main/perception_msgs/msg/ObjectList.msg) |  |
| [`perception_msgs/msg/ObjectState`](https://github.com/ika-rwth-aachen/perception_interfaces/blob/main/perception_msgs/msg/ObjectState.msg) |  |
| [`route_planning_msgs/msg/LaneBoundary`](https://github.com/ika-rwth-aachen/planning_interfaces/blob/main/route_planning_msgs/msg/LaneBoundary.msg) |  |
| [`route_planning_msgs/msg/LaneElement`](https://github.com/ika-rwth-aachen/planning_interfaces/blob/main/route_planning_msgs/msg/LaneElement.msg) |  |
| [`route_planning_msgs/msg/RegulatoryElement`](https://github.com/ika-rwth-aachen/planning_interfaces/blob/main/route_planning_msgs/msg/RegulatoryElement.msg) |  |
| [`route_planning_msgs/msg/Route`](https://github.com/ika-rwth-aachen/planning_interfaces/blob/main/route_planning_msgs/msg/Route.msg) |  |
| [`route_planning_msgs/msg/RouteElement`](https://github.com/ika-rwth-aachen/planning_interfaces/blob/main/route_planning_msgs/msg/RouteElement.msg) |  |
| [`sensor_msgs/msg/PointCloud2`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud.html) | Only the first three point cloud channels (usually `xyz`) are transformed. |
| [`trajectory_planning_msgs/msg/Trajectory`](https://github.com/ika-rwth-aachen/planning_interfaces/blob/main/trajectory_planning_msgs/msg/Trajectory.msg) |  |

### Adding Support for a New Message Type

Through application of preprocessor macros, adding support for a new ROS message type is as easy as adding only two lines of code. Note that the ROS message types have to integrate with [`tf2::doTransform`](http://wiki.ros.org/tf2/Tutorials/Transforming%20your%20own%20datatypes). Feel free to open a pull request to add support for more message types!

1. [`message_types.hpp`](./include/message_tf_frame_transformer/message_types.hpp)
   - include required message headers
1. [`message_types.macro`](./include/message_tf_frame_transformer/message_types.macro)
   - define information about the new message type by calling the `MESSAGE_TYPE` macro
      - `TYPE`: ROS message type (e.g. `geometry_msgs::msg::PointStamped`)
      - `NAME`: ROS message type name (e.g. `geometry_msgs/msg/PointStamped`)


## Nodes

| Package | Node | Description |
| --- | --- | --- |
| `message_tf_frame_transformer` | `message_tf_frame_transformer` | transform arbitrary ROS messages to a different coordinate frame |

### message_tf_frame_transformer/message_tf_frame_transformer

#### Subscribed Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/input` | see [Supported Message Types](#supported-message-types) | message to transform |

#### Published Topics

| Topic | Type | Description |
| --- | --- | --- |
| `~/transformed` | see [Supported Message Types](#supported-message-types) | transformed message |

#### Services

\-

#### Actions

\-

#### Parameters

| Parameter | Type | Description |
| --- | --- | --- |
| `~/target_frame_id` | `string` | target frame ID |
| `~/source_frame_id` | `string` | source frame ID (optional; if message has no [`std_msgs/Header`](https://docs.ros.org/en/api/std_msgs/html/msg/Header.html)) |


## Acknowledgements

This research is accomplished within the project [6GEM](https://6gem.de/) (FKZ 16KISK036K). We acknowledge the financial support for the project by the Federal Ministry of Education and Research of Germany (BMBF).

