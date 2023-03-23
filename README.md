# generic_transform

<p align="center">
  <img src="https://img.shields.io/github/v/release/ika-rwth-aachen/generic_transform"/>
  <img src="https://img.shields.io/github/license/ika-rwth-aachen/generic_transform"/>
  <a href="https://github.com/ika-rwth-aachen/generic_transform/actions/workflows/build.yml"><img src="https://github.com/ika-rwth-aachen/generic_transform/actions/workflows/build.yml/badge.svg"/></a>
  <img src="https://img.shields.io/badge/ROS-noetic-blueviolet"/>
  <img src="https://img.shields.io/badge/ROS 2-humble|rolling-blueviolet"/>
  <a href="https://github.com/ika-rwth-aachen/generic_transform"><img src="https://img.shields.io/github/stars/ika-rwth-aachen/generic_transform?style=social"/></a>
</p>

The *generic_transform* package provides a ROS / ROS 2 node(let) to transform arbitrary ROS messages to a different coordinate frame.

- [Installation](#installation)
- [Usage](#usage)
- [Supported Message Types](#supported-message-types)
- [Nodes/Nodelets](#nodesnodelets)
- [Acknowledgements](#acknowledgements)


## Installation

> :warning: At the time of writing, the ROS package has not yet been released to the public package manager repositories yet. In case the installation fails, you have to install the package from source for now.

The *generic_transform* package is released as an official ROS / ROS 2 package and can easily be installed via a package manager.

```bash
sudo apt install ros-$ROS_DISTRO-generic-transform
```

If you would like to install *generic_transform* from source, simply clone this repository into your ROS workspace. All dependencies that are listed in the ROS [`package.xml`](./package.xml) can be installed using [*rosdep*](http://wiki.ros.org/rosdep).

```bash
# generic_transform$
rosdep install -r --ignore-src --from-paths .

# ROS 2
# workspace$
colcon build --packages-up-to generic_transform --cmake-args -DCMAKE_BUILD_TYPE=Release

# ROS
# workspace$
catkin build -DCMAKE_BUILD_TYPE=Release generic_transform
```


## Usage

In order to transform messages on topic `$INPUT_TOPIC` to frame `$TARGET_FRAME_ID` and publish them to topic `$OUTPUT_TOPIC`, the *generic_transform* node can be started with the following topic remappings and parameter setting. The `frame_id` parameter is required, while the topics otherwise default to `~/input` and `~/transformed` in the node's private namespace.

```bash
# ROS 2
ros2 run generic_transform generic_transform --ros-args \
  -r \~/input:=$INPUT_TOPIC \
  -r \~/transformed:=$OUTPUT_TOPIC \
  -p frame_id:=$TARGET_FRAME_ID

# ROS
rosrun generic_transform generic_transform \
  ~input:=$INPUT_TOPIC \
  ~transformed:=$OUTPUT_TOPIC \
  _frame_id:=$TARGET_FRAME_ID
```


## Supported Message Types

The *generic_transform* package is able to support any ROS message types that integrates with [`tf2::doTransform`](http://wiki.ros.org/tf2/Tutorials/Transforming%20your%20own%20datatypes). Currently the following message types are explicitly supported.

| ROS | ROS 2 |
| --- | --- |
| [`geometry_msgs/PointStamped`](https://docs.ros.org/en/api/geometry_msgs/html/msg/PointStamped.html) | [`geometry_msgs/msg/PointStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PointStamped.html) |
| [`geometry_msgs/PoseStamped`](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html) | [`geometry_msgs/msg/PoseStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html) |
| [`geometry_msgs/Vector3Stamped`](https://docs.ros.org/en/api/geometry_msgs/html/msg/Vector3Stamped.html) | [`geometry_msgs/msg/Vector3Stamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Vector3Stamped.html) |
| [`geometry_msgs/WrenchStamped`](https://docs.ros.org/en/api/geometry_msgs/html/msg/WrenchStamped.html) | [`geometry_msgs/msg/WrenchStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/WrenchStamped.html) |
| [`sensor_msgs/PointCloud2`](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) | [`sensor_msgs/msg/PointCloud2`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud.html) |

### Adding Support for a New Message Type

Through application of preprocessor macros, adding support for a new ROS message type is as easy as adding only two lines of code. Note that the ROS message types have to integrate with [`tf2::doTransform`](http://wiki.ros.org/tf2/Tutorials/Transforming%20your%20own%20datatypes). Feel free to open a pull request to add support for more message types!

1. [`message_types.h`](./include/generic_transform/message_types.h) (ROS) / [`message_types.ros2.hpp`](./include/generic_transform/message_types.ros2.hpp) (ROS 2)
   - include required message headers
1. [`message_types.macro`](./include/generic_transform/message_types.h) (ROS) / [`message_types.ros2.macro`](./include/generic_transform/message_types.ros2.hpp) (ROS 2)
   - define information about the new message type by calling the `MESSAGE_TYPE` macro
      - `TYPE`: ROS message type (e.g. `geometry_msgs::msg::PointStamped`)
      - `NAME`: *(ROS 2 only)* ROS message type name (e.g. `geometry_msgs/msg/PointStamped`)


## Nodes/Nodelets

##### ROS 2

| Package | Node | Description |
| --- | --- | --- |
| `generic_transform` | `generic_transform` | transform arbitrary ROS messages to a different coordinate frame |

##### ROS

| Package | Node | Nodelet | Description |
| --- | --- | --- | --- |
| `generic_transform` | `generic_transform` | `GenericTransform` | transform arbitrary ROS messages to a different coordinate frame |

### generic_transform/generic_transform

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
| `~/frame_id` | `string` | target frame ID |


## Acknowledgements

This research is accomplished within the projects [6GEM](https://6gem.de/) (FKZ 16KISK036K). We acknowledge the financial support for the projects by the Federal Ministry of Education and Research of Germany (BMBF).
