# generic_transform

This package provides a ROS / ROS2 node to transform arbitrary ROS messages to a different coordinate frame.

- [Supported Message Types](#supported-message-types)
- [Nodes](#nodes)
  - [generic_transform/generic_transform](#generic_transformgeneric_transform)


## Supported Message Types

| ROS | ROS 2 |
| --- | --- |
| [`geometry_msgs/PointStamped`](https://docs.ros.org/en/api/geometry_msgs/html/msg/PointStamped.html) | [`geometry_msgs/msg/PointStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PointStamped.html) |
| [`geometry_msgs/PoseStamped`](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html) | [`geometry_msgs/msg/PoseStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html) |
| [`geometry_msgs/Vector3Stamped`](https://docs.ros.org/en/api/geometry_msgs/html/msg/Vector3Stamped.html) | [`geometry_msgs/msg/Vector3Stamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Vector3Stamped.html) |
| [`geometry_msgs/WrenchStamped`](https://docs.ros.org/en/api/geometry_msgs/html/msg/WrenchStamped.html) | [`geometry_msgs/msg/WrenchStamped`](https://docs.ros2.org/foxy/api/geometry_msgs/msg/WrenchStamped.html) |
| [`sensor_msgs/PointCloud2`](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html) | [`sensor_msgs/msg/PointCloud2`](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud.html) |


## Nodes

##### ROS 2

| Package | Node | Description |
| --- | --- | --- |
| `generic_transform` | `generic_transform` | transform arbitrary ROS messages to a different coordinate frame |

##### ROS

| Package | Nodelet | Description |
| --- | --- | --- |
| `generic_transform` | `GenericTransform` | transform arbitrary ROS messages to a different coordinate frame |

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
