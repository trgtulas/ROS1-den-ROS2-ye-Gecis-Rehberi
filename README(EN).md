# ROS1 to ROS2 Migration Guide

This guide is designed to help you migrate your mobile robot system using ROS1 (Noetic) to ROS2 (Humble). This guide focuses on the mobile robot **LIMCOBOT**, which features LIDAR and a depth camera, and is based on Python and C++. Additionally, the robot will be simulated in the **Gazebo** environment, allowing for pre-installation testing.

## 📌 Target Audience
This guide is designed for users with **intermediate knowledge of ROS1** and provides a practical, example-based migration process.

## 📦 Content Headings

- 🔁 Key differences between ROS1 and ROS2 (Noetic → Humble)
- 🛠️ Creating a ROS2 package structure and workspace using `colcon`
- 🚀 `.launch` (XML) → Python-based launch file conversion
- ⚙️ Parameter management and dynamic configuration
- 🔄 TF and TF2 conversions
- 📡 Navigation, mapping, and MoveIt migration
- 🧪 Sample code with ROS1 and ROS2 versions
- 🧭 **Gazebo simulation**: Simulation structure with ROS1 vs. ROS2
- 🔗 Hybrid environment migration with `ros1_bridge`

## 🧰 Requirements

- Ubuntu 22.04 (Jammy)
- ROS1 Noetic (for comparison)
- ROS2 Humble
- VSCode (recommended)
- Tools like colcon, rosdep, vcs, etc.
- Gazebo (Classic and Ignition/GZ supported)

## 📁 Directory Structure

- `docs/`: Technical documentation and migration steps
- `examples/`: Sample code showing ROS1 and ROS2 versions
- `images/`: Diagrams and screenshots

## 🚀 Getting Started

1. Clone this repo.

2. Follow the steps in the `docs/` folder.

3. Test your own code migration with the examples.

## 🤝 Contributing

We welcome community contributions! Fork it, submit a pull request with improvements or fixes.

---

Let's start modernizing your ROS system together 🧠🤖

---

# Key Differences Between ROS1 (Noetic) and ROS2 (Humble)

In this section, we will examine in detail the key architectural differences between ROS1 and ROS2, the new features, and why switching to ROS2 is necessary.

---

## 1. Introduction: Why ROS2?

ROS1 has become a standard platform in robotics for many years. However, over time, the following shortcomings created the need for a more sustainable, secure, and modular infrastructure:

- **Lack of real-time support**
- **Lack of security measures**
- **Limited performance in multi-robot systems**
- **Lack of flexibility in distributed systems**

ROS2 was designed from the ground up to address these shortcomings. Specifically, thanks to its **DDS (Data Distribution Service)** infrastructure, it offers a more flexible, secure, and customizable communication structure.

---

## 2. Communication Infrastructure

| Feature | ROS1 | ROS2 (Humble) |
|--|-------------------------------------------|-----------------------------------|
| Protocol | TCPROS / UDPROS | DDS-based (FastDDS, CycloneDDS, etc.) |
| QoS Support | No | Yes (reliability, durability, history, etc.) |
| Multicast | No | Yes |
| Security | Requires external solutions | Integrates with SROS2 |
| Discovery | Manual or topic-based | Automatic discovery |

---

## 3. Workspace and Package Structure

The workspace structure has been streamlined in ROS2. The `colcon` tool enables parallel compilation and management of independent packages.

### ROS1 (catkin)
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```
### ROS2 (colcon)
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

---

## 4. 🛠️ Tools and Command Line Comparison

With ROS2, command line tools have been significantly restructured and made more modular by being divided into subcommands. This allows separate tools for each resource type (topic, service, param, bag, dll).

#### 🔄 General Command Comparison

| Process | ROS1 Command | ROS2 Command |
|---------------------------|----------------------------------------|
| Package listing | `rospack list` | `ros2 pkg list` |
| Finding package path | `rospack find <pkg>` | `ros2 pkg prefix <pkg>` |
| Running a node | `rosrun <pkg> <node>` | `ros2 run <pkg> <node>` |
| Running a launch file | `roslaunch <pkg> <file>` | `ros2 launch <pkg> <file>` |

#### 📡 Topic Actions

| Action | ROS1 | ROS2 |
|----------------------------|----------------------------|-----------------------------------------|
| Listing | `rostopic list` | `ros2 topic list` |
| Monitoring published data | `rostopic echo /topic` | `ros2 topic echo /topic` |
| Displaying information | `rostopic info /topic` | `ros2 topic info /topic` |
| Publish (manual) | `rostopic pub` | `ros2 topic pub` |
| Send test message | `rostopic pub -1` | `ros2 topic pub --once` |

#### 🧪 Service Operations

| Operation | ROS1 | ROS2 |
|---------------------------|------------------------------|----------------------------------------|
| Listing | `rosservice list` | `ros2 service list` |
| Displaying information | `rosservice info` | `ros2 service info` |
| Calling a service | `rosservice call` | `ros2 service call` |
| Querying the service type | `rosservice type` | `ros2 service type` |

### ⚙️ Parameter Operations

| Operation | ROS1 | ROS2 |
|----------------------------|----------------------------|----------------------------------------|
| Listing params | `rosparam list` | `ros2 param list` |
| Getting param values ​​| `rosparam get /param` | `ros2 param get <node> <param>` |
| Setting param values ​​| `rosparam set /param val` | `ros2 param set <node> <param> val` |
| Loading from param file | `rosparam load file.yaml` | Passing YAML with `ros2 launch` |

#### 💾 Bag Recording & Playback

| Process | ROS1 | ROS2 |
|----------------------------|----------------------------|-----------------------------------------|
| Starting recording | `rosbag record -a` | `ros2 bag record -a` |
| Playing recording | `rosbag play file.bag` | `ros2 bag play file` |
| View content | `rosbag info file.bag` | `ros2 bag info file` |

#### 🧩 Message and Service Types

| Process | ROS1 | ROS2 |
|---------------------------|------------------------------|----------------------------------------|
| List message types | `rosmsg list` | `ros2 interface list` |
| View message types | `rosmsg show <type>` | `ros2 interface show <type>` |
| List service types | `rossrv list` | `ros2 interface list` (same command) |
| Show service types | `rossrv show <type>` | `ros2 interface show <type>` |

---

## 5. 🚀 Node and Launch Management

There are significant differences between ROS1 and ROS2 in terms of node initialization and launch systems. ROS2 made node initialization more modular and programmable.

### 🚀 Launch Files

- **ROS1**: Works with XML files with the `.launch` extension.
- **ROS2**: Uses Python-based `.launch.py` files. This allows for more dynamic conditional operations, loops, and parameter management.

**Sample ROS1 launch file (`start_robot.launch`)**:
```xml
<launch> 
<node pkg="my_robot" type="robot_node.py" name="robot_node" output="screen" />
</launch>
```

**Same structure with Python on ROS2 (start_robot.launch.py):**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
return LaunchDescription([ 
Node( 
package='my_robot', 
executable='robot_node', 
name='robot_node', 
output='screen' 
) 
])
```


### 📦 Node Definition: Structural Differences

##### ROS1 Python Node (example)
```python
#!/usr/bin/env python
import rospy

def main(): 
rospy.init_node('simple_node') 
rospy.loginfo("Hello ROS1!")

if __name__ == '__main__': 
main()
```

##### ROS1 Python Node (example)
```python
import rclpy
from rclpy.node import Node

class SimpleNode(Node): 
def __init__(self): 
super().__init__('simple_node') 
self.get_logger().info("Hello ROS2!")

def main(): 
rclpy.init() 
node = SimpleNode() 
rclpy.spin(node) 
node.destroy_node()
rclpy.shutdown()

if __name__ == '__main__:
main()
```
#### 🧠 ROS1 and ROS2 Node Structure Comparison

In ROS1, the node structure is simple and function-based. A node is started with `rospy` and run with `spin()`. Modularity is low, but generally sufficient for small projects.

In ROS2, the node structure is object-oriented (OOP). Each node inherits from the `Node` class. This:
- Makes the code more readable and modular.
- Components such as parameters, publishers, and subscribers are organized within the class.
- Increases testability.
- Advanced features such as lifecycle, QoS, and callback management are integrated.

| Feature | ROS1 | ROS2 |
|--------------------------|------------------------------|---------------------------------|
| Structure | Function-based | Class-based (OOP) |
| Entry point | `rospy.init_node()` | `rclpy.init()` |
| Logger | `rospy.loginfo()` | `self.get_logger().info()` |
| Modularity | Low | High |
| Resource management | Automatic | `destroy_node()`, `shutdown()`|
| Advanced node features | None | Yes (Lifecycle, Component, etc.)|

ROS2 offers a more sustainable node structure for larger and more complex systems.

---

## 6. Advanced Features Unique to ROS2

ROS2 offers a much more robust infrastructure compared to ROS1, not only in terms of architecture but also in terms of advanced features. These features are designed specifically for industrial and large-scale applications.

---

### 🔄 Lifecycle Nodes

ROS2 introduces the **lifecycle node** structure to standardize node state management. In this structure, a node transitions between specific states in a controlled manner:

- `unconfigured`
- `inactive`
- `active`
- `finalized`

This allows:
- Nodes to be activated when the system is ready.
- Nodes can be deactivated and restarted in case of errors.
- System control becomes more secure and configurable.

---

### 🧩 Component Nodes

The **component node** feature allows multiple nodes to be run in the same process. This structure:
- Reduces memory usage
- Reduces startup time
- Enables dynamically adding nodes within the same application

It is particularly useful for **embedded systems** and **multi-module robotics software**.

---

### 📶 QoS (Quality of Service) Profiles

ROS2 offers **QoS profiles** for fine-tuning data communication. These profiles allow you to define different transmission policies for each topic or service.

For example:
- **reliability**: `reliable` (reliable) vs `best_effort` (may experience loss)
- **durability**: `volatile` (only if there are active subscribers) vs `transient_local` (previous data is retained)
- **history**: `keep_last`, `keep_all`

This allows you to define a specific communication style for each usage scenario.

---

### 🔐 SROS2: Secure ROS

ROS2 provides **secure communication** (Security ROS 2 - SROS2) using the DDS infrastructure. Features include:

- Data encryption
- Authentication
- Authorization

This structure is particularly important in critical areas such as networked robots, cloud integrations, and the defense industry.

---

Thanks to these advanced features of ROS2, it becomes possible to develop more modular, flexible, secure, and high-performance robot systems.

---

## 7. Parameter System and Dynamic Configuration

Using parameters in robot applications is critical for configuring node behavior and making runtime adjustments. ROS1 and ROS2 take quite different approaches in this regard.

---

### 📦 ROS1 Parameter Structure

In ROS1, parameters are stored on a central **parameter server**. This structure:
- Is shared and accessible by all nodes.
- Parameters are typically defined with the `rosparam` command or launch files.
- Parameters can be loaded from `.yaml` files.

**Example:**
```bash
rosparam set /robot_speed 1.0
rosparam get /robot_speed
```

```xml
<param name="robot_speed" value="1.0" />
<rosparam file="$(find my_pkg)/config/settings.yaml" />
```
However, in ROS1, parameter changes generally do not take effect until the node is restarted. The `dynamic_reconfigure` package is used for real-time configuration.

---
### ⚙️ ROS2 Parameter System

In ROS2, parameter management is performed on a per-node basis. Instead of a global parameter server, each node has its own parameter space.

**Parameters:**
- They are defined with `declare_parameter()` when creating the node.
- Can be read or updated at runtime with the `ros2 param` tool.
- YAML files are integrated into launch files.

**Example:**
```bash
ros2 param set /my_node robot_speed 1.0
ros2 param get /my_node robot_speed
```
YAML parameter transfer with a launch file:
```python
Node(
package='my_pkg',
executable='robot_node',
name='robot_node',
parameters=['config/settings.yaml']
)
```
---
### 🔄 Dynamically Managing Parameter Changes

In ROS1, parameters can be changed instantly via the GUI or terminal with the `dynamic_reconfigure` package. This is especially useful for runtime configurations such as PID tuning.

ROS2 lacks `dynamic_reconfigure`; instead, each node defines callback functions to listen for parameter updates:

```python
self.add_on_set_parameters_callback(self.param_callback)
```
With this method, parameters can be detected instantly and node behavior can be updated.

---

📊 Versus Interchangeable Property Table

| Property | ROS1 | ROS2 |
| ------------------------- | ------------------------------------- | ---------------------------------------- |
| Parameter field | Global parameter server | Node-specific parameters |
| YAML file integration | `<rosparam>` or `rosparam load` | `parameters` field in Python launch file |
| Runtime change | Restart may be required | Dynamically supported |
| Dynamic configuration | `dynamic_reconfigure` | `set_parameters_callback()` function |
| Param tool | `rosparam` | `ros2 param` |

ROS2's parameter structure is more secure, isolated, and modular. Nodes cannot directly access each other's parameters, reducing the risk of errors and preventing parameter confusion in multi-robot systems.

---

## 8. Navigation, SLAM, and MoveIt Migration

Basic functions such as localization, mapping, route planning, and robot arm control in mobile robot systems were provided in ROS1 by packages such as `move_base`, `gmapping`, `amcl`, and `moveit`. With ROS2, most of these packages have been completely rewritten and made more modular.

---

### 🚀 Navigation: `move_base` → `Navigation2 (nav2)`

**ROS1:**
`move_base` provides all navigation components in a single node. It is extensible but has a monolithic structure.

**ROS2:**
`nav2` (Navigation2) is a modular, node-based lifecycle system with behavior tree support. Each component is configured as an independent node.

| Feature | ROS1 (`move_base`) | ROS2 (`nav2`) |
|----------------------------|-----------------------------|----------------------------------|
| Structure | Single node, monolithic | Modular, lifecycle nodes |
| Path planner | Plugin-based | Plugin + behavior tree |
| Recovery behaviors | Static | Flexible with IT |
| Parameter management | Fixed structure | Dynamic lifecycle + YAML |
| TF2 integration | Partial | Full TF2 |

---

### 🧩 Key Components in Navigation2

- `nav2_amcl`: Localization (ROS1 `amcl` equivalent)
- `nav2_costmap_2d`: Obstacle mapping (ROS1 `costmap_2d`)
- `nav2_map_server`: Map loader and publisher (ROS1 `map_server`)
- `nav2_bt_navigator`: Behavior tree system for mission control
- `nav2_lifecycle_manager`: Manages all components with their lifecycle
- `nav2_smoother`: Path smoothing (usually required special plugins in ROS1)

---

### 🗺️ SLAM: `gmapping` → `slam_toolbox`

| Feature | ROS1 (`gmapping`) | ROS2 (`slam_toolbox`) |
|----------------------------|----------------------------|-----------------------------------|
| Real-time SLAM | Yes | Yes |
| Map editing | Limited | Dynamic |
| Service-assisted control | No | Yes (`pause`, `save_map`, etc.) |
| Performance | Low (single-core) | High (multi-core support) |

`slam_toolbox` for SLAM in ROS2 is equipped with advanced features such as online and offline mapping, map control with a service.

---

### 🤖 MoveIt: `moveit` → `moveit2`

| Feature | ROS1 (`moveit`) | ROS2 (`moveit2`) |
|------------------------------|------------------------------|------------------------------------|
| Planning framework | OMPL, plugin-based | Same |
| RViz integration | `rviz` | `rviz2` |
| Real-time control | Limited | More powerful with `moveit_servo` |
| ROS2 compatibility | None | Full compatibility + QoS support |
| Task scheduling | `moveit_task_constructor` | ROS2 version available |

---

### ➕ Other Important Migration Packages

| Purpose | ROS1 Package | ROS2 Counterpart |
|---------------------------|------------------------|-------------------------------------|
| Localization | `amcl` | `nav2_amcl` |
| Obstacle map | `costmap_2d` | `nav2_costmap_2d` |
| Map loader | `map_server` | `nav2_map_server` |
| Path smoothing | Usually custom solution | `nav2_smoother` |
| Task management | None | `nav2_bt_navigator` (BT-based) |
| Planning Visualization | `moveit_visual_tools` | `moveit_visual_tools` (compatible) |
| Servo Control | Restricted | `moveit_servo` |

---

### 📝 Migration Recommendations

- If you're using `move_base`, starting with `nav2_bringup` is a good step.
- For SLAM, `slam_toolbox` is more advanced in terms of both performance and ease of control.
- For MoveIt integrations, `moveit2` and `moveit_setup_assistant` are available in ROS2.
- Because the components are now lifecycle nodes, your initialization/management structure should change.
- Learning the Behavior Tree structure is critical to fully utilize the Navigation2 system.

---

ROS2's navigation, mapping, and arm control systems represent a transition to a more flexible, modular, and high-performance architecture. Correctly configuring these systems will help you unlock your robot's full potential.

---

## 9. Gazebo Simulation: ROS1 vs. ROS2

Gazebo is a powerful physics engine that allows testing robots in virtual environments. It can be integrated with both ROS1 and ROS2, but the integration structure and tools used have changed over time. With ROS2, **Gazebo Classic** (formerly Gazebo) as well as **Ignition (GZ) Gazebo** systems are now supported.

---

### 🏗️ General Architecture Changes

| Feature | ROS1 (Noetic) | ROS2 (Humble) |
|---------------------------|------------------------|-------------------------------------------|
| Integrated simulation tool | `gazebo_ros` | `gazebo_ros_pkgs`, `gz_ros2_control`, `ros_ign` |
| Supported Gazebo version | Gazebo Classic | Gazebo Classic + Ignition (GZ) |
| Control infrastructure | `ros_control` + `gazebo_ros_control` | `ros2_control` + `gz_ros2_control` |
| Robot files | `.urdf`, `.xacro` | Same, but more integrated with `ros2_control` |
| Sensor plugin structure | XML + `.gazebo` tags | Same logic, but compatible with the ROS2 API |

---

### ⚙️ Gazebo Simulation in ROS1

A typical simulation system in ROS1 includes the following components:
- `gazebo_ros` package
- `.world` files (environments)
- Robot defined with `.urdf` or `.xacro`
- Hardware interface with `ros_control`
- Sensor plugins (e.g.: `gazebo_ros_camera`, `gazebo_ros_laser`)

**Launch file example (ROS1):**
```xml
<launch>
<include file="$(find gazebo_ros)/launch/empty_world.launch"/>
<param name="robot_description" command="$(find xacro)/xacro $(find my_robot)/urdf/my_robot.urdf.xacro" />
<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model my_robot" />
</launch>
```

---

### ⚙️ Gazebo Simulation in ROS2

In ROS2, the structure has become more modular and standardized. `gazebo_ros_pkgs` has been ported for ROS2, and robot control is much more integrated thanks to the `gz_ros2_control` package.

The main simulation building blocks supported by ROS2 are:

- **gazebo_ros**: Basic Gazebo-ROS interface
- **ros2_control**: ROS2-based hardware interface
- **gz_ros2_control**: Provides a connection between Gazebo and ros2_control
- **ros_ign**: ROS interface for GZ (Ignition) simulation systems

### Launch file example (ROS2):
```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
return LaunchDescription([
IncludeLaunchDescription(
PythonLaunchDescriptionSource([
get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
),
])
```

### 🔌 Hardware and Sensor Integration

| Feature | ROS1 | ROS2 |
| -------------------------------- | --------------------------------- | -------------------------------------------- |
| LIDAR | `gazebo_ros_laser` plugin | Same XML format, made compatible for ROS2 |
| Camera | `gazebo_ros_camera` | `gazebo_ros_camera` (ROS2 port) |
| Hardware control | `ros_control + effort/joint` | `ros2_control + gz_ros2_control` |
| Plugin installation | `<gazebo>` tags in URDF | Works the same way |

### 🛠️ Migration Recommendations

- If you are using Gazebo Classic, the ROS1 architecture can be ported directly to ROS2.
- For new systems, using `ros2_control + gz_ros2_control` is more performant and maintainable.
- ROS2-compatible versions (with the same name) should be used for sensor plugins.
- The `xacro` and `robot_state_publisher` structures remain the same in ROS2, only the launch system has been switched to Python.

### 🎯 Summary

| Feature | ROS1 (Noetic) | ROS2 (Humble) |
| -------------------------- | ------------------------- | -------------------------------------------- |
| Simulation infrastructure | `gazebo_ros` | `gazebo_ros_pkgs`, `gz_ros2_control` |
| Control system | `ros_control` | `ros2_control` |
| Sensor plugins | Plugin-based | Same, ROS2-compatible versions |
| Launch format | XML (`.launch`) | Python (`.launch.py`) |
| Robot description | `.urdf`, `.xacro` | Same |
| GZ (Ignition) support | No | Yes (`ros_ign`, `gz_ros2_bridge`) |

In ROS2, the simulation system has not only been ported, but also made more flexible and powerful in terms of hardware control, parametric management, and launch infrastructure. Gazebo integration is still indispensable for providing a safe testing environment before the real robot.

---

## 10. Using TF and TF2

The TF (Transform) system is needed to accurately correlate sensor data, the positions of robot parts, and moving objects in robot systems. TF provides transformation between different coordinate systems (e.g., `base_link`, `laser`, `odom`, `map`). The structure of this system is different in ROS1 and ROS2.

---

### 🔄 ROS1: Mixed Use of `tf` and `tf2`

In ROS1, both `tf` and `tf2` libraries can be used:
- `tf` is the old system, simple but limited
- `tf2` is the more modern and recommended system
- Both systems have been used together for a long time

**Common usage:**
- `tf::TransformListener`, `tf::TransformBroadcaster` (`tf`)
- `tf2_ros::Buffer`, `tf2_ros::TransformListener` (`tf2`)

---

### 🔁 ROS2: Only `tf2`

With ROS2, the TF system is completely built on **tf2`**:
- `tf` is no longer supported
- All broadcast and lookup operations This is done via `tf2_ros`
- Static and dynamic transform publishers are lifecycle compatible.

---

### 📌 Broadcast and Listen Differences

| Process | ROS1 | ROS2 |
|------------------------|-------------------------------------|-------------------------------------|
| Static transform propagation | `static_transform_publisher` CLI or node | `ros2 run tf2_ros static_transform_publisher` |
| Dynamic transform propagation | `tf::TransformBroadcaster` | `tf2_ros.TransformBroadcaster` |
| Transform listening | `tf::TransformListener` | `tf2_ros.TransformListener` |
| TF2 support | Optional | Default and mandatory |
| Message type | `tf`/`tfMessage` | `geometry_msgs/msg/TransformStamped` |

---

### 🧪 Static Transform CLI Comparison

**ROS1:**
```bash
rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms
```

**ROS2:**
```bash
ros2 run tf2_ros static_transform_publisher x y z roll pitch yaw frame_id child_frame_id
```

- In ROS2, transformations are entered as roll, pitch, yaw instead of quaternion.

- The ROS2 command is simpler and works with automatic frequency.

---

🧩 TF2 Usage Examples (Code Logic)
ROS1:

- Listener:
- `tf::TransformListener listener;`

- Broadcaster:
- `tf::TransformBroadcaster br;`

ROS2:

- Listener:
- `tf_buffer = Buffer()`, `listener = TransformListener(buffer, node)`

- Publisher:
- `StaticTransformBroadcaster`, `TransformBroadcaster`

In ROS2, all these classes are included in the tf2_ros package and are implemented with QoS settings.

---

🗺️ RViz and TF2
- The TF visualization system in RViz (and 'rviz2') is the same in ROS1 and ROS2.

- To test that TF trees are being rendered correctly:
- `rosrun tf view_frames` → ROS1
- `ros2 run tf2_tools view_frames` → ROS2 (extracts as a PDF)

✅ Migration Recommendations
- All code containing `tf::` should be migrated to `tf2_ros`

- The transform message type should be `geometry_msgs/msg/TransformStamped`

- If you use `tf` in your ROS1 code, this will not work directly in ROS2.

- The ROS2 version of the CLI commands should be used for static transformations.

- Getting used to the `tf2_ros.Buffer` structure will provide a more robust structure in the long run.

In ROS2, the transform system is completely Being built on `tf2`, a more consistent, flexible, and DDS-compatible architecture is provided. A correct TF structure is essential for the reliable operation of all systems, such as navigation, SLAM, and robotic arms.

---

---

## 🔧 Additional Tools: Components That Can Help in the Migration Process

Directly migrating the entire system to ROS2 may not always be possible. If some components need to remain in ROS1 temporarily, the following tools can assist in this process.

---

### 🔗 `ros1_bridge`: Building a Bridge Between ROS1 and ROS2

`ros1_bridge` is a bridge layer that allows you to exchange messages and services between ROS1 and ROS2 systems. Workarounds or incremental It's very useful in migration scenarios.

**When to use it?**
- If some drivers or nodes haven't yet been ported to ROS2.
- If newly developed systems in ROS2 need to be tested with ROS1 data.

**Key Features:**
- Automatic bridge between messages defined in the same way in ROS1 and ROS2.
- Support for topics, services, and (limited) actions.
- Requires compilation from source code; custom messages may require additional configuration.

**Official project page:**
👉 https://github.com/ros2/ros1_bridge

---
