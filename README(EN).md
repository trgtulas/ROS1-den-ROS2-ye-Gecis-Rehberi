ROS1 to ROS2 Migration Guide

This guide was prepared to help you migrate your mobile robot system using ROS1 (Noetic) to ROS2 (Humble). Our guide focuses on a mobile robot called LIMCOBOT, which has a LIDAR and a depth camera, and is based on Python and C++ languages. In addition, the robot's simulation will be carried out in the Gazebo environment to perform pre-hardware tests.

📌 Target Audience

This guide is prepared for users with intermediate knowledge of ROS1 and offers a practical, example-based migration process.

📦 Table of Contents

🔁 Key differences between ROS1 and ROS2 (Noetic → Humble)

🛠️ Creating ROS2 package structure and workspace using colcon

🚀 .launch (XML) → Python-based launch file conversion

⚙️ Parameter management and dynamic configuration

🔄 TF and TF2 transformations

📡 Navigation, mapping, and MoveIt migration

🧪 Example codes with ROS1 and ROS2 versions

🧭 Gazebo simulation: ROS1 vs ROS2 simulation structure

🔗 Hybrid environment migration with ros1_bridge

🧰 Requirements

Ubuntu 22.04 (Jammy)

ROS1 Noetic (for comparison)

ROS2 Humble

VSCode (recommended)

colcon, rosdep, vcs, etc. tools

Gazebo (Classic and Ignition/GZ supported)

📁 Directory Structure

docs/: Technical documentation and migration steps

examples/: Example codes showing ROS1 and ROS2 versions

images/: Diagrams and screenshots

🚀 Getting Started

Clone this repository.

Follow the steps in the docs/ folder.

Test your code migration with examples.

🤝 Contributing

We welcome community contributions! Fork, send pull requests with enhancements or fixes.

Let's modernize your ROS system together 🧠🤖

Key Differences Between ROS1 (Noetic) and ROS2 (Humble)

In this section, we will examine in detail the fundamental architectural differences between ROS1 and ROS2, new features, and why a transition to ROS2 should be made.

1. Introduction: Why ROS2?

ROS1 has become a standard platform in robotics for many years. However, over time, due to the following shortcomings, a more sustainable, secure, and modular infrastructure was needed:

Lack of real-time support

Absence of security measures

Limited performance in multi-robot systems

Lack of flexibility in distributed systems

ROS2 was designed from scratch to address these shortcomings. Especially with its DDS (Data Distribution Service) infrastructure, it offers a more flexible, secure, and customizable communication structure.

2. Communication Infrastructure
Feature	ROS1	ROS2 (Humble)
Protocol	TCPROS / UDPROS	DDS-based (FastDDS, CycloneDDS, etc.)
QoS Support	No	Yes (reliability, durability, history vs.)
Multicast	No	Yes
Security	External solutions required	Integrated with SROS2
Discovery	Manual or topic-based	Automatic discovery
3. Workspace and Package Structure

The workspace structure in ROS2 has been modernized. The colcon tool enables parallel compilation and management of independent packages.

ROS1 (catkin)
code
Bash
download
content_copy
expand_less

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
ROS2 (colcon)
code
Bash
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
4. 🛠️ Tools and Command Line Comparison

With ROS2, command-line tools have been largely restructured and divided into subcommands, making them more modular. This allows separate tools to be used for each resource type (topic, service, param, bag, dll.).

🔄 General Command Comparison
Operation	ROS1 Command	ROS2 Command
List packages	rospack list	ros2 pkg list
Find package path	rospack find <pkg>	ros2 pkg prefix <pkg>
Run node	rosrun <pkg> <node>	ros2 run <pkg> <node>
Run launch file	roslaunch <pkg> <file>	ros2 launch <pkg> <file>
📡 Topic Operations
Operation	ROS1	ROS2
List	rostopic list	ros2 topic list
Monitor published data	rostopic echo /topic	ros2 topic echo /topic
Display information	rostopic info /topic	ros2 topic info /topic
Publish (manual)	rostopic pub	ros2 topic pub
Send test message	rostopic pub -1	ros2 topic pub --once
🧪 Service Operations
Operation	ROS1	ROS2
List	rosservice list	ros2 service list
Display information	rosservice info	ros2 service info
Call service	rosservice call	ros2 service call
Query service type	rosservice type	ros2 service type
⚙️ Parameter Operations
Operation	ROS1	ROS2
List parameters	rosparam list	ros2 param list
Get parameter value	rosparam get /param	ros2 param get <node> <param>
Set parameter value	rosparam set /param val	ros2 param set <node> <param> val
Load from parameter file	rosparam load file.yaml	ros2 launch passes YAML
💾 Bag Recording & Playback
Operation	ROS1	ROS2
Start recording	rosbag record -a	ros2 bag record -a
Play recording	rosbag play file.bag	ros2 bag play file
Display content	rosbag info file.bag	ros2 bag info file
🧩 Message and Service Types
Operation	ROS1	ROS2
List message types	rosmsg list	ros2 interface list
Inspect message type	rosmsg show <type>	ros2 interface show <type>
List service types	rossrv list	ros2 interface list (same command)
Show service type	rossrv show <type>	ros2 interface show <type>
5. 🚀 Node and Launch Management

There are significant differences in node startup and launch systems between ROS1 and ROS2. ROS2 has made node startup more modular and programmable.

🚀 Launch Files

ROS1: Works with XML files with the .launch extension.

ROS2: Python-based .launch.py files are used. This makes conditional operations, loops, and parameter management more dynamic.

Example ROS1 launch file (start_robot.launch):

code
Xml
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
<launch>
  <node pkg="my_robot" type="robot_node.py" name="robot_node" output="screen" />
</launch>

Same structure in ROS2 with Python (start_robot.launch.py):

code
Python
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
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
📦 Node Definition: Structural Differences
code
Python
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
#!/usr/bin/env python
import rospy

def main():
    rospy.init_node('simple_node')
    rospy.loginfo("Hello ROS1!")

if __name__ == '__main__':
    main()
code
Python
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
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

if __name__ == '__main__':
    main()
🧠 ROS1 and ROS2 Node Structure Comparison

In ROS1, the node structure is simple and function-based. A node is started with rospy and run with spin(). Modularity is low, generally sufficient for small projects.

In ROS2, the node structure is object-oriented (OOP). Each node inherits from the Node class. This allows for:

More readable and modular code.

Components like parameters, publishers, and subscribers are organized within the class.

Increased testability.

Integration of advanced features such as Lifecycle, QoS, and callback management.

Feature	ROS1	ROS2
Structure	Function-based	Class-based (OOP)
Entry point	rospy.init_node()	rclpy.init()
Logger	rospy.loginfo()	self.get_logger().info()
Modularity	Low	High
Resource management	Automatic	destroy_node(), shutdown()
Advanced node features	No	Yes (Lifecycle, Component, etc.)

ROS2 offers a more sustainable node structure for larger and more complex systems.

6. ROS2-Specific Advanced Features

ROS2 offers a much more powerful infrastructure compared to ROS1, not only architecturally but also with its advanced features. These features are designed especially for industrial and large-scale applications.

🔄 Lifecycle Nodes

ROS2 introduces the lifecycle node structure to standardize the state management of nodes. In this structure, a node transitions between specific states in a controlled manner:

unconfigured

inactive

active

finalized

This allows for:

Nodes are activated when the system is ready.

Nodes can be de-activated and restarted in error states.

System control becomes more secure and configurable.

🧩 Component Nodes

The component node feature makes it possible to run multiple nodes within the same process. This structure:

Reduces memory usage

Shortens startup time

Enables dynamic addition of nodes within the same application

It is particularly useful for embedded systems and multi-module robotic software.

📶 QoS (Quality of Service) Profiles

ROS2 offers QoS profiles for fine-tuning data communication. These profiles allow you to define different transmission policies for each topic or service.

For example:

reliability: reliable vs best_effort (can be lost)

durability: volatile (only if active subscriber) vs transient_local (previous data is retained)

history: keep_last, keep_all

This allows a communication method specific to each use case to be defined.

🔐 SROS2: Secure ROS

ROS2, using the DDS infrastructure, provides secure communication (Security ROS 2 - SROS2). Features include:

Data encryption

Authentication

Authorization

This structure is of great importance, especially in critical areas such as robots operating over networks, cloud integrations, and the defense industry.

Thanks to these advanced features of ROS2, it becomes possible to develop more modular, flexible, secure, and high-performance robot systems.

7. Parameter System and Dynamic Configuration

Parameter usage in robot applications is critical for configuring node behavior and making adjustments at runtime. ROS1 and ROS2 adopt quite different approaches in this regard.

📦 ROS1 Parameter Structure

In ROS1, parameters are stored on a central parameter server. This structure:

Is commonly accessible by all nodes.

Parameters are typically defined using the rosparam command or launch files.

Parameters can be loaded from .yaml files.

Example:

code
Bash
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
rosparam set /robot_speed 1.0
rosparam get /robot_speed
code
Xml
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
<param name="robot_speed" value="1.0" />
<rosparam file="$(find my_pkg)/config/settings.yaml" />

However, in ROS1, parameter changes generally do not take effect without restarting the node. The dynamic_reconfigure package is used for real-time configuration.

⚙️ ROS2 Parameter System

In ROS2, parameter management is done separately for each node. Instead of a global parameter server, each node has its own parameter space.

Parameters:

Defined with declare_parameter() when the node is created.

Can be read or updated at runtime with the ros2 param tool.

YAML files are integrated into launch files.

Example:

code
Bash
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
ros2 param set /my_node robot_speed 1.0
ros2 param get /my_node robot_speed

YAML parameter passing with launch file:

code
Python
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
Node(
    package='my_pkg',
    executable='robot_node',
    name='robot_node',
    parameters=['config/settings.yaml']
)
🔄 Dynamically Managing Parameter Changes

In ROS1, parameters can be changed instantly via GUI or terminal using the dynamic_reconfigure package. This is useful especially for runtime configurations like PID tuning.

In ROS2, dynamic_reconfigure is not present; instead, each node defines callback functions to listen for parameter updates within itself:

code
Python
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
self.add_on_set_parameters_callback(self.param_callback)

With this method, parameters can be detected instantly and node behavior can be updated.

📊 Comparative Features Table

Feature	ROS1	ROS2
Parameter space	Global parameter server	Node-specific parameters
YAML file integration	<rosparam> or rosparam load	parameters field in Python launch file
Runtime change	May require restart	Dynamically supported
Dynamic configuration	dynamic_reconfigure	set_parameters_callback() function
Parameter tool	rosparam	ros2 param

ROS2's parameter structure is more secure, isolated, and modular. Nodes cannot directly access each other's parameters, which reduces the risk of errors and prevents parameter confusion in multi-robot systems.

8. Navigation, SLAM and MoveIt Migration

In mobile robot systems, fundamental functions such as localization, mapping, path planning, and robot arm control were provided by packages like move_base, gmapping, amcl, moveit in ROS1. With ROS2, most of these packages have been completely rewritten and made more modular.

🚀 Navigation: move_base → Navigation2 (nav2)

ROS1:
move_base provides all navigation components within a single node. It is extensible but has a monolithic structure.

ROS2:
nav2 (Navigation2) is a modular, lifecycle node-based, and behavior tree-supported system. Each component is structured as an independent node.

Feature	ROS1 (move_base)	ROS2 (nav2)
Structure	Single node, monolithic	Modular, lifecycle nodes
Path planner	Plugin-based	Plugin + behavior tree
Recovery behaviors	Static	Flexible with BT
Parameter management	Fixed structure	Dynamic lifecycle + YAML
TF2 integration	Partial	Full TF2
🧩 Important Components in Navigation2

nav2_amcl: Localization (ROS1 amcl counterpart)

nav2_costmap_2d: Obstacle mapping (ROS1 costmap_2d)

nav2_map_server: Map loader and publisher (ROS1 map_server)

nav2_bt_navigator: Behavior tree system for task control

nav2_lifecycle_manager: Manages all components with lifecycle

nav2_smoother: Path smoothing (ROS1 usually required custom plugins)

🗺️ SLAM: gmapping → slam_toolbox
Feature	ROS1 (gmapping)	ROS2 (slam_toolbox)
Real-time SLAM	Yes	Yes
Map editing	Limited	Dynamic
Service-supported control	No	Yes (pause, save_map, etc.)
Performance	Low (single-core)	High (multi-core support)

For SLAM in ROS2, slam_toolbox is equipped with advanced features such as online and offline mapping, and map control via services.

🤖 MoveIt: moveit → moveit2
Feature	ROS1 (moveit)	ROS2 (moveit2)
Planning infrastructure	OMPL, plugin-based	Same
RViz integration	rviz	rviz2
Real-time control	Limited	More powerful with moveit_servo
ROS2 compatibility	No	Full compatibility + QoS support
Task planning	moveit_task_constructor	ROS2 version available
➕ Other Important Migration Packages
Purpose	ROS1 Package	ROS2 Counterpart
Localization	amcl	nav2_amcl
Obstacle map	costmap_2d	nav2_costmap_2d
Map loader	map_server	nav2_map_server
Path smoothing	Generally custom solution	nav2_smoother
Task management	No	nav2_bt_navigator (BT-based)
Planning visualization	moveit_visual_tools	moveit_visual_tools (compatible)
Servo control	Limited	moveit_servo
📝 Migration Recommendations

If you are using move_base, starting with nav2_bringup is a good step.

For SLAM, slam_toolbox is more advanced in terms of both performance and ease of control.

ROS2 versions of moveit2 and moveit_setup_assistant are available for MoveIt integrations.

Since components are now lifecycle nodes, your startup/management structure should change.

Learning the Behavior Tree structure is critical to fully utilize the Navigation2 system.

ROS2's navigation, mapping, and arm control systems mean a transition to a more flexible, modular, and high-performance structure. Configuring these systems correctly will help you unlock your robot's full potential.

9. Gazebo Simulation: ROS1 vs ROS2

Gazebo is a powerful physics engine that allows robots to be tested in virtual environments. It can work integrated with both ROS1 and ROS2, but the integration structure and tools used have changed over time. With ROS2, Gazebo Classic (formerly Gazebo) as well as Ignition (GZ) Gazebo systems have started to be supported.

🏗️ Changes in General Architecture
Feature	ROS1 (Noetic)	ROS2 (Humble)
Integrated simulation tool	gazebo_ros	gazebo_ros_pkgs, gz_ros2_control, ros_ign
Supported Gazebo version	Gazebo Classic	Gazebo Classic + Ignition (GZ)
Control infrastructure	ros_control + gazebo_ros_control	ros2_control + gz_ros2_control
Robot files	.urdf, .xacro	Same, but more integrated with ros2_control
Sensor plugin structure	XML + .gazebo tags	Same logic, but compatible with ROS2 API
⚙️ Gazebo Simulation in ROS1

In ROS1, a typical simulation system includes the following parts:

gazebo_ros package

.world files (environments)

Robot defined with .urdf or .xacro

Hardware interface with ros_control

Sensor plugins (e.g., gazebo_ros_camera, gazebo_ros_laser)

Launch file example (ROS1):

code
Xml
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch"/>
  <param name="robot_description" command="$(find xacro)/xacro $(find my_robot)/urdf/my_robot.urdf.xacro" />
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model my_robot" />
</launch>
⚙️ Gazebo Simulation in ROS2

In ROS2, the structure has become more modular and standardized. gazebo_ros_pkgs has been ported for ROS2, and thanks to the gz_ros2_control package, robot control works much more integrated.

The main simulation building blocks supported by ROS2:

gazebo_ros: Basic Gazebo-ROS connection

ros2_control: ROS2-based hardware interface

gz_ros2_control: Provides connection between Gazebo and ros2_control

ros_ign: ROS interface for GZ (Ignition) simulation systems

Launch file example (ROS2):
code
Python
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
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
🔌 Hardware and Sensor Integration
Feature	ROS1	ROS2
LIDAR	gazebo_ros_laser plugin	Same XML format, made compatible with ROS2
Camera	gazebo_ros_camera	gazebo_ros_camera (ROS2 port)
Hardware control	ros_control + effort/joint	ros2_control + gz_ros2_control
Plugin loading	<gazebo> tags within URDF	Works with the same method
🛠️ Migration Recommendations

If you are using Gazebo Classic, the ROS1 structure can be directly ported to ROS2.

For new systems, using ros2_control + gz_ros2_control is more performant and sustainable.

ROS2-compatible versions of sensor plugins (with the same name) should be used.

The xacro and robot_state_publisher structure remains the same in ROS2, only the launch system has switched to Python.

🎯 Summary
Feature	ROS1 (Noetic)	ROS2 (Humble)
Simulation infrastructure	gazebo_ros	gazebo_ros_pkgs, gz_ros2_control
Control system	ros_control	ros2_control
Sensor plugins	Plugin-based	Same, ROS2 compatible versions
Launch format	XML (.launch)	Python (.launch.py)
Robot definition	.urdf, .xacro	Same
GZ (Ignition) support	No	Yes (ros_ign, gz_ros2_bridge)

In ROS2, the simulation system has not only been ported, but also made more flexible and powerful in terms of hardware control, parametric management, and launch infrastructure. Gazebo integration is still indispensable for providing a safe testing environment before the real robot.

10. TF and TF2 Usage

In robot systems, the TF (Transform) system is needed to correctly relate sensor data, robot part positions, and moving objects. TF provides transformations between different coordinate systems (e.g., base_link, laser, odom, map). The structure of this system differs in ROS1 and ROS2.

🔄 ROS1: Mixed Use of tf and tf2

In ROS1, both tf and tf2 libraries can be used:

tf is the old system, simple but limited

tf2 is the more modern and recommended system

Both systems have been used together for a long time

Common usage:

tf::TransformListener, tf::TransformBroadcaster (tf)

tf2_ros::Buffer, tf2_ros::TransformListener (tf2)

🔁 ROS2: tf2 Only

With ROS2, the TF system is built entirely on tf2:

tf is no longer supported

All broadcast and lookup operations are done via tf2_ros

Static and dynamic transform publishers are lifecycle compliant

📌 Broadcast and Listen Differences
Operation	ROS1	ROS2
Static transform broadcast	static_transform_publisher CLI or node	ros2 run tf2_ros static_transform_publisher
Dynamic transform broadcast	tf::TransformBroadcaster	tf2_ros.TransformBroadcaster
Transform listening	tf::TransformListener	tf2_ros.TransformListener
TF2 support	Optional	Default and mandatory
Message type	tf/tfMessage	geometry_msgs/msg/TransformStamped
🧪 Static Transform CLI Comparison

ROS1:

code
Bash
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
rosrun tf static_transform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms

ROS2:

code
Bash
download
content_copy
expand_less
IGNORE_WHEN_COPYING_START
IGNORE_WHEN_COPYING_END
ros2 run tf2_ros static_transform_publisher x y z roll pitch yaw frame_id child_frame_id

In ROS2, transformations are entered as roll, pitch, yaw instead of quaternion

The ROS2 command is simpler, operates with automatic frequency

🧩 TF2 Usage Examples (Code Logic)
ROS1:

Listener:

tf::TransformListener listener;

Broadcaster:

tf::TransformBroadcaster br;

ROS2:

Listener:

tf_buffer = Buffer(), listener = TransformListener(buffer, node)

Broadcaster:

StaticTransformBroadcaster, TransformBroadcaster

In ROS2, all these classes are located in the tf2_ros package and run with QoS settings.

🗺️ RViz and TF2

The TF visualization system in RViz (and 'rviz2') is the same in ROS1 and ROS2.

To test that TF trees are being published correctly:

rosrun tf view_frames → ROS1

ros2 run tf2_tools view_frames → ROS2 (outputs as PDF)

✅ Migration Recommendations

All code containing tf:: should be transitioned to the tf2_ros structure.

The transform message type should be geometry_msgs/msg/TransformStamped.

If your ROS1 code uses tf, it will not work directly in ROS2.

The ROS2 version of the CLI commands should be used for static transformations.

Getting used to the tf2_ros.Buffer structure provides a stronger structure in the long run.

Thanks to the transform system in ROS2 being built entirely on tf2, a more consistent, flexible, and DDS-compatible structure has been provided. A correct TF structure is a fundamental prerequisite for the reliable operation of all systems such as navigation, SLAM, and robot arm.

🔧 Additional Tools: Components That Can Help in the Migration Process

It may not always be possible to migrate the entire system directly to ROS2. If some components need to temporarily remain in ROS1, the following tools can assist you in this process.

🔗 ros1_bridge: Bridging Between ROS1 and ROS2

ros1_bridge is a bridge layer that allows you to exchange messages and services between ROS1 and ROS2 systems. It is very useful in temporary solutions or phased migration scenarios.

When to use it?

If some drivers or nodes have not yet been ported to ROS2.

If newly developed systems in ROS2 need to be tested with ROS1 data.

Key Features:

Automatic bridging between messages defined identically in ROS1 and ROS2.

Support for topics, services, and (to a limited extent) actions.

Requires compilation from source code; special messages may require extra configuration.

Official project page:
👉 https://github.com/ros2/ros1_bridge
