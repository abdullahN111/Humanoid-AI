---
sidebar_position: 4
---

# Chapter 3: Humanoid Structure with URDF

## Purpose of URDF in Humanoid Robotics

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. In humanoid robotics, URDF serves as the blueprint that defines the physical structure, kinematic properties, and visual appearance of humanoid robots. It's essential for:

- **Simulation**: Creating accurate physics models for testing and development
- **Control**: Providing kinematic information for motion planning and control algorithms
- **AI Reasoning**: Giving AI systems understanding of the robot's physical capabilities and constraints

URDF enables AI agents to reason about the robot's physical form, understand how different parts connect and move, and plan appropriate actions based on the robot's structure.

## Links, Joints, and Coordinate Frames

### Links: The Rigid Bodies

Links represent the rigid bodies of a robot. Each link has:

- **Physical properties**: Mass, center of mass, and inertia matrix
- **Visual properties**: Mesh or geometric shape for visualization
- **Collision properties**: Shape for collision detection

Example of a link definition:
```xml
<link name="torso">
  <inertial>
    <mass value="10.0" />
    <origin xyz="0 0 0.1" />
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <mesh filename="package://humanoid_description/meshes/torso.dae" />
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0" />
    <geometry>
      <box size="0.3 0.2 0.4" />
    </geometry>
  </collision>
</link>
```

### Joints: The Connections

Joints define how links connect and move relative to each other. The main joint types are:

- **Revolute**: Rotational joint with limited range of motion (e.g., elbow, knee)
- **Continuous**: Rotational joint with unlimited range of motion (e.g., shoulder)
- **Prismatic**: Linear sliding joint (less common in humanoid robots)
- **Fixed**: No movement, used for permanent connections

Example of a joint definition:
```xml
<joint name="left_shoulder_pitch" type="revolute">
  <parent link="torso" />
  <child link="left_upper_arm" />
  <origin xyz="0.0 0.15 0.3" rpy="0 0 0" />
  <axis xyz="1 0 0" />
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0" />
</joint>
```

### Coordinate Frames: The Reference Systems

Each link has its own coordinate frame with:
- **Origin**: Position relative to parent link
- **Orientation**: Rotation relative to parent link
- **Axes**: X, Y, Z directions for movement and rotation

These frames enable:
- **Forward kinematics**: Calculating end-effector position from joint angles
- **Inverse kinematics**: Calculating joint angles to reach desired position
- **Motion planning**: Planning movements in 3D space

## How URDF Enables Simulation, Control, and AI Reasoning

### Simulation Capabilities

URDF enables realistic simulation by providing:

- **Physics properties**: Mass, inertia, and friction parameters
- **Collision geometry**: Shapes for detecting when parts contact each other or the environment
- **Joint limits**: Range of motion constraints that prevent impossible poses
- **Visual models**: 3D meshes for rendering the robot in simulation environments

Example simulation setup:
```xml
<gazebo reference="left_foot">
  <mu1>0.8</mu1>
  <mu2>0.8</mu2>
  <kp>1000000.0</kp>
  <kd>100.0</kd>
  <material>Gazebo/Blue</material>
</gazebo>
```

### Control Capabilities

URDF provides the kinematic structure needed for:

- **Forward kinematics**: Understanding where each part is in space
- **Inverse kinematics**: Calculating joint angles to reach desired positions
- **Dynamics**: Understanding how forces affect robot movement
- **Trajectory planning**: Planning smooth movements between poses

### AI Reasoning Capabilities

URDF enables AI systems to understand:

- **Physical constraints**: What movements are possible and safe
- **Manipulation planning**: How to use end effectors (hands, feet) effectively
- **Balance and stability**: Understanding center of mass and support polygons
- **Task feasibility**: Whether the robot can physically perform requested actions

## URDF Structure and Best Practices

### Complete Robot Definition

A humanoid robot URDF typically includes:

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- Base link -->
  <link name="base_link" />

  <!-- Torso -->
  <link name="torso">
    <!-- ... physical and visual properties ... -->
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link" />
    <child link="torso" />
    <origin xyz="0 0 0" rpy="0 0 0" />
  </joint>

  <!-- Arms -->
  <link name="left_upper_arm">
    <!-- ... properties ... -->
  </link>
  <!-- More links and joints for complete structure ... -->

  <!-- Sensors -->
  <link name="camera_link">
    <!-- Camera mounting point -->
  </link>
  <!-- Joint connecting camera to head ... -->
</robot>
```

### Xacro for Complex Robots

For complex humanoid robots, Xacro (XML Macros) is often used to simplify URDF:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="humanoid">

  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="simple_arm" params="side parent *origin">
    <joint name="${side}_shoulder_pitch_joint" type="revolute">
      <parent link="${parent}" />
      <child link="${side}_upper_arm" />
      <xacro:insert_block name="origin" />
      <axis xyz="1 0 0" />
      <limit lower="${-M_PI/2}" upper="${M_PI/2}" effort="100" velocity="3.0" />
    </joint>
    <!-- More joint and link definitions ... -->
  </xacro:macro>

  <!-- Use the macro to create both arms -->
  <xacro:simple_arm side="left" parent="torso">
    <origin xyz="0.1 0.15 0.3" rpy="0 0 0" />
  </xacro:simple_arm>

  <xacro:simple_arm side="right" parent="torso">
    <origin xyz="0.1 -0.15 0.3" rpy="0 0 0" />
  </xacro:simple_arm>
</robot>
```

## Practical Example: Humanoid Robot URDF

Here's a simplified example of a humanoid robot's upper body:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base and torso -->
  <link name="base_link" />

  <link name="torso">
    <inertial>
      <mass value="15.0" />
      <origin xyz="0 0 0.2" />
      <inertia ixx="0.5" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="0.3" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.2" />
      <geometry>
        <box size="0.3 0.3 0.4" />
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.2" />
      <geometry>
        <box size="0.3 0.3 0.4" />
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 0" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" />
      <geometry>
        <sphere radius="0.1" />
      </geometry>
    </visual>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso" />
    <child link="head" />
    <origin xyz="0 0 0.4" />
    <axis xyz="0 1 0" />
    <limit lower="-0.5" upper="0.5" effort="10" velocity="2.0" />
  </joint>

  <!-- Left arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="2.0" />
      <origin xyz="0 0 -0.1" />
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.005" />
    </inertial>
    <visual>
      <origin xyz="0 0 -0.1" />
      <geometry>
        <cylinder radius="0.05" length="0.2" />
      </geometry>
    </visual>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso" />
    <child link="left_upper_arm" />
    <origin xyz="0.15 0.15 0.3" />
    <axis xyz="0 1 0" />
    <limit lower="-1.57" upper="1.57" effort="50" velocity="3.0" />
  </joint>
</robot>
```

## Using URDF with AI Systems

### Robot State Information for AI

AI systems can use URDF information to:

- **Understand reachability**: Which locations the robot can physically reach
- **Plan safe movements**: Avoiding self-collisions and environmental obstacles
- **Optimize actions**: Choosing the most efficient joint configurations

### URDF in ROS Nodes

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener
import urdf_parser_py.urdf as urdf

class URDFAnalyzerNode(Node):
    def __init__(self):
        super().__init__('urdf_analyzer')

        # Load robot description from parameter server
        robot_desc = self.get_parameter_or('robot_description', '').value

        if robot_desc:
            self.robot_model = urdf.Robot.from_xml_string(robot_desc)
            self.log_robot_info()

    def log_robot_info(self):
        self.get_logger().info(f'Robot name: {self.robot_model.name}')
        self.get_logger().info(f'Number of links: {len(self.robot_model.links)}')
        self.get_logger().info(f'Number of joints: {len(self.robot_model.joints)}')

        # Analyze joint limits for AI planning
        for joint in self.robot_model.joints:
            if joint.joint_type in ['revolute', 'prismatic']:
                self.get_logger().info(
                    f'Joint {joint.name}: limits [{joint.limit.lower}, {joint.limit.upper}]')
```

URDF provides the essential bridge between abstract AI reasoning and concrete robot physics, enabling AI systems to understand and effectively control the physical humanoid robot form.