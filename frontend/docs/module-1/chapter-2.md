---
sidebar_position: 3
---

# Chapter 2: Connecting Python AI Agents to ROS 2

## Using rclpy to Build ROS 2 Nodes in Python

rclpy is the Python client library for ROS 2, providing a Python API to create ROS 2 nodes, publish and subscribe to topics, and provide or use services. It enables Python-based AI agents to seamlessly integrate with ROS 2 systems.

### Installing rclpy

rclpy is typically installed as part of the ROS 2 Python development packages. To install:

```bash
sudo apt install python3-ros-dev-tools  # On Ubuntu
```

Or use pip if available:

```bash
pip install rclpy
```

### Basic Node Structure

A basic ROS 2 node in Python using rclpy follows this pattern:

```python
import rclpy
from rclpy.node import Node

class MyAIAgentNode(Node):
    def __init__(self):
        super().__init__('ai_agent_node')
        # Initialize publishers, subscribers, and services here

    def callback_function(self, msg):
        # Process incoming messages
        pass

def main(args=None):
    rclpy.init(args=args)
    ai_agent_node = MyAIAgentNode()

    try:
        rclpy.spin(ai_agent_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Building Python ROS 2 Nodes

### Creating a Publisher Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AIPublisherNode(Node):
    def __init__(self):
        super().__init__('ai_publisher_node')
        self.publisher = self.create_publisher(String, 'ai_decisions', 10)
        self.timer = self.create_timer(1.0, self.publish_decision)
        self.counter = 0

    def publish_decision(self):
        msg = String()
        msg.data = f'AI Decision {self.counter}: Move forward'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.counter += 1
```

### Creating a Subscriber Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class AISubscriberNode(Node):
    def __init__(self):
        super().__init__('ai_subscriber_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Process image with AI model
        decision = self.process_with_ai(cv_image)

        # Publish decision to another topic
        self.publish_decision(decision)

    def process_with_ai(self, image):
        # Your AI processing logic here
        return "decision"
```

## Bridging AI/LLM Agents with ROS Controllers

### AI Decision Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import openai  # Example with OpenAI API

class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')

        # Subscribe to sensor data
        self.sensor_sub = self.create_subscription(
            String, 'sensor_data', self.sensor_callback, 10)

        # Publish commands to robot
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.ai_context = []

    def sensor_callback(self, msg):
        # Add sensor data to AI context
        self.ai_context.append(msg.data)

        # Generate decision with AI
        decision = self.ask_ai_for_decision()

        # Convert AI decision to robot command
        cmd = self.decision_to_command(decision)
        self.cmd_pub.publish(cmd)

    def ask_ai_for_decision(self):
        # Query AI model with current context
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=self.ai_context
        )
        return response.choices[0].message.content

    def decision_to_command(self, decision):
        # Convert AI decision text to Twist message
        cmd = Twist()
        if "forward" in decision.lower():
            cmd.linear.x = 0.5
        elif "turn left" in decision.lower():
            cmd.angular.z = 0.5
        # Add more mappings as needed
        return cmd
```

## Command Flow from Decision-Making to Actuation

### Complete Flow Example

The complete flow from AI decision-making to robot actuation involves several steps:

1. **Sensing**: Robot sensors publish data to topics (e.g., `/camera/image_raw`, `/scan`, `/imu/data`)
2. **Perception**: Perception nodes process raw sensor data and publish processed information (e.g., `/objects/detected`, `/navigation/map`)
3. **AI Decision**: AI nodes subscribe to processed data and generate high-level decisions (e.g., `/ai/decision`, `/navigation/goal`)
4. **Planning**: Motion planning nodes convert high-level goals to specific commands (e.g., `/trajectory/plan`)
5. **Control**: Low-level controllers execute commands on hardware (e.g., `/cmd_vel`, `/joint_commands`)

### Example Integration Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # Subscribe to sensor data
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)

        # Publish robot commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Publish to AI system
        self.ai_pub = self.create_publisher(String, 'ai_input', 10)

        # Subscribe to AI decisions
        self.ai_sub = self.create_subscription(
            String, 'ai_decision', self.ai_decision_callback, 10)

        self.safety_enabled = True

    def scan_callback(self, msg):
        # Process LIDAR data
        min_distance = min(msg.ranges)

        # Check for obstacles
        if min_distance < 0.5 and self.safety_enabled:
            # Emergency stop
            stop_cmd = Twist()
            self.cmd_pub.publish(stop_cmd)
            self.get_logger().warn('Safety stop: obstacle detected!')
            return

        # Prepare sensor data for AI
        ai_input = f"Obstacle distance: {min_distance:.2f}m"
        self.ai_pub.publish(String(data=ai_input))

    def ai_decision_callback(self, msg):
        # Convert AI decision to robot command
        decision = msg.data.lower()
        cmd = Twist()

        if 'move forward' in decision:
            cmd.linear.x = 0.3
        elif 'turn left' in decision:
            cmd.angular.z = 0.5
        elif 'turn right' in decision:
            cmd.angular.z = -0.5
        elif 'stop' in decision:
            # Command is already zero
            pass

        self.cmd_pub.publish(cmd)
```

## Best Practices for AI-ROS Integration

### Error Handling and Safety

```python
def ai_decision_callback(self, msg):
    try:
        # Validate AI decision
        if not self.validate_decision(msg.data):
            self.get_logger().warn(f'Invalid AI decision: {msg.data}')
            return

        # Convert to command
        cmd = self.decision_to_command(msg.data)

        # Validate command is safe
        if self.is_safe_command(cmd):
            self.cmd_pub.publish(cmd)
        else:
            self.get_logger().warn('Unsafe command from AI prevented')

    except Exception as e:
        self.get_logger().error(f'Error processing AI decision: {e}')
        # Emergency stop
        self.emergency_stop()
```

### State Management

Maintain consistent state between the AI agent and the physical robot:

```python
class AIBridgeNode(Node):
    def __init__(self):
        # ... other initialization
        self.robot_state = {
            'position': None,
            'orientation': None,
            'velocity': None,
            'battery_level': None
        }

    def update_robot_state(self, msg, state_key):
        self.robot_state[state_key] = msg.data
        self.publish_state_to_ai()

    def publish_state_to_ai(self):
        state_msg = String()
        state_msg.data = str(self.robot_state)
        self.ai_state_pub.publish(state_msg)
```

This chapter provides the foundation for connecting Python-based AI agents to ROS 2 systems. The next chapter will cover how URDF defines humanoid robot structure.