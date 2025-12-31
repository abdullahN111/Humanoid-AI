# Quickstart Guide: Vision-Language-Action (VLA) Integration

**Feature**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md) | **Date**: 2025-12-30

## Overview

This quickstart guide provides a step-by-step introduction to implementing the Vision-Language-Action (VLA) integration for humanoid robots. The guide covers setting up voice processing, cognitive planning with LLMs, and executing autonomous tasks using the VLA framework.

## Prerequisites

### System Requirements
- Ubuntu 20.04 or later (for ROS 2 Humble Hawksbill)
- Python 3.8 or later
- Node.js 16+ and npm 8+ (for Docusaurus documentation)
- At least 8GB RAM (16GB recommended for LLM processing)
- NVIDIA GPU with 8GB+ VRAM (for local Whisper and LLM processing) or access to cloud APIs
- ROS 2 Humble Hawksbill installed

### Software Dependencies
- ROS 2 Humble Hawksbill
- Docusaurus (for documentation)
- OpenAI Python library (for Whisper API access)
- Transformers library (for local Whisper models)
- PyTorch (for local model execution)

## Installation

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install ROS 2 Dependencies
```bash
# Navigate to the ROS workspace
cd ros_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Install Python Dependencies
```bash
pip3 install torch torchvision torchaudio
pip3 install openai openai-whisper transformers
pip3 install ros2-message-filter ros2-interfaces
```

### 4. Install Docusaurus Documentation Dependencies
```bash
cd frontend
npm install
```

## Setting Up Voice Processing

### 1. Configure Audio Input
```bash
# Check available audio devices
arecord -l

# Test audio recording
arecord -D hw:0,0 -f cd test.wav
```

### 2. Initialize Whisper Model
For local processing:
```python
import whisper
model = whisper.load_model("medium")  # or "small", "large" based on requirements
```

For API-based processing:
```python
import openai
openai.api_key = "your-api-key"
```

### 3. Create Voice Processing Node
Create `voice_processor.py`:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import AudioData
import whisper
import torch

class VoiceProcessor(Node):
    def __init__(self):
        super().__init__('voice_processor')
        self.subscription = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10)
        self.publisher = self.create_publisher(String, 'voice_command', 10)

        # Load Whisper model
        self.model = whisper.load_model("medium")

    def audio_callback(self, msg):
        # Process audio data through Whisper
        audio_np = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32)
        audio_np /= 32768.0  # Normalize to [-1, 1]

        result = self.model.transcribe(audio_np)
        transcript = result["text"]

        # Publish the transcript
        cmd_msg = String()
        cmd_msg.data = transcript
        self.publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    voice_processor = VoiceProcessor()
    rclpy.spin(voice_processor)
    voice_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Setting Up Cognitive Planning

### 1. Configure LLM Access
For OpenAI API:
```bash
export OPENAI_API_KEY="your-api-key"
```

For local LLM (using Hugging Face):
```bash
pip3 install transformers accelerate
```

### 2. Create Planning Node
Create `cognitive_planner.py`:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import json

class CognitivePlanner(Node):
    def __init__(self):
        super().__init__('cognitive_planner')
        self.subscription = self.create_subscription(
            String,
            'voice_command',
            self.command_callback,
            10)
        self.publisher = self.create_publisher(String, 'action_plan', 10)

    def command_callback(self, msg):
        goal = msg.data
        plan = self.generate_plan(goal)

        plan_msg = String()
        plan_msg.data = json.dumps(plan)
        self.publisher.publish(plan_msg)

    def generate_plan(self, goal):
        prompt = f"""
        You are a robotic task planner. Convert the following natural language goal into a sequence of actions for a humanoid robot.

        Goal: {goal}

        Return a JSON object with the following structure:
        {{
            "goal": "{goal}",
            "steps": [
                {{
                    "action": "action_name",
                    "parameters": {{"param1": "value1", ...}},
                    "description": "Human-readable description"
                }}
            ]
        }}

        Actions available: move_to, pick_up, place, speak, wait, navigate, manipulate
        """

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.3
        )

        return json.loads(response.choices[0].message.content)

def main(args=None):
    rclpy.init(args=args)
    cognitive_planner = CognitivePlanner()
    rclpy.spin(cognitive_planner)
    cognitive_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Setting Up Plan Execution

### 1. Create Execution Manager
Create `execution_manager.py`:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
import json

class ExecutionManager(Node):
    def __init__(self):
        super().__init__('execution_manager')
        self.subscription = self.create_subscription(
            String,
            'action_plan',
            self.plan_callback,
            10)

        # Initialize action clients for different robot capabilities
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.manipulation_client = ActionClient(self, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')

    def plan_callback(self, msg):
        plan = json.loads(msg.data)
        self.execute_plan(plan)

    def execute_plan(self, plan):
        for step in plan['steps']:
            self.execute_step(step)

    def execute_step(self, step):
        action = step['action']
        params = step['parameters']

        if action == 'move_to':
            self.execute_navigation(params)
        elif action == 'pick_up':
            self.execute_manipulation(params)
        # Add other action handlers as needed

def main(args=None):
    rclpy.init(args=args)
    execution_manager = ExecutionManager()
    rclpy.spin(execution_manager)
    execution_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Running the Complete VLA System

### 1. Launch the System
Create a launch file `vla_system.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vla_integration',
            executable='voice_processor',
            name='voice_processor'
        ),
        Node(
            package='vla_integration',
            executable='cognitive_planner',
            name='cognitive_planner'
        ),
        Node(
            package='vla_integration',
            executable='execution_manager',
            name='execution_manager'
        )
    ])
```

### 2. Start the System
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch the VLA system
ros2 launch vla_integration vla_system.launch.py
```

### 3. Test with Sample Command
In another terminal:
```bash
# Send a test voice command
ros2 topic pub /voice_command std_msgs/String "data: 'Please move to the kitchen and bring me a cup'"
```

## Building Documentation

### 1. Navigate to Documentation Directory
```bash
cd frontend
```

### 2. Start Documentation Server
```bash
npm start
```

### 3. Build Documentation for Production
```bash
npm run build
```

## Troubleshooting

### Common Issues

1. **Audio Input Not Working**
   - Check audio device permissions: `sudo usermod -a -G audio $USER`
   - Verify audio device: `arecord -l`
   - Test with: `arecord -D hw:0,0 -f cd test.wav`

2. **Whisper Model Loading Fails**
   - Ensure sufficient GPU memory
   - Check PyTorch installation: `python3 -c "import torch; print(torch.cuda.is_available())"`
   - Verify model download: `python3 -c "import whisper; whisper.load_model('small')"`

3. **LLM API Access Issues**
   - Verify API key is set: `echo $OPENAI_API_KEY`
   - Check network connectivity
   - Verify account has API access

### Performance Tuning

1. **Reduce Whisper Model Size**
   ```python
   model = whisper.load_model("small")  # Instead of "medium" or "large"
   ```

2. **Optimize LLM Usage**
   - Use local models for simple tasks
   - Implement caching for repeated requests
   - Use appropriate model size for task complexity

3. **Memory Management**
   - Clear GPU cache periodically: `torch.cuda.empty_cache()`
   - Use mixed precision: `torch.cuda.amp.GradScaler()`

## Next Steps

1. **Advanced Configuration**: Explore configuration options for each component
2. **Simulation Testing**: Use Isaac Sim or Gazebo for testing in simulation
3. **Hardware Integration**: Connect to physical robot hardware
4. **Custom Actions**: Extend the system with custom robot actions
5. **Performance Optimization**: Fine-tune for specific hardware configurations

## Resources

- [Module 1: ROS 2 Nervous System](../module-1/intro.md) - Foundation for communication
- [Module 2: Digital Twin Simulation](../module-2/intro.md) - Simulation environment setup
- [Module 3: AI-Robot Brain](../module-3/intro.md) - Perception and navigation components
- [API Contracts](./contracts/api-contracts.md) - Detailed interface specifications
- [Data Models](./data-model.md) - Data structure definitions
- [Research Findings](./research.md) - Technical research and decisions