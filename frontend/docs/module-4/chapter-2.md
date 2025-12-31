---
sidebar_position: 2
---

# Chapter 2: Cognitive Planning with Large Language Models

## Overview
This chapter covers cognitive planning capabilities using Large Language Models for translating natural language goals into executable action plans, using LLMs as high-level planners and task decomposers that bridge LLM reasoning with ROS 2 action graphs to create intelligent robots that can reason about complex tasks.

## Learning Objectives
By the end of this chapter, you will be able to:
- Use Large Language Models for cognitive planning in robotics (FR-004)
- Translate natural language goals into executable action plans (FR-005)
- Apply LLMs as high-level planners and task decomposers (FR-006)
- Bridge LLM reasoning with ROS 2 action graphs (FR-007)

## Table of Contents
- Introduction to LLM-based Cognitive Planning
- LLM Selection and Configuration
- Prompt Engineering for Robotic Tasks
- Action Plan Generation and Validation
- Integration with ROS 2 Action Graphs
- Practical Examples and Code

## Introduction
Large Language Models (LLMs) can serve as cognitive planners for robotic systems, decomposing high-level goals into executable action sequences. This chapter explores how to leverage LLMs for robotic task planning and execution.

## LLM Selection and Configuration
Choosing the right LLM for robotic planning depends on various factors including computational requirements, response quality, and privacy concerns.

### Cloud-based LLMs
Cloud-based LLMs like OpenAI's GPT models or Anthropic's Claude offer powerful reasoning capabilities with minimal computational overhead on the robot.

```python
import openai

# Configure OpenAI API
openai.api_key = os.getenv("OPENAI_API_KEY")

def plan_with_llm(goal, context):
    prompt = f"""
    You are a robotic task planner. Given the following goal and context,
    decompose the task into a sequence of executable actions for a humanoid robot.

    Goal: {goal}
    Context: {context}

    Return a JSON object with the following structure:
    {{
        "goal": "{goal}",
        "steps": [
            {{
                "action": "action_name",
                "parameters": {{"param1": "value1", ...}},
                "description": "Human-readable description"
            }}
        ],
        "estimated_duration": "estimated time in seconds"
    }}

    Available actions: move_to, pick_up, place, speak, wait, navigate, manipulate, perceive
    """

    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.3
    )

    return json.loads(response.choices[0].message.content)
```

### Local LLMs
For privacy or latency-sensitive applications, local models like Llama or Mistral can be used.

```python
from transformers import AutoTokenizer, AutoModelForCausalLM
import torch

# Load a local model (adjust based on computational resources)
model_name = "microsoft/DialoGPT-medium"
tokenizer = AutoTokenizer.from_pretrained(model_name)
model = AutoModelForCausalLM.from_pretrained(model_name)

def plan_with_local_llm(goal, context):
    # Implementation for local LLM planning
    # This is a simplified example - actual implementation would require
    # more sophisticated prompt engineering and response parsing
    pass
```

## Prompt Engineering for Robotic Tasks
Effective prompt engineering is crucial for getting reliable action plans from LLMs.

### System Prompt Template
```python
SYSTEM_PROMPT = """
You are a robotic task planning assistant. Your role is to decompose high-level goals into executable action sequences for humanoid robots.

Guidelines:
1. Always return valid JSON
2. Use only the available actions: {available_actions}
3. Include parameters for each action
4. Consider robot capabilities and environmental constraints
5. Include error handling and fallback options when appropriate
"""
```

### Example Prompt Structure
```python
def create_planning_prompt(goal, context, robot_capabilities, environment_state):
    return f"""
{SYSTEM_PROMPT}

Goal: {goal}

Context:
- Robot capabilities: {robot_capabilities}
- Environment state: {environment_state}
- Constraints: {context.get('constraints', {})}

Please decompose this goal into a sequence of actions that the robot can execute to achieve the goal.
"""
```

## Action Plan Generation and Validation
LLMs generate action plans that need to be validated before execution.

### Plan Validation
```python
def validate_plan(plan, robot_state, environment_state):
    """Validate an LLM-generated plan against robot capabilities and environment constraints."""
    for step in plan['steps']:
        # Check if robot can perform the action
        if not can_perform_action(robot_state, step['action'], step.get('parameters', {})):
            return False, f"Robot cannot perform action: {step['action']}"

        # Check if environment allows the action
        if not environment_allows_action(environment_state, step['action'], step.get('parameters', {})):
            return False, f"Environment does not allow action: {step['action']}"

    return True, "Plan is valid"
```

## Integration with ROS 2 Action Graphs
LLM-generated plans need to be converted to ROS 2 action graphs for execution.

### Action Graph Structure
```python
class ActionGraph:
    def __init__(self, plan):
        self.nodes = []
        self.edges = []
        self.plan = plan
        self.create_graph_from_plan()

    def create_graph_from_plan(self):
        """Convert LLM plan to ROS 2 action graph."""
        for i, step in enumerate(self.plan['steps']):
            node = self.create_action_node(step)
            self.nodes.append(node)

            # Add dependencies between steps
            if i > 0:
                self.edges.append((self.nodes[i-1], node))

    def create_action_node(self, step):
        """Create a ROS 2 action node from a plan step."""
        # Map LLM action to ROS 2 action
        ros_action = self.map_to_ros_action(step['action'], step['parameters'])
        return ros_action
```

## Performance and Cost Optimization
LLM-based planning involves trade-offs between cost, latency, and quality.

### Caching Strategies
```python
import hashlib
from functools import lru_cache

@lru_cache(maxsize=128)
def cached_plan_generation(goal_hash, context_hash):
    """Cache frequently requested plans to reduce API costs and latency."""
    # Implementation would use the hash values to retrieve or generate plans
    pass

def get_plan_with_cache(goal, context):
    goal_hash = hashlib.md5(goal.encode()).hexdigest()
    context_hash = hashlib.md5(str(context).encode()).hexdigest()

    return cached_plan_generation(goal_hash, context_hash)
```

## Summary
This chapter introduced LLM-based cognitive planning for robotic systems, covering model selection, prompt engineering, plan validation, and integration with ROS 2. The next chapter will explore how to combine voice-to-action and cognitive planning capabilities into a complete autonomous humanoid system.

## References
- [OpenAI API Documentation](https://platform.openai.com/docs/)
- [Hugging Face Transformers](https://huggingface.co/transformers/)
- [Large Language Models for Robotics](https://arxiv.org/abs/2206.07648)
- [ROS 2 Action Architecture](https://docs.ros.org/en/humble/Concepts/About-Actions.html)