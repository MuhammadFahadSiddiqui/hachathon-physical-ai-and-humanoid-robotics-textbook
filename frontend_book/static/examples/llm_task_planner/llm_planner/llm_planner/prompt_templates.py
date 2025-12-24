#!/usr/bin/env python3
"""
Prompt Templates for LLM Task Planning

System prompts and few-shot examples for constraining LLM outputs to valid
robot action sequences.

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

from typing import List, Dict, Any


SYSTEM_PROMPT = """You are a task planning AI for a humanoid robot. Your role is to decompose natural language commands into executable action sequences using ONLY the following 12 action primitives:

AVAILABLE ACTIONS:
- navigate(target: str | [x,y,theta])  // Move to location or coordinates
- stop()  // Emergency halt
- follow(target_class: str, distance: float)  // Track moving target
- detect_object(object_class: str, max_results: int)  // Vision-based object detection
- scan_environment()  // 360° LIDAR scan
- grasp(object_id: str, force: float)  // Close gripper on object
- release()  // Open gripper, drop object
- open_gripper(width: float)  // Set gripper to open position
- close_gripper(force: float)  // Set gripper to closed position
- move_arm(pose_name: str)  // Move arm to predefined pose (home/ready/stow/neutral)
- wait(duration: float)  // Pause for seconds
- say(text: str)  // Text-to-speech output

CONSTRAINTS:
1. Output ONLY valid JSON (no markdown, no explanations, no code blocks)
2. Use only actions from the list above - NO other actions
3. Decompose complex tasks into step-by-step primitives
4. Ensure preconditions are met (e.g., grasp requires detect_object first)
5. Include expected_duration for each action (seconds)
6. Known locations: kitchen, bedroom, living_room, bathroom, table, shelf, charging_station, home, user

OUTPUT FORMAT:
[
  {"action": "action_name", "parameters": {...}, "expected_duration": X.X},
  ...
]

If the command is ambiguous or impossible, output:
{"error": "Clarification needed: <specific question>"}
"""


FEW_SHOT_EXAMPLES = [
    {
        "user": "Go to the kitchen",
        "assistant": '[{"action": "navigate", "parameters": {"target": "kitchen"}, "expected_duration": 15.0}]'
    },
    {
        "user": "Pick up the red cube",
        "assistant": '[{"action": "detect_object", "parameters": {"object_class": "cube", "max_results": 5}, "expected_duration": 3.0}, {"action": "navigate", "parameters": {"target": "detected_object_pose"}, "expected_duration": 5.0}, {"action": "grasp", "parameters": {"object_id": "cube_0", "force": 8.0}, "expected_duration": 4.0}]'
    },
    {
        "user": "Bring me the bottle from the table",
        "assistant": '[{"action": "navigate", "parameters": {"target": "table"}, "expected_duration": 10.0}, {"action": "detect_object", "parameters": {"object_class": "bottle", "max_results": 1}, "expected_duration": 3.0}, {"action": "grasp", "parameters": {"object_id": "bottle_0", "force": 10.0}, "expected_duration": 4.0}, {"action": "navigate", "parameters": {"target": "user"}, "expected_duration": 10.0}, {"action": "say", "parameters": {"text": "Here is your bottle"}, "expected_duration": 2.0}]'
    },
    {
        "user": "Stop immediately",
        "assistant": '[{"action": "stop", "parameters": {}, "expected_duration": 0.5}]'
    },
    {
        "user": "Clean the room",
        "assistant": '{"error": "Clarification needed: What specific cleaning task? (pick up objects, vacuum, wipe surfaces)"}'
    }
]


def build_messages_for_openai(user_command: str) -> List[Dict[str, str]]:
    """
    Build OpenAI Chat API messages with system prompt and few-shot examples.

    Args:
        user_command: Natural language command from user

    Returns:
        List of message dictionaries for OpenAI API
    """
    messages = [
        {"role": "system", "content": SYSTEM_PROMPT}
    ]

    # Add few-shot examples
    for example in FEW_SHOT_EXAMPLES:
        messages.append({"role": "user", "content": example["user"]})
        messages.append({"role": "assistant", "content": example["assistant"]})

    # Add actual user command
    messages.append({"role": "user", "content": user_command})

    return messages


def build_prompt_for_ollama(user_command: str) -> str:
    """
    Build prompt for Ollama (which uses single-string prompts, not chat format).

    Args:
        user_command: Natural language command from user

    Returns:
        Complete prompt string
    """
    prompt = SYSTEM_PROMPT + "\n\n"

    # Add few-shot examples
    for example in FEW_SHOT_EXAMPLES:
        prompt += f"USER: {example['user']}\n"
        prompt += f"ASSISTANT: {example['assistant']}\n\n"

    # Add actual user command
    prompt += f"USER: {user_command}\n"
    prompt += "ASSISTANT: "

    return prompt


# Cached plans for common commands (fallback when LLM fails)
CACHED_PLANS = {
    "stop": [
        {"action": "stop", "parameters": {}, "expected_duration": 0.5}
    ],
    "go home": [
        {"action": "navigate", "parameters": {"target": "home"}, "expected_duration": 20.0}
    ],
    "go to charging station": [
        {"action": "navigate", "parameters": {"target": "charging_station"}, "expected_duration": 20.0}
    ],
    "emergency stop": [
        {"action": "stop", "parameters": {}, "expected_duration": 0.5}
    ],
    "reset": [
        {"action": "move_arm", "parameters": {"pose_name": "home"}, "expected_duration": 3.0},
        {"action": "open_gripper", "parameters": {"width": 0.08}, "expected_duration": 1.5}
    ],
}


def get_cached_plan(command: str) -> List[Dict[str, Any]] | None:
    """
    Get cached plan for common command if available.

    Args:
        command: User command (lowercased)

    Returns:
        Cached plan or None if not found
    """
    return CACHED_PLANS.get(command.lower().strip())
