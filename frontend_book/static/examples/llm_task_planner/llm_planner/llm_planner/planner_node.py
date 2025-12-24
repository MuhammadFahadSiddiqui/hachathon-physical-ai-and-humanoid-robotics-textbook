#!/usr/bin/env python3
"""
LLM Task Planner Node

ROS 2 node that translates natural language commands to robot action sequences
using LLMs (OpenAI GPT-4 or Ollama Llama 3.1).

Author: Module 4 - Vision-Language-Action (VLA)
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from typing import List, Dict, Any, Optional

# LLM clients
try:
    from openai import OpenAI, OpenAIError
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

import requests

# Local imports
from llm_planner.prompt_templates import (
    build_messages_for_openai,
    build_prompt_for_ollama,
    get_cached_plan
)
from llm_planner.action_primitives import validate_action_plan


class LLMPlannerNode(Node):
    """
    ROS 2 node for LLM-based task planning.

    Subscribes to /voice_commands, generates action plans using LLM,
    validates plans, and publishes to /task_plan.

    Parameters:
        llm_backend (str): "openai" or "ollama"
        model (str): Model name (e.g., "gpt-4-turbo", "llama3.1:8b")
        temperature (float): LLM temperature (0.0 = deterministic)
        max_tokens (int): Maximum response tokens
        timeout (float): API call timeout in seconds
        openai_api_key (str): OpenAI API key (env var)
        ollama_host (str): Ollama server URL
        max_retries (int): Max retries for invalid plans
    """

    def __init__(self):
        super().__init__('llm_planner_node')

        # Declare parameters
        self.declare_parameter('llm_backend', 'ollama')
        self.declare_parameter('model', 'llama3.1:8b')
        self.declare_parameter('temperature', 0.0)
        self.declare_parameter('max_tokens', 500)
        self.declare_parameter('timeout', 30.0)
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('ollama_host', 'http://localhost:11434')
        self.declare_parameter('max_retries', 3)

        # Get parameters
        self.llm_backend = self.get_parameter('llm_backend').value
        self.model = self.get_parameter('model').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.timeout = self.get_parameter('timeout').value
        self.max_retries = self.get_parameter('max_retries').value

        # Initialize LLM client
        if self.llm_backend == 'openai':
            if not OPENAI_AVAILABLE:
                self.get_logger().error('OpenAI SDK not installed. Run: pip install openai')
                raise RuntimeError('OpenAI SDK not available')

            api_key = self.get_parameter('openai_api_key').value
            if not api_key:
                self.get_logger().error('OPENAI_API_KEY not set. Set via environment variable or parameter.')
                raise RuntimeError('OpenAI API key required')

            self.openai_client = OpenAI(api_key=api_key)
            self.get_logger().info(f'Initialized OpenAI client with model: {self.model}')

        elif self.llm_backend == 'ollama':
            self.ollama_host = self.get_parameter('ollama_host').value
            # Test Ollama connection
            try:
                response = requests.get(f'{self.ollama_host}/api/tags', timeout=5)
                response.raise_for_status()
                self.get_logger().info(f'Connected to Ollama at {self.ollama_host}')
                self.get_logger().info(f'Using model: {self.model}')
            except Exception as e:
                self.get_logger().error(f'Failed to connect to Ollama: {e}')
                self.get_logger().error('Ensure Ollama is running: ollama serve')
                raise

        else:
            raise ValueError(f'Invalid llm_backend: {self.llm_backend}. Use "openai" or "ollama"')

        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            10
        )

        self.publisher = self.create_publisher(String, 'task_plan', 10)

        self.get_logger().info(
            f'LLM Planner started: backend={self.llm_backend}, '
            f'model={self.model}, temp={self.temperature}'
        )

    def voice_command_callback(self, msg: String):
        """Handle incoming voice commands and generate task plans."""
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')

        start_time = time.time()

        # Check cached plans first
        cached_plan = get_cached_plan(command)
        if cached_plan:
            self.get_logger().info(f'Using cached plan for: "{command}"')
            self.publish_plan(cached_plan, time.time() - start_time, from_cache=True)
            return

        # Generate plan with LLM (with retries)
        plan = None
        error_message = None

        for attempt in range(self.max_retries):
            try:
                plan = self.generate_plan(command)

                # Validate plan
                is_valid, error = validate_action_plan(plan)

                if is_valid:
                    generation_time = time.time() - start_time
                    self.get_logger().info(
                        f'Generated valid plan with {len(plan)} actions in {generation_time:.2f}s'
                    )
                    self.publish_plan(plan, generation_time)
                    return
                else:
                    self.get_logger().warn(
                        f'Invalid plan (attempt {attempt + 1}/{self.max_retries}): {error}'
                    )
                    error_message = error
                    # Retry with error feedback
                    command += f"\nPrevious attempt failed: {error}. Please fix and try again."

            except Exception as e:
                self.get_logger().error(f'Planning failed (attempt {attempt + 1}/{self.max_retries}): {e}')
                error_message = str(e)

        # All retries exhausted
        self.get_logger().error(f'Failed to generate valid plan after {self.max_retries} attempts')
        self.publish_error(command, error_message or "Unknown error")

    def generate_plan(self, command: str) -> List[Dict[str, Any]]:
        """
        Generate action plan using LLM.

        Args:
            command: Natural language command

        Returns:
            List of action dictionaries

        Raises:
            ValueError: If response is invalid JSON or malformed
        """
        if self.llm_backend == 'openai':
            return self.generate_plan_openai(command)
        elif self.llm_backend == 'ollama':
            return self.generate_plan_ollama(command)
        else:
            raise ValueError(f'Unknown backend: {self.llm_backend}')

    def generate_plan_openai(self, command: str) -> List[Dict[str, Any]]:
        """Generate plan using OpenAI API."""
        messages = build_messages_for_openai(command)

        try:
            response = self.openai_client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
                timeout=self.timeout
            )

            response_text = response.choices[0].message.content
            self.get_logger().debug(f'OpenAI response: {response_text}')

            return self.parse_llm_response(response_text)

        except OpenAIError as e:
            raise ValueError(f'OpenAI API error: {e}')

    def generate_plan_ollama(self, command: str) -> List[Dict[str, Any]]:
        """Generate plan using Ollama API."""
        prompt = build_prompt_for_ollama(command)

        try:
            response = requests.post(
                f'{self.ollama_host}/api/generate',
                json={
                    'model': self.model,
                    'prompt': prompt,
                    'stream': False,
                    'options': {
                        'temperature': self.temperature,
                        'num_predict': self.max_tokens
                    }
                },
                timeout=self.timeout
            )

            response.raise_for_status()
            response_data = response.json()
            response_text = response_data['response']

            self.get_logger().debug(f'Ollama response: {response_text}')

            return self.parse_llm_response(response_text)

        except requests.RequestException as e:
            raise ValueError(f'Ollama API error: {e}')

    def parse_llm_response(self, response_text: str) -> List[Dict[str, Any]]:
        """
        Parse LLM response to extract action plan.

        Args:
            response_text: Raw LLM output

        Returns:
            List of action dictionaries

        Raises:
            ValueError: If response is invalid JSON or contains error
        """
        # Remove markdown code blocks if present
        text = response_text.strip()

        if text.startswith("```"):
            # Extract content between code fences
            parts = text.split("```")
            if len(parts) >= 2:
                text = parts[1]
                # Remove language identifier (e.g., "json")
                if text.startswith("json"):
                    text = text[4:].strip()

        # Parse JSON
        try:
            plan = json.loads(text)
        except json.JSONDecodeError as e:
            raise ValueError(f'Invalid JSON from LLM: {e}\nResponse: {text}')

        # Check for error response
        if isinstance(plan, dict) and 'error' in plan:
            raise ValueError(f"LLM requested clarification: {plan['error']}")

        # Validate structure
        if not isinstance(plan, list):
            raise ValueError(f'Plan must be JSON array, got: {type(plan)}')

        return plan

    def publish_plan(self, plan: List[Dict[str, Any]], generation_time: float, from_cache: bool = False):
        """Publish validated plan to /task_plan topic."""
        total_duration = sum(action.get('expected_duration', 0) for action in plan)

        plan_msg = {
            'plan': plan,
            'total_duration': total_duration,
            'num_actions': len(plan),
            'llm_model': self.model if not from_cache else 'cached',
            'generation_time': generation_time,
            'from_cache': from_cache
        }

        msg = String()
        msg.data = json.dumps(plan_msg, indent=2)
        self.publisher.publish(msg)

        self.get_logger().info(f'Published plan: {len(plan)} actions, {total_duration:.1f}s total')

    def publish_error(self, command: str, error: str):
        """Publish error message to /task_plan topic."""
        error_msg = {
            'error': error,
            'command': command,
            'llm_model': self.model,
            'timestamp': time.time()
        }

        msg = String()
        msg.data = json.dumps(error_msg, indent=2)
        self.publisher.publish(msg)


def main(args=None):
    """Main entry point for the LLM planner node."""
    rclpy.init(args=args)

    try:
        node = LLMPlannerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
