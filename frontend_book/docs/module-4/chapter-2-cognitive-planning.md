---
title: "Cognitive Planning with LLMs"
sidebar_position: 3
id: chapter-2-cognitive-planning
---

# Chapter 2: Cognitive Planning with LLMs

## Introduction

Imagine instructing a humanoid robot with the simple command: "Go to the kitchen and bring me a water bottle." For a human, this task is intuitive, requiring spatial reasoning, object recognition, and sequential planning. For a robot, however, this natural language instruction must be decomposed into a precise sequence of low-level actions: navigate to kitchen, scan for water bottles, approach detected object, activate gripper, grasp bottle, navigate back to user. This translation from high-level natural language to executable robot primitives is the domain of **cognitive task planning** using Large Language Models (LLMs).

In this chapter, you'll build an LLM-based task planner that transforms voice commands from Chapter 1 into structured action sequences for your humanoid robot. Unlike rule-based systems that require manually coding every possible command variant, LLMs leverage billions of parameters trained on internet-scale text data to generalize to novel instructions, understand context, and reason about task feasibility.

**Two LLM Deployment Options**: You can implement this system using either OpenAI's GPT-4 API (cloud-based, ~$0.03 per planning request, <5s latency) or Meta's Llama 3.1 8B model via Ollama (local inference, completely free, 5-15s latency on CPU). Both options produce valid action sequences with >95% accuracy on standard robot tasks, making LLM-based planning accessible regardless of your budget.

**The VLA Research Foundation**: This chapter builds on cutting-edge Vision-Language-Action (VLA) research. Google's **SayCan** (2022) pioneered grounding language models in robot affordances by combining LLM task planning with learned value functions to ensure physical feasibility. Liang et al.'s **Code as Policies** (2022) demonstrated that LLMs can generate executable robot code directly from natural language, treating planning as a code generation problem. These papers established that modern LLMs possess sufficient world knowledge and reasoning capabilities for real-world robot control when properly constrained to valid action primitives.

**Action Primitive Decomposition**: The key to robust LLM planning is defining a library of **action primitives**—atomic robot behaviors like `navigate(target)`, `grasp(object)`, `detect_object(class)`, `open_gripper()`. Your LLM planner will learn to decompose complex tasks into sequences of these primitives, validated against your robot's capabilities before execution. By the end of this chapter, you'll have a working planner that translates arbitrary natural language commands into JSON action sequences executable by your humanoid's navigation, perception, and manipulation systems.

**What You'll Build**: A ROS 2 node (`llm_planner`) that subscribes to `/voice_commands` from Chapter 1, sends the transcribed command to your chosen LLM with a carefully engineered prompt constraining outputs to valid action primitives, parses the JSON response, validates the plan, and publishes it to `/task_plan` for execution. This planner will handle both simple commands ("stop") and complex multi-step tasks ("find the red bottle on the shelf and bring it to the table"), serving as the cognitive brain for your autonomous humanoid.

## LLM Setup: Two Deployment Paths

You have two equally viable options for running the LLM planner. Choose based on your priorities—**cost vs latency**, **privacy vs performance**, **local vs cloud**. Both paths are fully supported in the downloadable code, and you can switch between them by changing a single configuration parameter.

### Path 1: Ollama (Local Inference) ✅ Recommended for Students

**Ollama** is an open-source tool that runs LLMs locally on your machine, similar to how you run Docker containers. It manages model downloads, inference, and provides a REST API compatible with OpenAI's format.

**Advantages**:
- **Cost**: $0 (completely free, unlimited requests)
- **Privacy**: All processing local, no API calls, no data sent to cloud
- **Offline**: Works without internet after initial model download
- **Learning**: Full visibility into inference process, model files, prompts

**Disadvantages**:
- **Latency**: 5-10s for simple plans (CPU), 2-5s with GPU
- **RAM**: Requires 16GB+ for Llama 3.1 8B model (24GB recommended)
- **Quality**: ~90-93% valid action sequences (vs 95-98% for GPT-4)

**Installation** (Ubuntu 22.04):

```bash
# Download and install Ollama
curl -fsSL https://ollama.com/install.sh | sh

# Verify installation
ollama --version  # Should output v0.1.x or later

# Pull Llama 3.1 8B model (~4.7GB download)
ollama pull llama3.1:8b

# Start Ollama server (runs in background)
ollama serve

# Test API (should return model info)
curl http://localhost:11434/api/tags
```

**First Test**:
```bash
# Test task planning with Llama 3.1
curl http://localhost:11434/api/generate -d '{
  "model": "llama3.1:8b",
  "prompt": "Decompose this robot task into actions: go to the kitchen",
  "stream": false
}'
```

The server runs on `localhost:11434` and provides an OpenAI-compatible REST API. The Llama 3.1 8B model will auto-load into RAM on first request (~6-8GB memory usage).

### Path 2: OpenAI GPT-4 API (Cloud Inference) ⚡ Best Performance

**OpenAI's GPT-4** is a state-of-the-art LLM with 1.76 trillion parameters (220x larger than Llama 3.1 8B), offering superior reasoning and instruction following.

**Advantages**:
- **Quality**: ~95-98% valid action sequences, handles ambiguous commands better
- **Latency**: <5s for simple plans, <15s for complex (faster than local Llama on CPU)
- **No Hardware Requirements**: Runs on minimal RAM, offloads to cloud
- **Latest Models**: Access to GPT-4 Turbo, GPT-4o (vision-capable) when released

**Disadvantages**:
- **Cost**: ~$0.03 per planning request (~$1-2 for full module completion)
- **Privacy**: Commands sent to OpenAI servers (data retention policy applies)
- **Internet Required**: Fails without connectivity
- **Rate Limits**: 500 requests/minute for paid tier (sufficient for robotics)

**Installation**:

```bash
# Install OpenAI Python SDK
pip install openai

# Verify installation
python -c "import openai; print(openai.__version__)"  # 1.0.0+
```

**API Key Setup**:

1. Sign up at [platform.openai.com](https://platform.openai.com) (requires phone number, credit card)
2. Navigate to **API Keys** → **Create new secret key** → copy `sk-...`
3. Set environment variable:

```bash
# Temporary (current session only)
export OPENAI_API_KEY='sk-your-key-here'

# Permanent (add to ~/.bashrc)
echo "export OPENAI_API_KEY='sk-your-key-here'" >> ~/.bashrc
source ~/.bashrc
```

**First Test**:
```python
from openai import OpenAI
client = OpenAI()  # Auto-uses OPENAI_API_KEY env var

response = client.chat.completions.create(
    model="gpt-4-turbo",
    messages=[{"role": "user", "content": "Decompose: go to the kitchen"}],
    temperature=0
)
print(response.choices[0].message.content)
```

**Cost Estimation** (as of 2024):
- GPT-4 Turbo: $0.01/1K input tokens, $0.03/1K output tokens
- Average planning request: ~500 input + ~200 output tokens
- **Cost per request**: ~$0.011 (500×0.01/1000) + $0.006 (200×0.03/1000) = **$0.017**
- **Full module (100 test commands)**: ~$1.70

### Switching Between LLMs

Both paths use identical code. Switch by changing the `llm_backend` parameter:

```yaml
# config/llm_config.yaml
llm_planner:
  ros__parameters:
    llm_backend: "ollama"  # or "openai"
    model: "llama3.1:8b"   # or "gpt-4-turbo"
    ollama_host: "http://localhost:11434"
    openai_api_key: "${OPENAI_API_KEY}"  # from env var
```

The planner node auto-detects the backend and routes requests accordingly. You can develop with Ollama (free) and deploy with GPT-4 (higher quality) without code changes.

## Action Primitives: The Robot's Vocabulary

An **action primitive** is an atomic robot behavior that cannot be decomposed further—the fundamental building blocks of robot tasks. Just as English sentences are composed of words, robot plans are composed of action primitives. Defining a clear primitive library is critical: too coarse-grained (e.g., `make_sandwich()`), and the LLM lacks flexibility; too fine-grained (e.g., `move_joint_1_by_0.01_radians()`), and planning becomes intractable.

### Core Primitive Set (12 Actions)

Our humanoid uses 12 action primitives covering navigation, perception, and manipulation:

**Navigation Primitives**:
1. **`navigate`**: Move to a named location or coordinates
   - Parameters: `target` (string or [x, y, theta])
   - Example: `{"action": "navigate", "target": "kitchen"}`
   - Maps to: Nav2 `/navigate_to_pose` action server

2. **`stop`**: Halt all motion immediately
   - Parameters: None
   - Emergency brake for safety-critical situations

3. **`follow`**: Track a moving target (person, object)
   - Parameters: `target_class` (string), `distance` (float meters)
   - Example: `{"action": "follow", "target_class": "person", "distance": 2.0}`

**Perception Primitives**:
4. **`detect_object`**: Scan for objects matching a class
   - Parameters: `object_class` (string), `max_results` (int)
   - Example: `{"action": "detect_object", "object_class": "bottle", "max_results": 5}`
   - Maps to: Isaac ROS `/detect_objects` action server
   - Returns: List of detected poses

5. **`scan_environment`**: Perform 360° LIDAR scan
   - Parameters: None
   - Updates occupancy map for navigation

**Manipulation Primitives**:
6. **`grasp`**: Close gripper around detected object
   - Parameters: `object_id` (from detect_object), `force` (Newtons)
   - Example: `{"action": "grasp", "object_id": "bottle_0", "force": 10.0}`
   - Precondition: Object must be within gripper reach (<0.5m)

7. **`release`**: Open gripper and drop held object
   - Parameters: None
   - Precondition: Gripper must be closed

8. **`open_gripper`**: Set gripper to open position
   - Parameters: `width` (meters, default: 0.08)

9. **`close_gripper`**: Set gripper to closed position
   - Parameters: `force` (Newtons, default: 5.0)

10. **`move_arm`**: Move arm to predefined pose
    - Parameters: `pose_name` (string: "home", "ready", "stow")
    - Example: `{"action": "move_arm", "pose_name": "ready"}`

**Utility Primitives**:
11. **`wait`**: Pause for duration
    - Parameters: `duration` (seconds)
    - Example: `{"action": "wait", "duration": 2.0}`
    - Use for: Stabilization after movement, object settling

12. **`say`**: Text-to-speech output
    - Parameters: `text` (string)
    - Example: `{"action": "say", "text": "Task completed"}`
    - Feedback to user during execution

### JSON Schema Format

Each action primitive follows a strict JSON schema for LLM output parsing:

```json
{
  "action": "navigate",           // Action name (enum)
  "parameters": {                 // Action-specific params
    "target": "kitchen"
  },
  "preconditions": [              // Must be true to execute
    "robot_localized",
    "path_exists_to_target"
  ],
  "expected_duration": 15.0       // Seconds (for timeout)
}
```

**Schema Validation**: The planner validates every LLM-generated action against JSON schemas defined in `config/action_primitives.json`. Invalid actions (unknown action name, missing required parameter, out-of-range value) are rejected before execution.

**Example**: Complex task decomposition

**Input**: "Go to the kitchen and bring me a water bottle"

**LLM Output** (JSON action sequence):
```json
[
  {"action": "navigate", "parameters": {"target": "kitchen"}, "expected_duration": 15.0},
  {"action": "detect_object", "parameters": {"object_class": "bottle", "max_results": 1}, "expected_duration": 3.0},
  {"action": "navigate", "parameters": {"target": "detected_object_pose"}, "expected_duration": 5.0},
  {"action": "grasp", "parameters": {"object_id": "bottle_0", "force": 10.0}, "expected_duration": 4.0},
  {"action": "navigate", "parameters": {"target": "user"}, "expected_duration": 15.0},
  {"action": "release", "parameters": {}, "expected_duration": 2.0},
  {"action": "say", "parameters": {"text": "Here is your water bottle"}, "expected_duration": 2.0}
]
```

### ROS 2 Action Server Mapping

Each primitive maps to a ROS 2 action server from previous modules:

| Primitive | ROS 2 Action Server | Module Source |
|-----------|---------------------|---------------|
| `navigate` | `/navigate_to_pose` (nav2_msgs/NavigateToPose) | Module 3 Ch3 (Nav2) |
| `detect_object` | `/detect_objects` (isaac_ros_msgs/DetectObjects) | Module 3 Ch2 (Isaac ROS) |
| `grasp`, `release` | `/gripper_action` (control_msgs/GripperCommand) | New (this chapter) |
| `move_arm` | `/follow_joint_trajectory` (control_msgs/FollowJointTrajectory) | Module 1 Ch2 |
| `say` | `/tts_action` (std_msgs/String) | New (this chapter) |

The `action_executor` node (Chapter 3) subscribes to `/task_plan`, iterates through the action sequence, and dispatches each primitive to the corresponding action server.

## Prompt Engineering: Constraining LLM Outputs

Large language models are trained on general internet text and have no innate understanding of robot constraints. Without careful prompt engineering, an LLM might generate plans like `"fly_to(ceiling)"` or `"make_sandwich()"` (not decomposed into primitives). Your prompt must **constrain** the LLM's outputs to valid action sequences while preserving flexibility for novel tasks.

### System Prompt Template

The system prompt defines the LLM's role and output constraints:

```python
SYSTEM_PROMPT = """You are a task planning AI for a humanoid robot. Your role is to decompose natural language commands into executable action sequences using ONLY the following 12 action primitives:

AVAILABLE ACTIONS:
- navigate(target: str | [x,y,theta])
- stop()
- follow(target_class: str, distance: float)
- detect_object(object_class: str, max_results: int)
- scan_environment()
- grasp(object_id: str, force: float)
- release()
- open_gripper(width: float)
- close_gripper(force: float)
- move_arm(pose_name: str)
- wait(duration: float)
- say(text: str)

CONSTRAINTS:
1. Output ONLY valid JSON (no markdown, no explanations)
2. Use only actions from the list above
3. Decompose complex tasks into step-by-step primitives
4. Ensure preconditions are met (e.g., grasp requires detect_object first)
5. Include expected_duration for each action (seconds)

OUTPUT FORMAT:
[
  {"action": "action_name", "parameters": {...}, "expected_duration": X.X},
  ...
]

If the command is ambiguous or impossible, output:
{"error": "Clarification needed: <specific question>"}
"""
```

**Key Design Decisions**:
- **Enumeration**: List all 12 actions explicitly to prevent hallucinated actions
- **JSON-only**: No markdown code blocks (````json`), no explanatory text—reduces parsing errors
- **Preconditions**: Remind LLM to order actions logically (detect before grasp)
- **Error handling**: Allow LLM to request clarification for ambiguous commands

### Few-Shot Examples

**Few-shot learning** provides the LLM with input-output examples demonstrating the desired behavior. Include 3-5 examples covering simple → complex tasks:

**Example 1** (Simple navigation):
```python
USER: "Go to the kitchen"
ASSISTANT: [
  {"action": "navigate", "parameters": {"target": "kitchen"}, "expected_duration": 15.0}
]
```

**Example 2** (Perception + manipulation):
```python
USER: "Pick up the red cube"
ASSISTANT: [
  {"action": "detect_object", "parameters": {"object_class": "cube", "max_results": 5}, "expected_duration": 3.0},
  {"action": "navigate", "parameters": {"target": "detected_object_pose"}, "expected_duration": 5.0},
  {"action": "grasp", "parameters": {"object_id": "cube_0", "force": 8.0}, "expected_duration": 4.0}
]
```

**Example 3** (Complex multi-step):
```python
USER: "Bring me the bottle from the table"
ASSISTANT: [
  {"action": "navigate", "parameters": {"target": "table"}, "expected_duration": 10.0},
  {"action": "detect_object", "parameters": {"object_class": "bottle", "max_results": 1}, "expected_duration": 3.0},
  {"action": "grasp", "parameters": {"object_id": "bottle_0", "force": 10.0}, "expected_duration": 4.0},
  {"action": "navigate", "parameters": {"target": "user"}, "expected_duration": 10.0},
  {"action": "say", "parameters": {"text": "Here is your bottle"}, "expected_duration": 2.0}
]
```

**Example 4** (Ambiguous command):
```python
USER: "Clean the room"
ASSISTANT: {"error": "Clarification needed: What specific cleaning task? (pick up objects, vacuum, wipe surfaces)"}
```

### Temperature and Sampling

**Temperature** controls LLM output randomness (0.0 = deterministic, 1.0 = creative):

```python
response = client.chat.completions.create(
    model="gpt-4-turbo",
    messages=[...],
    temperature=0,  # Deterministic planning (same input → same output)
    max_tokens=500,  # Limit response length
    top_p=1.0  # Nucleus sampling (1.0 = disabled)
)
```

**For robot planning, use temperature=0** to ensure consistent, repeatable plans. High temperature introduces variability that complicates debugging and validation.

### JSON Parsing and Error Handling

LLMs occasionally output malformed JSON (missing brackets, trailing commas). Implement robust parsing:

```python
import json

def parse_llm_response(response_text: str) -> list:
    """Parse LLM response with error recovery."""
    try:
        # Remove markdown code blocks if present
        text = response_text.strip()
        if text.startswith("```"):
            text = text.split("```")[1]
            if text.startswith("json"):
                text = text[4:]  # Remove "json" prefix

        plan = json.loads(text)

        # Validate structure
        if isinstance(plan, dict) and "error" in plan:
            raise ValueError(f"LLM requested clarification: {plan['error']}")
        if not isinstance(plan, list):
            raise ValueError("Plan must be a JSON array")

        return plan
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON from LLM: {e}")
```

## Plan Validation: Safety Before Execution

Even with perfect prompts, LLMs can generate invalid plans (unreachable targets, missing preconditions, parameter out of bounds). **Never execute unvalidated plans**—validate against robot capabilities and environment state.

### Validation Checks

**1. Action Name Validation**:
```python
VALID_ACTIONS = {"navigate", "stop", "follow", "detect_object", "scan_environment",
                 "grasp", "release", "open_gripper", "close_gripper", "move_arm", "wait", "say"}

for action in plan:
    if action["action"] not in VALID_ACTIONS:
        raise ValueError(f"Unknown action: {action['action']}")
```

**2. Parameter Validation** (from JSON schema):
```python
# Example: navigate requires "target" parameter
if action["action"] == "navigate":
    if "target" not in action["parameters"]:
        raise ValueError("navigate missing required parameter: target")

    target = action["parameters"]["target"]
    if isinstance(target, str):
        if target not in known_locations:  # ["kitchen", "bedroom", "table", ...]
            raise ValueError(f"Unknown location: {target}")
```

**3. Precondition Checking**:
```python
# Example: grasp requires prior detect_object
if action["action"] == "grasp":
    # Check if any previous action was detect_object
    has_detection = any(a["action"] == "detect_object" for a in plan[:i])
    if not has_detection:
        raise ValueError("grasp requires detect_object first")
```

**4. Parameter Range Validation**:
```python
# Example: gripper force must be 0-50 Newtons
if action["action"] == "grasp":
    force = action["parameters"]["force"]
    if not (0 <= force <= 50):
        raise ValueError(f"Invalid grasp force: {force} (must be 0-50N)")
```

### Retry Logic for Invalid Plans

If validation fails, retry with an augmented prompt:

```python
def plan_task_with_retry(command: str, max_retries: int = 3) -> list:
    """Generate and validate task plan with retries."""
    for attempt in range(max_retries):
        try:
            plan = call_llm(command)
            validate_plan(plan)
            return plan
        except ValueError as e:
            if attempt == max_retries - 1:
                raise  # Give up after max retries
            # Retry with error feedback
            command += f"\nPrevious attempt failed: {e}. Please fix and try again."

    raise ValueError("Max retries exceeded")
```

### Fallback to Cached Plans

For common commands, maintain a cache of pre-validated plans:

```python
CACHED_PLANS = {
    "stop": [{"action": "stop", "parameters": {}, "expected_duration": 0.5}],
    "go home": [{"action": "navigate", "parameters": {"target": "home"}, "expected_duration": 20.0}],
    # ... more common commands
}

def get_plan(command: str) -> list:
    """Get plan from cache or LLM."""
    if command.lower() in CACHED_PLANS:
        return CACHED_PLANS[command.lower()]
    return plan_task_with_retry(command)
```

## Runnable Example

Download and test the complete LLM planner:

**[📥 Download: llm_task_planner.zip](/examples/llm_task_planner.zip)**

### Build and Launch

```bash
# Extract package
cd ~/ros2_ws/src
unzip llm_task_planner.zip

# Build
cd ~/ros2_ws
colcon build --packages-select llm_planner

# Source
source install/setup.bash

# Launch planner (Ollama backend)
ros2 launch llm_planner llm_planner.launch.py llm_backend:=ollama

# Or launch with GPT-4 (requires OPENAI_API_KEY env var)
ros2 launch llm_planner llm_planner.launch.py llm_backend:=openai model:=gpt-4-turbo
```

### Test Commands

Terminal 1 (planner):
```bash
ros2 launch llm_planner llm_planner.launch.py
```

Terminal 2 (publish voice command):
```bash
# Simple command
ros2 topic pub --once /voice_commands std_msgs/String "data: 'go to the kitchen'"

# Complex command
ros2 topic pub --once /voice_commands std_msgs/String "data: 'find the red bottle and bring it to me'"
```

Terminal 3 (view generated plan):
```bash
ros2 topic echo /task_plan
```

**Expected output** (for "go to the kitchen"):
```json
{
  "plan": [
    {
      "action": "navigate",
      "parameters": {"target": "kitchen"},
      "expected_duration": 15.0
    }
  ],
  "total_duration": 15.0,
  "llm_model": "llama3.1:8b",
  "generation_time": 3.2
}
```

## Practice Exercises

1. **Add Custom Action Primitive** (Easy, 30min)
   - Define a new `inspect_object` action that moves the camera to focus on a detected object
   - Update `action_primitives.json`, system prompt, and validator
   - Test: "inspect the red cube"

2. **Implement Retry Logic** (Medium, 1hr)
   - Add retry mechanism for failed grasps (max 3 attempts)
   - Log each retry attempt with timestamps
   - Test with intentionally failed grasps

3. **Handle Ambiguous Commands** (Medium, 1hr)
   - Test commands like "clean up" or "help me"
   - Implement clarification request publishing to `/clarification_request` topic
   - Test LLM's ability to ask for details

4. **Measure Planning Latency** (Easy, 30min)
   - Add timing instrumentation to `planner_node.py`
   - Log latency for 10 simple vs 10 complex commands
   - Compare Ollama vs GPT-4 (if you have API key)

5. **Compare GPT-4 vs Llama Quality** (Medium, 1hr)
   - Test same 20 commands with both backends
   - Measure: valid JSON rate, action sequence correctness, latency
   - Document findings in a table

6. **Visualize Plans in RViz** (Hard, 2hrs)
   - Create RViz plugin to display action sequences as timeline
   - Show current action, progress bar, expected vs actual duration
   - Publish plan visualization markers

## Troubleshooting

### Issue 1: LLM generates invalid JSON

**Symptoms**: `JSONDecodeError: Expecting value: line 1 column 1`

**Causes**:
- LLM output includes explanatory text ("Here's the plan: ...")
- Markdown code blocks (````json ... ````)
- Trailing commas, missing brackets

**Solutions**:
```python
# 1. Strengthen system prompt
"Output ONLY valid JSON. No markdown, no explanations, no code blocks."

# 2. Strip markdown in parser (see JSON Parsing section above)

# 3. Use temperature=0 for deterministic output
```

### Issue 2: OpenAI API rate limit exceeded

**Symptoms**: `openai.RateLimitError: Rate limit reached for requests`

**Solutions**:
```python
# 1. Implement exponential backoff
import time
from openai import RateLimitError

for attempt in range(5):
    try:
        return client.chat.completions.create(...)
    except RateLimitError:
        wait_time = 2 ** attempt  # 1s, 2s, 4s, 8s, 16s
        time.sleep(wait_time)

# 2. Switch to Ollama (no rate limits)
llm_backend: "ollama"

# 3. Upgrade OpenAI tier (requires payment history)
```

### Issue 3: Ollama connection refused

**Symptoms**: `requests.exceptions.ConnectionError: [Errno 111] Connection refused`

**Causes**:
- Ollama server not running
- Wrong port (should be 11434)

**Solutions**:
```bash
# 1. Check if Ollama is running
curl http://localhost:11434/api/tags

# 2. Start Ollama server
ollama serve

# 3. Verify model is pulled
ollama list  # Should show llama3.1:8b

# 4. Pull model if missing
ollama pull llama3.1:8b
```

### Issue 4: Plan execution timeout

**Symptoms**: Action takes longer than `expected_duration`, executor aborts

**Causes**:
- Underestimated duration (navigate in large space)
- Robot stuck (obstacle blocking path)

**Solutions**:
```python
# 1. Increase duration estimates in prompt examples
{"action": "navigate", "expected_duration": 30.0}  # Was 15.0

# 2. Add timeout buffer in executor (Chapter 3)
timeout = action["expected_duration"] * 1.5  # 50% buffer

# 3. Implement progress monitoring (cancel if no progress for 5s)
```

## References

1. **Ahn, M., et al. (2022)**. "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances." *Conference on Robot Learning (CoRL)*. [Paper](https://arxiv.org/abs/2204.01691)
   - Introduced SayCan, grounding LLMs in robot value functions for feasible task planning

2. **Liang, J., et al. (2022)**. "Code as Policies: Language Model Programs for Embodied Control." *arXiv:2209.07753*. [Paper](https://arxiv.org/abs/2209.07753)
   - Demonstrated LLMs generating executable robot code from natural language

3. **Singh, I., et al. (2023)**. "ProgPrompt: Generating Situated Robot Task Plans using Large Language Models." *IEEE International Conference on Robotics and Automation (ICRA)*. [Paper](https://arxiv.org/abs/2209.11302)
   - Advanced prompt engineering for LLM-based robot planning with spatial reasoning

---

**Next Chapter**: [Chapter 3 - Capstone: Autonomous Humanoid VLA](./chapter-3-capstone-vla.md) integrates voice commands, LLM planning, navigation, vision, and manipulation into a complete autonomous system. You'll deploy the full VLA pipeline in Isaac Sim and execute end-to-end tasks like "bring me the bottle from the shelf" autonomously in <2 minutes.
