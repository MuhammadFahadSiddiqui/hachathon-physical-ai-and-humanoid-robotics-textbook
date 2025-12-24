---
title: "Module 4: Vision-Language-Action (VLA)"
sidebar_position: 5
id: index
---

# Module 4: Vision-Language-Action (VLA)

Welcome to Module 4, where you'll master cutting-edge Vision-Language-Action systems that enable humanoid robots to understand natural language commands and execute complex tasks autonomously. This module builds on your ROS 2 fundamentals (Module 1), simulation skills (Module 2), and AI perception capabilities (Module 3) to teach you how large language models (LLMs) are revolutionizing robot control.

## Learning Objectives

By the end of this module, you will:

1. **Implement voice command recognition** using OpenAI Whisper for hands-free robot control with >90% transcription accuracy
2. **Translate natural language to robot actions** using LLMs (GPT-4 or local Llama 3.1) to decompose complex tasks into executable primitives
3. **Build an end-to-end autonomous system** integrating voice, planning, navigation, vision, and manipulation in a capstone demonstration
4. **Understand VLA research** from leading papers (RT-2, SayCan, PaLM-E) bridging AI and robotics

## Prerequisites

### Required Knowledge
- **Module 1**: ROS 2 nodes, topics, action servers, launch files
- **Module 2**: Gazebo or Isaac Sim simulation environments
- **Module 3**: Nav2 navigation and/or Isaac ROS perception
- **Python**: Advanced proficiency (async/await, decorators, type hints)
- **Linux**: Terminal commands, environment variables, package management

### Hardware Requirements

**Microphone Required**

This module requires a microphone for voice command recognition:

- **Built-in laptop microphone**: Sufficient for testing (16kHz+ sampling rate)
- **USB microphone**: Recommended for production use (noise isolation, better accuracy)
- **Supported OS**: Ubuntu 22.04 LTS (native or WSL2), macOS 12+, Windows 10+ with WSL2

**LLM Inference Options**

You have **two options** for LLM-based planning - choose based on your budget and performance needs:

#### Option 1: Local Inference (FREE) ✅ Recommended for Students

Run Llama 3.1 8B model locally using Ollama:

- **Hardware**: 16GB+ RAM (24GB+ recommended), CPU only (GPU optional but faster)
- **Cost**: $0 (completely free, runs on your machine)
- **Latency**: 5-10s for simple plans, 15-20s for complex tasks (CPU), 2-5s with GPU
- **Quality**: Very good for educational use, supports multi-step reasoning
- **Privacy**: All processing local, no API calls

**Setup**:
```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llama3.1:8b
```

#### Option 2: Cloud API (PAID) ⚡ Best Performance

Use OpenAI GPT-4 API for production-grade quality:

- **Cost**: ~$0.03 per planning request (~$1-2 for full module completion)
- **Latency**: `<5s` for simple plans, `<15s` for complex tasks
- **Quality**: Excellent, state-of-the-art reasoning
- **Requirements**: OpenAI API key (signup at [platform.openai.com](https://platform.openai.com))

**Both options are fully supported in all chapters** - code examples work with either backend.

### Software Requirements

All software is **free and open-source** (except optional GPT-4 API):

- **Ubuntu 22.04 LTS** (native, WSL2, or dual-boot)
- **ROS 2 Humble**: From Module 1 setup
- **Python 3.10+**: System default on Ubuntu 22.04
- **OpenAI Whisper**: Free MIT license (pip install openai-whisper)
- **Ollama**: Free MIT license (local LLM inference)
- **Python Libraries**: sounddevice, rclpy, openai (all free/open-source)

## Module Structure

### Chapter 1: Voice-to-Action with OpenAI Whisper

Learn to implement voice command recognition for natural language robot control.

**Topics**:
- Whisper model selection (base 74M vs small 244M tradeoff)
- ROS 2 integration with audio capture (sounddevice)
- Confidence thresholding and noise filtering
- Publishing transcriptions to /voice_commands topic

**Runnable Example**: Speak "move forward two meters" → see transcription published to ROS 2 within 2s with >90% accuracy

**[📥 Download: whisper_voice_commands.zip](/examples/whisper_voice_commands.zip)**

---

### Chapter 2: Cognitive Planning with LLMs

Translate natural language into executable ROS 2 action sequences using large language models.

**Topics**:
- LLM setup (GPT-4 API vs Llama 3.1 via Ollama)
- Action primitive library design (navigate, grasp, detect_object, etc.)
- Prompt engineering for robot task decomposition
- Plan validation and error handling

**Runnable Example**: Send "go to the kitchen and bring me a water bottle" → LLM outputs JSON action sequence [{navigate, detect_object, grasp, navigate}] in `<5s`

**Cost Comparison**: Ollama (free) vs GPT-4 (~$0.03/request)

**[📥 Download: llm_task_planner.zip](/examples/llm_task_planner.zip)**

---

### Chapter 3: Capstone - Autonomous Humanoid VLA

Integrate voice, LLM planning, navigation, vision, and manipulation into a complete autonomous system.

**Topics**:
- Behavior coordination (state machine for action execution)
- Module integration (Whisper → LLM → Nav2 → Isaac ROS → Gripper)
- Failure handling and replanning
- RViz visualization for debugging (voice commands, task plans, execution status)

**Runnable Example**: Speak "bring me the bottle from the shelf" → humanoid completes full 6-step task autonomously in `<2min`

**[📥 Download: vla_capstone_demo.zip](/examples/vla_capstone_demo.zip)**

---

## Practice Exercises

Each chapter includes 4-6 hands-on exercises to deepen your understanding:

- **Chapter 1**: Add custom voice commands, implement wake word detection, tune confidence thresholds, test across accents, add logging, integrate text-to-speech feedback
- **Chapter 2**: Add custom action primitives, implement retry logic, handle ambiguous commands, measure planning latency, compare GPT-4 vs Llama quality, visualize plans in RViz
- **Chapter 3**: Add force feedback logging, implement multi-object tasks, test obstacle avoidance recovery, record demo video, test system resilience, tune state machine timeouts

**Total**: 12-18 practical exercises across all chapters

## Academic Rigor

This module cites **6+ peer-reviewed papers** from top AI and robotics venues:

- **CoRL** (Conference on Robot Learning)
- **RSS** (Robotics: Science and Systems)
- **ICRA** (International Conference on Robotics and Automation)
- **NeurIPS** (Neural Information Processing Systems)
- **T-RO** (IEEE Transactions on Robotics)
- **Science Robotics** (Top-tier journal)

Key papers covered:
- **RT-2** (DeepMind 2023): Vision-language-action models
- **SayCan** (Google 2022): Grounding language in robotic affordances
- **PaLM-E** (Google 2023): Embodied multimodal language models
- **Code as Policies** (Liang et al. 2022): LLM-based robot planning

All content grounded in current research (2020-2024) to ensure state-of-the-art techniques.

## Troubleshooting & Support

Each chapter includes dedicated troubleshooting sections addressing common issues:

- **Whisper**: Microphone detection failures, high latency (>5s), low accuracy (`<70%`), ROS 2 node errors
- **LLM Planning**: Invalid JSON generation, OpenAI API rate limits, Ollama connection refused, plan execution timeouts
- **Capstone**: Action primitive timeouts, gripper grasp failures, system stuck in planning state, RViz visualization issues

**Community Support**:
- [OpenAI Whisper GitHub](https://github.com/openai/whisper)
- [Ollama Docs](https://ollama.com/docs)
- [OpenAI Platform Forum](https://community.openai.com)
- [ROS 2 Discourse](https://discourse.ros.org)

## What's Next?

Ready to build voice-controlled autonomous humanoids? Start with:

1. **[Chapter 1: Voice-to-Action](./chapter-1-voice-to-action.md)** - Implement Whisper-based voice command recognition
2. **[Chapter 2: Cognitive Planning](./chapter-2-cognitive-planning.md)** - Translate natural language to robot actions with LLMs
3. **[Chapter 3: Capstone VLA](./chapter-3-capstone-vla.md)** - End-to-end autonomous humanoid demonstration

**Estimated Time**: 10-15 hours total (3-5 hours per chapter including exercises)

---

**Cost Disclaimer**: This module can be completed **entirely for free** using Ollama for local LLM inference and OpenAI Whisper (MIT license). OpenAI GPT-4 API is an optional paid upgrade (~$0.03/request) for production-grade quality. All code examples support both free and paid options.
