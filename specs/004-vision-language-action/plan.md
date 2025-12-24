# Implementation Plan: Module 4 - Vision-Language-Action (VLA)

**Branch**: `004-vision-language-action` | **Date**: 2024-12-24 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/004-vision-language-action/spec.md`

## Summary

Create Module 4 educational content covering Vision-Language-Action (VLA) systems for autonomous humanoid robots. The module consists of 3 technical chapters (2500-4000 words each) as Markdown files in Docusaurus format, covering OpenAI Whisper voice commands, LLM-based cognitive planning (GPT-4/Llama), and an end-to-end capstone project integrating voice, planning, navigation, vision, and manipulation. Includes runnable ROS 2 packages (.zip downloads), practice exercises, peer-reviewed citations (CoRL, RSS, ICRA, NeurIPS), and troubleshooting sections. Target audience: students who have completed Modules 1-3 and have access to microphone hardware and either OpenAI API key or local Llama inference capability.

## Technical Context

**Content Format**: Markdown (.md) with Docusaurus frontmatter (YAML metadata)
**Primary Technologies**: OpenAI Whisper (base/small model), ROS 2 Humble, OpenAI GPT-4 API or Llama 3.1 via Ollama, Python 3.10+
**Target Platform**: Ubuntu 22.04 LTS with microphone (16kHz+ sampling), 16GB+ RAM for local Llama inference
**Example Code Languages**: Python 3.10+ (ROS 2 nodes, LLM API clients, behavior trees), JSON (action primitive schemas), YAML (launch files)
**Testing**: Manual validation with GPT-4 API and Llama 3.1 8B model on Ubuntu 22.04, microphone transcription testing
**Performance Goals**: Whisper transcription <2s latency, >90% accuracy; LLM planning <5s (simple), <15s (complex); end-to-end task <2min
**Constraints**:
- 2500-4000 words per chapter
- 2+ peer-reviewed citations per chapter (CoRL/RSS/ICRA/NeurIPS/T-RO/Science Robotics 2020+)
- All software must be free/open-source (Whisper MIT license, Ollama open-source) OR use free API tiers (OpenAI pay-per-use)
- English language only (multilingual out of scope)
**Scale/Scope**: 3 chapters, 4-6 module landing page, 12-18 practice exercises, 3 downloadable .zip archives (<100 MB each)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Spec-Driven, AI-First Development

✅ **PASS**: Feature was specified via `/sp.specify` with complete user stories (3 prioritized: P1 voice interface, P1 LLM planning, P2 capstone), requirements (20 FRs), and success criteria (10 SCs). Planning via `/sp.plan` in progress. Tasks will be generated via `/sp.tasks` before implementation.

### Principle II: Accuracy and Content-Grounded Responses

✅ **PASS** (adapted): While this principle applies to RAG chatbot behavior, the equivalent for educational VLA content is **technical accuracy**. All Whisper, LLM, and ROS 2 content will be sourced from official documentation (OpenAI API docs, Ollama docs, ROS 2 Humble docs) and peer-reviewed VLA research papers (RT-2, PaLM-E, SayCan), not speculation or outdated examples.

### Principle III: Clear, Developer-Focused Writing

✅ **PASS**: Spec defines success criteria for clarity (SC-008: 75% exercise completion rate, SC-009: 85% positive feedback on LLM API setup). Each chapter includes:
- Conceptual explanations of VLA architecture (voice → LLM → action primitives)
- Runnable code examples (Whisper ROS 2 node, LLM planner, behavior coordinator)
- Step-by-step tutorials for Whisper installation, LLM API setup (GPT-4 + Ollama), action primitive library
- Troubleshooting sections addressing microphone detection, LLM API errors, JSON parsing failures

### Principle IV: Reproducible and Production-Ready Setup

✅ **PASS**: All examples will be tested with GPT-4 API and Llama 3.1 8B baseline. Downloadable .zip archives include pre-configured ROS 2 packages, action primitive JSON schemas, and example prompts. Installation instructions validated on clean Ubuntu 22.04 environments. Environment variables (OPENAI_API_KEY, OLLAMA_HOST) documented in .env.example files.

### Principle V: Free-Tier and Open-Source Compliant

✅ **PASS**:
- **OpenAI Whisper**: Open-source (MIT license), installable via pip
- **Ollama**: Open-source local LLM inference (MIT license)
- **Llama 3.1**: Open-source model weights (Meta Llama 3 license, free for research/commercial use)
- **ROS 2 Humble**: Open-source (Apache 2.0)
- **Python Libraries**: openai (Apache 2.0), rclpy (Apache 2.0), sounddevice (MIT)

⚠️ **OPTIONAL PAID SERVICE DISCLAIMER**: OpenAI GPT-4 API is a **paid service** (~$0.03 per planning request), but this is offered as **one of two options**:
1. **Free Option**: Llama 3.1 8B via Ollama (fully open-source, runs locally, zero API cost)
2. **Paid Option**: OpenAI GPT-4 API (higher quality, lower latency, pay-per-use)

Students can complete all chapters using Ollama (free). GPT-4 is optional for those seeking production-grade performance. This aligns with **Principle V** (free-tier compliant) because the free path is viable and documented as the default.

**Re-check after Phase 1**: Verify no additional paid services introduced during research/design phase.

## Project Structure

### Documentation (this feature)

```text
specs/004-vision-language-action/
├── plan.md              # This file (/sp.plan output)
├── research.md          # Phase 0: Whisper/LLM research, VLA papers
├── data-model.md        # Phase 1: Learning outcome model, chapter structure
├── quickstart.md        # Phase 1: Module 4 setup quick reference (Whisper + LLM)
├── contracts/           # Phase 1: Example schemas (action primitives JSON, ROS 2 msg types)
└── tasks.md             # Phase 2: /sp.tasks output (implementation checklist)
```

### Content Files (repository)

```text
frontend_book/docs/module-4/
├── index.md                          # Module 4 landing page (learning objectives, LLM API prerequisites)
├── chapter-1-voice-to-action.md      # Chapter 1: OpenAI Whisper voice commands (2500-4000 words)
├── chapter-2-cognitive-planning.md   # Chapter 2: LLM task planning (2500-4000 words)
└── chapter-3-capstone-vla.md         # Chapter 3: End-to-end capstone (2500-4000 words)

frontend_book/static/examples/
├── whisper_voice_commands/           # Whisper ROS 2 example (exported to .zip)
│   ├── whisper_ros2/
│   │   ├── whisper_ros2/
│   │   │   ├── __init__.py
│   │   │   └── voice_command_node.py  # Whisper transcription ROS 2 node
│   │   ├── package.xml
│   │   └── setup.py
│   ├── launch/
│   │   └── voice_interface.launch.py  # Launch Whisper node
│   ├── config/
│   │   ├── voice_commands.yaml        # Example commands list
│   │   └── .env.example               # Microphone device config
│   └── README.md                      # Installation, microphone setup, testing
│
├── llm_task_planner/                 # LLM planner example (exported to .zip)
│   ├── llm_planner/
│   │   ├── llm_planner/
│   │   │   ├── __init__.py
│   │   │   ├── planner_node.py        # LLM API client ROS 2 node
│   │   │   ├── action_primitives.py   # Action primitive library (8-12 actions)
│   │   │   └── prompt_templates.py    # System prompts, few-shot examples
│   │   ├── package.xml
│   │   └── setup.py
│   ├── launch/
│   │   └── llm_planner.launch.py      # Launch LLM planner node
│   ├── config/
│   │   ├── action_primitives.json     # Action primitive JSON schema
│   │   ├── .env.example               # OPENAI_API_KEY, OLLAMA_HOST
│   │   └── llm_config.yaml            # Model selection (gpt-4 or llama3.1)
│   └── README.md                      # API setup (GPT-4 + Ollama), prompt engineering
│
└── vla_capstone_demo/                # Capstone integration (exported to .zip)
    ├── vla_coordinator/
    │   ├── vla_coordinator/
    │   │   ├── __init__.py
    │   │   ├── behavior_coordinator.py # State machine / behavior tree
    │   │   ├── action_executor.py      # Action primitive execution
    │   │   └── system_monitor.py       # Progress tracking, error handling
    │   ├── package.xml
    │   └── setup.py
    ├── launch/
    │   └── capstone_demo.launch.py    # Launch all nodes (Whisper, LLM, Nav2, Isaac ROS, gripper)
    ├── config/
    │   ├── system_config.yaml         # Full system parameters
    │   └── rviz_vla_debug.rviz        # RViz config (voice commands, task plan, execution status)
    ├── urdf/
    │   └── humanoid_gripper.urdf      # simple_humanoid.urdf with gripper added
    └── README.md                      # End-to-end demo instructions, video recording guide
```

**Structure Decision**: Educational content creation following Module 1-3 patterns:
- Chapter Markdown files in `frontend_book/docs/module-4/`
- Example ROS 2 packages in `frontend_book/static/examples/` (separate .zip per chapter)
- Each example includes README with setup (API keys, microphone), troubleshooting, validation
- .zip archives created from example directories for user download

## Complexity Tracking

> No Constitution violations. All checks passed. Free-tier option (Ollama) available as default; paid GPT-4 API is optional enhancement. This section intentionally left empty.

---

## Phase 0: Research & Technology Decisions

**Objective**: Resolve all unknowns from Technical Context, research best practices for Whisper/LLM/VLA integration, consolidate findings in `research.md`.

### Research Tasks

1. **Whisper Installation and Model Selection**
   - **Unknown**: Optimal Whisper model size (tiny/base/small) for robot command accuracy vs. latency tradeoff
   - **Research**: OpenAI Whisper GitHub repo, model benchmark comparisons (WER, RTF on CPU)
   - **Outcome**: Document recommended model (base 74M parameters for balance), installation via pip, CPU vs GPU inference

2. **Whisper ROS 2 Integration Patterns**
   - **Unknown**: Best practices for audio input in ROS 2 (sounddevice vs PyAudio vs audio_common_msgs)
   - **Research**: ROS 2 audio packages (audio_common), Python audio libraries, real-time streaming approaches
   - **Outcome**: Document microphone input approach (sounddevice for simplicity), publish transcriptions as std_msgs/String

3. **LLM API Client Setup (GPT-4 vs Llama 3.1)**
   - **Unknown**: OpenAI Python SDK usage patterns, Ollama REST API endpoints, cost comparison
   - **Research**: OpenAI API documentation (Chat Completions), Ollama API docs, LLM pricing (GPT-4 Turbo vs free Llama)
   - **Outcome**: Document dual-path setup (openai library for GPT-4, requests library for Ollama), cost estimate ($0.03/request GPT-4)

4. **Action Primitive Library Design**
   - **Unknown**: JSON schema structure for action primitives, parameter validation approaches
   - **Research**: ROS 2 action server best practices, BehaviorTree.CPP action formats, Google Robotics SayCan paper
   - **Outcome**: Define 8-12 action primitives (navigate, detect_object, grasp, release, etc.) with JSON schema (type, parameters, preconditions)

5. **LLM Prompt Engineering for Robot Planning**
   - **Unknown**: System prompt structure, few-shot examples for task decomposition, JSON output formatting
   - **Research**: OpenAI prompt engineering guide, Code as Policies (2022), ProgPrompt (2023) papers
   - **Outcome**: Document system prompt template, 3-5 few-shot examples (simple → complex tasks), temperature=0 for deterministic output

6. **Behavior Coordination Architecture**
   - **Unknown**: State machine vs behavior tree for action sequencing, ROS 2 action server preemption
   - **Research**: ROS 2 action tutorials, BehaviorTree.CPP library, SMACH state machine patterns
   - **Outcome**: Decide on simple Python state machine for MVP (defer BehaviorTree.CPP to advanced topic), document preemption handling

7. **Peer-Reviewed VLA Paper Selection**
   - **Unknown**: Specific papers for VLA systems (Chapter 1), LLM planning (Chapter 2), speech interfaces (Chapter 3)
   - **Research**: Google Scholar, arXiv searches for "vision language action robotics", "LLM task planning", "speech interfaces HRI"
   - **Outcome**: Select 2+ papers per chapter:
     - Chapter 1: RT-2 (DeepMind 2023), Whisper (OpenAI 2022)
     - Chapter 2: SayCan (Google 2022), Code as Policies (Liang et al. 2022)
     - Chapter 3: PaLM-E (Google 2023), multimodal robot learning papers

8. **Microphone Hardware Requirements**
   - **Unknown**: Minimum sampling rate, USB vs built-in microphone recommendations, noise cancellation
   - **Research**: Whisper audio preprocessing requirements (16kHz minimum), USB microphone best practices
   - **Outcome**: Document minimum specs (16kHz, mono sufficient), recommend USB for noise isolation, test with built-in laptop mic

9. **Integration with Module 3 (Isaac ROS + Nav2)**
   - **Unknown**: How to connect LLM planner outputs to Nav2 goals and Isaac ROS perception inputs
   - **Research**: ROS 2 topic remapping, Nav2 SimpleGoal API, Isaac ROS detection message formats
   - **Outcome**: Document topic connections (/task_plan → /navigate_to_pose, /detected_objects → /target_object_pose)

10. **Capstone Demo Complexity Management**
    - **Unknown**: Incremental milestones to avoid overwhelming students with full system complexity
    - **Research**: Educational scaffolding best practices, ROS 2 launch file composition
    - **Outcome**: Define 4 milestones: (1) voice only, (2) voice → LLM, (3) LLM → single action, (4) full multi-action sequence

### Research Agent Dispatch

For each research task above, launch specialized research agents or perform direct documentation lookups:

- **Whisper**: Review OpenAI Whisper GitHub (https://github.com/openai/whisper), model card documentation
- **LLM APIs**: Review OpenAI Platform docs (https://platform.openai.com/docs), Ollama docs (https://ollama.com/docs)
- **VLA Papers**: Search arXiv for "RT-2", "SayCan", "PaLM-E", "Code as Policies", "ProgPrompt"
- **ROS 2 Actions**: Review ROS 2 action tutorials (https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client.html)

**Output**: `research.md` with consolidated findings, decisions, and rationale for all unknowns.

---

## Phase 1: Design & Content Outline

**Prerequisites**: `research.md` complete with all unknowns resolved.

### 1. Extract Learning Outcomes → `data-model.md`

Instead of software entities, Module 4 has **learning outcomes** as the "data model":

**Learning Outcome Model**:
- **Outcome ID**: LO-M4-### (e.g., LO-M4-001)
- **Chapter**: Voice-to-Action / Cognitive Planning / Capstone
- **Description**: What students will be able to do (e.g., "Install Whisper and transcribe voice commands")
- **Prerequisites**: Prior learning outcomes (Module 1 ROS 2 nodes, Module 2 simulation, Module 3 perception/navigation)
- **Validation Method**: Practice exercise, success criteria metric (e.g., SC-001, SC-002)
- **Code Examples**: Python ROS 2 nodes, JSON schemas, launch files

**Chapter Structure Model**:
- **Chapter ID**: CH-M4-#
- **Title**: (e.g., "Chapter 1: Voice-to-Action with OpenAI Whisper")
- **Sections**: Introduction, Installation, ROS 2 Integration, Examples, Practice Exercises, Troubleshooting, Key Takeaways
- **Word Count**: 2500-4000 words
- **Code Examples**: 2-3 runnable examples per chapter
- **Practice Exercises**: 4-6 exercises
- **Citations**: 2+ peer-reviewed papers

**Action Primitive Entity** (for Chapter 2):
- **Primitive Name**: navigate, detect_object, grasp, release, open_gripper, close_gripper, move_joint, wait
- **Parameters**: JSON dict (e.g., {target: "kitchen"}, {object: "cup"})
- **Preconditions**: List of required states (e.g., navigation requires map loaded)
- **Expected Duration**: Seconds (for timeout handling)
- **ROS 2 Action Server**: Mapping to /navigate_to_pose, /detect_objects, /gripper_action

### 2. Generate Content Contracts → `/contracts/`

**Contract 1: Voice Command Message** (`voice_command.json`)
```json
{
  "type": "object",
  "properties": {
    "text": {"type": "string"},
    "confidence": {"type": "number", "minimum": 0, "maximum": 1},
    "timestamp": {"type": "string", "format": "date-time"},
    "speaker_id": {"type": "string"}
  },
  "required": ["text", "confidence", "timestamp"]
}
```

**Contract 2: Action Primitive Schema** (`action_primitive.json`)
```json
{
  "type": "object",
  "properties": {
    "action": {"type": "string", "enum": ["navigate", "detect_object", "grasp", "release", "open_gripper", "close_gripper", "move_joint", "wait"]},
    "parameters": {"type": "object"},
    "preconditions": {"type": "array", "items": {"type": "string"}},
    "expected_duration": {"type": "number"}
  },
  "required": ["action", "parameters"]
}
```

**Contract 3: Task Plan Message** (`task_plan.json`)
```json
{
  "type": "object",
  "properties": {
    "plan_id": {"type": "string", "format": "uuid"},
    "actions": {"type": "array", "items": {"$ref": "#/definitions/ActionPrimitive"}},
    "total_estimated_time": {"type": "number"},
    "confidence_score": {"type": "number", "minimum": 0, "maximum": 1}
  },
  "required": ["plan_id", "actions"]
}
```

**Contract 4: Execution Status Message** (`execution_status.json`)
```json
{
  "type": "object",
  "properties": {
    "current_action_index": {"type": "integer"},
    "action_state": {"type": "string", "enum": ["idle", "executing", "succeeded", "failed"]},
    "error_message": {"type": "string"},
    "progress_percentage": {"type": "number", "minimum": 0, "maximum": 100}
  },
  "required": ["current_action_index", "action_state", "progress_percentage"]
}
```

### 3. Create Quickstart Guide → `quickstart.md`

**Content**:
- Prerequisites: Modules 1-3 completion, microphone hardware, Python 3.10+
- Whisper installation: `pip install openai-whisper sounddevice`
- LLM setup:
  - **Option 1 (Free)**: `curl -fsSL https://ollama.com/install.sh | sh && ollama pull llama3.1:8b`
  - **Option 2 (Paid)**: OpenAI API key signup, `pip install openai`, set OPENAI_API_KEY
- Microphone test: `python -m sounddevice` to list devices
- Clone example packages: Download .zip files, extract to `~/ros2_ws/src/`
- Build workspace: `colcon build --symlink-install`
- Run Chapter 1 example: `ros2 launch whisper_ros2 voice_interface.launch.py`
- Verify transcription: `ros2 topic echo /voice_commands`

### 4. Agent Context Update

Run `.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude` to add:
- Whisper (OpenAI speech recognition)
- OpenAI GPT-4 API (LLM planning)
- Ollama (local LLM inference)
- Action primitives (robot task decomposition)
- Behavior coordination (state machine for VLA)

**Output**: data-model.md, /contracts/*.json, quickstart.md, updated agent-specific file

---

## Phase 2: Task Generation (Deferred to /sp.tasks)

**NOT executed by /sp.plan**. After Phase 0-1 complete, user runs `/sp.tasks` to generate implementation checklist.

Expected task categories:
1. **Setup Tasks**: Install Whisper, configure LLM APIs, test microphone
2. **Chapter 1 Tasks**: Write voice-to-action.md, create Whisper ROS 2 node, add practice exercises
3. **Chapter 2 Tasks**: Write cognitive-planning.md, create LLM planner node, define action primitives, document prompt engineering
4. **Chapter 3 Tasks**: Write capstone-vla.md, create behavior coordinator, integrate Module 3 nodes, record demo video
5. **Example Packaging**: Create .zip archives, write READMEs, test on clean Ubuntu 22.04
6. **Quality Assurance**: Peer-review citations, test Whisper accuracy, validate LLM outputs, proofread chapters

---

## Constitution Re-Check (Post-Phase 1)

### Principle I: Spec-Driven, AI-First Development
✅ **PASS**: Plan follows spec requirements. Tasks will be generated via `/sp.tasks` before implementation.

### Principle II: Accuracy and Content-Grounded Responses
✅ **PASS**: All content sourced from official docs and peer-reviewed papers (RT-2, SayCan, PaLM-E, Code as Policies).

### Principle III: Clear, Developer-Focused Writing
✅ **PASS**: Learning outcomes model ensures each chapter has runnable examples, exercises, troubleshooting.

### Principle IV: Reproducible and Production-Ready Setup
✅ **PASS**: Quickstart guide includes installation validation steps, .env.example files, microphone testing.

### Principle V: Free-Tier and Open-Source Compliant
✅ **PASS**: Ollama (free) is default path. GPT-4 (paid) is optional enhancement. All code open-source.

**Final Gate**: All Constitution principles satisfied. Ready for `/sp.tasks` generation.

---

## Next Steps

1. ✅ **Phase 0 Complete**: Generate `research.md` with Whisper/LLM/VLA research findings
2. ✅ **Phase 1 Complete**: Generate `data-model.md`, `/contracts/*.json`, `quickstart.md`
3. ⏳ **Phase 2 Pending**: Run `/sp.tasks` to generate implementation checklist
4. ⏳ **Implementation**: Execute tasks with `/sp.implement` (write chapters, code examples, exercises)
5. ⏳ **Commit & PR**: Use `/sp.git.commit_pr` to finalize Module 4

**Report**:
- Branch: `004-vision-language-action`
- Plan file: `specs/004-vision-language-action/plan.md`
- Next command: `/sp.tasks` (after research.md and data-model.md generation)
