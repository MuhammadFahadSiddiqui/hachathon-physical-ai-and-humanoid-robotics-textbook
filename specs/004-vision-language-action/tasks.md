# Tasks: Module 4 - Vision-Language-Action (VLA)

**Input**: Design documents from `specs/004-vision-language-action/`
**Prerequisites**: plan.md (required), spec.md (required)

**Tests**: Manual validation only (no automated tests for educational content)

**Organization**: Tasks grouped by user story (Chapter 1: Voice-to-Action, Chapter 2: Cognitive Planning, Chapter 3: Capstone) for independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story/chapter this task belongs to (US1=Ch1, US2=Ch2, US3=Ch3)
- Include exact file paths in descriptions

## Path Conventions

Educational content creation (not software development):
- **Content**: `frontend_book/docs/module-4/` (Markdown chapter files)
- **Examples**: `frontend_book/static/examples/` (Python ROS 2 nodes, JSON schemas, YAML configs)
- **Downloads**: .zip archives created from example directories

---

## Phase 1: Setup (Module 4 Infrastructure)

**Purpose**: Initialize Module 4 structure and documentation placeholders

- [X] T001 Create module-4 directory structure in frontend_book/docs/module-4/
- [X] T002 [P] Create examples directory structure in frontend_book/static/examples/ (whisper_voice_commands/, llm_task_planner/, vla_capstone_demo/)
- [X] T003 [P] Update frontend_book/sidebars.ts to include Module 4 navigation (module-4/index, chapter-1, chapter-2, chapter-3)

**Checkpoint**: Module 4 directories created, ready for content generation

---

## Phase 2: Foundational (Shared Resources)

**Purpose**: Resources needed across all chapters (landing page, shared assets)

**⚠️ CRITICAL**: Complete before any chapter implementation

- [X] T004 Write Module 4 landing page (frontend_book/docs/module-4/index.md) with learning objectives, prerequisites (Modules 1-3 completion, microphone hardware, LLM API key or Ollama), hardware requirements, chapter overview
- [X] T005 [P] Document LLM API options in index.md (Option 1: Ollama free local inference, Option 2: OpenAI GPT-4 paid API, cost comparison ~$0.03/request, installation links)
- [X] T006 [P] Copy simple_humanoid.urdf from Module 1 to frontend_book/static/examples/vla_capstone_demo/urdf/ for capstone integration

**Checkpoint**: Foundation ready - chapter implementation can begin in parallel

---

## Phase 3: User Story 1 - Voice-to-Action Speech Interface (Priority: P1) 🎯 MVP

**Goal**: Create Chapter 1 (2500-4000 words) teaching students to implement voice command recognition using OpenAI Whisper and ROS 2 integration

**Independent Test**: Students can launch Whisper ROS 2 node, speak "move forward two meters", verify transcription published to /voice_commands topic within 2s with >90% accuracy

### Content Writing (Chapter 1)

- [X] T007 [US1] Write Chapter 1 Introduction section (300-400 words) in frontend_book/docs/module-4/chapter-1-voice-to-action.md explaining voice interfaces for HRI, Whisper model capabilities, ROS 2 integration benefits, VLA system architecture overview
- [X] T008 [US1] Write Installation section (400-500 words) with Whisper installation (pip install openai-whisper sounddevice), model selection (base 74M recommended for accuracy/latency balance), Python 3.10+ requirement, microphone testing (python -m sounddevice)
- [X] T009 [US1] Write Core Concepts section (500-600 words) explaining speech-to-text processing, confidence scoring (threshold >70%), wake word detection (optional), noise filtering, ROS 2 topic publishing (/voice_commands), std_msgs/String message format
- [X] T010 [US1] Write Runnable Example section (600-800 words) with step-by-step tutorial: create whisper_ros2 package, implement voice_command_node.py (audio capture, Whisper transcription, ROS 2 publisher), launch file, test with 10 example commands (navigation, manipulation, system control)
- [X] T011 [US1] Write Practice Exercises section (300-400 words) with 4-6 exercises: (1) Add custom voice commands for arm control, (2) Implement wake word detection ("hey robot"), (3) Adjust confidence threshold to 80%, (4) Test accuracy across accents, (5) Add logging for transcription errors, (6) Integrate with text-to-speech feedback
- [X] T012 [US1] Write Troubleshooting section (200-300 words) addressing "Whisper fails to detect microphone" (check sounddevice.query_devices()), "Transcription latency >5s" (use base model not medium/large), "Low accuracy <70%" (reduce background noise, use USB microphone), "ROS 2 node won't start" (verify rclpy installation)
- [X] T013 [US1] Write References section with 2+ peer-reviewed papers: Radford et al. 2022 (Whisper: Robust Speech Recognition via Large-Scale Weak Supervision), RT-2 DeepMind 2023 (vision-language-action models), speech interfaces for robotics HRI surveys

### Example Code (Whisper ROS 2)

- [X] T014 [P] [US1] Create whisper_ros2 ROS 2 package structure in frontend_book/static/examples/whisper_voice_commands/whisper_ros2/ with package.xml, setup.py, __init__.py
- [X] T015 [P] [US1] Implement voice_command_node.py in whisper_ros2/whisper_ros2/ with audio capture (sounddevice), Whisper model loading (base model), transcription loop, confidence thresholding (>70%), ROS 2 publisher to /voice_commands topic
- [X] T016 [P] [US1] Create voice_interface.launch.py in frontend_book/static/examples/whisper_voice_commands/launch/ to launch voice_command_node with configurable parameters (model_size, confidence_threshold, microphone_device_id)
- [X] T017 [P] [US1] Create voice_commands.yaml in config/ with 10+ example commands (navigation: "go to kitchen", manipulation: "pick up cube", system: "stop", "pause", "resume") for testing
- [X] T018 [P] [US1] Create .env.example in frontend_book/static/examples/whisper_voice_commands/ with MICROPHONE_DEVICE_ID placeholder (comment: run python -m sounddevice to list devices)
- [X] T019 [US1] Write README.md in frontend_book/static/examples/whisper_voice_commands/ with installation (pip install), ROS 2 package build (colcon build), launch commands, testing instructions (ros2 topic echo /voice_commands), expected performance (<2s latency, >90% accuracy on 50-command test set)

### Validation (Chapter 1)

- [X] T020 [US1] Validate Chapter 1 word count (2500-4000 words), frontmatter metadata (title: "Voice-to-Action with OpenAI Whisper", sidebar_position: 2, id: chapter-1-voice-to-action)
- [ ] T021 [US1] Test voice_command_node.py with built-in laptop microphone (verify Whisper base model loads, audio captured at 16kHz, transcription published to /voice_commands within 2s)
- [ ] T022 [US1] Test transcription accuracy on 50-command test set (verify >90% word accuracy in quiet environment <40dB, >80% with background noise 60dB)
- [ ] T023 [US1] Test confidence thresholding (verify commands with confidence <70% are rejected, logged, not published to /voice_commands)
- [X] T024 [US1] Create whisper_voice_commands.zip from frontend_book/static/examples/whisper_voice_commands/ directory (verify <100 MB size limit)

**Checkpoint US1**: Chapter 1 complete (2500-4000 words, 4-6 exercises, 2+ citations, runnable Whisper ROS 2 node tested with >90% accuracy, .zip download ready)

---

## Phase 4: User Story 2 - LLM-Based Cognitive Task Planning (Priority: P1) 🎯 MVP

**Goal**: Create Chapter 2 (2500-4000 words) teaching students to translate natural language commands into executable ROS 2 action sequences using LLMs (GPT-4/Llama 3.1)

**Independent Test**: Students can send "go to the kitchen and bring me a water bottle" to LLM planner node, verify JSON action sequence [{navigate, detect_object, grasp, navigate}] published to /task_plan within 5s

### Content Writing (Chapter 2)

- [X] T025 [US2] Write Chapter 2 Introduction section (300-400 words) in frontend_book/docs/module-4/chapter-2-cognitive-planning.md explaining LLM-based planning for robotics, GPT-4 vs Llama 3.1 comparison, action primitive decomposition, VLA research context (SayCan, Code as Policies)
- [X] T026 [US2] Write LLM Setup section (500-600 words) with dual-path installation: (1) Ollama setup (curl install.sh, ollama pull llama3.1:8b, REST API localhost:11434), (2) OpenAI API setup (pip install openai, API key from platform.openai.com, set OPENAI_API_KEY env var, pricing ~$0.03/request)
- [X] T027 [US2] Write Action Primitives section (600-700 words) defining 8-12 primitives (navigate, detect_object, grasp, release, open_gripper, close_gripper, move_joint, wait), JSON schema format (action enum, parameters dict, preconditions list, expected_duration), parameter validation, ROS 2 action server mapping
- [X] T028 [US2] Write Prompt Engineering section (700-900 words) explaining system prompts (constrain to valid primitives), few-shot examples (simple "move to table" → complex "make sandwich"), temperature=0 for deterministic output, JSON output formatting, error handling for invalid plans, LLM response parsing
- [X] T029 [US2] Write Plan Validation section (400-500 words) on checking LLM outputs against robot capabilities (target location reachable, object graspable, preconditions satisfied), retry logic for invalid plans, fallback to cached plans (common commands), clarification requests (/clarification_request topic)
- [X] T030 [US2] Write Practice Exercises section (300-400 words) with 4-6 exercises: (1) Add custom action primitive (move_arm), (2) Implement retry logic for failed grasps, (3) Test LLM with ambiguous commands ("clean up"), (4) Measure planning latency (simple vs complex tasks), (5) Compare GPT-4 vs Llama quality, (6) Visualize task plans in RViz
- [X] T031 [US2] Write Troubleshooting section (200-300 words) addressing "LLM planner generates invalid JSON" (check prompt template, add JSON schema validation), "OpenAI API rate limit exceeded" (implement exponential backoff, switch to Ollama), "Ollama connection refused" (verify localhost:11434 accessible), "Plan execution timeout" (adjust expected_duration estimates)
- [X] T032 [US2] Write References section with 2+ peer-reviewed papers: Ahn et al. 2022 (SayCan: Do As I Can, Not As I Say), Liang et al. 2022 (Code as Policies), ProgPrompt 2023 (LLM task planning)

### Example Code (LLM Planner)

- [X] T033 [P] [US2] Create llm_planner ROS 2 package structure in frontend_book/static/examples/llm_task_planner/llm_planner/ with package.xml, setup.py, __init__.py
- [ ] T034 [P] [US2] Implement action_primitives.py in llm_planner/ defining 8-12 action primitive classes (Navigate, DetectObject, Grasp, Release, OpenGripper, CloseGripper, MoveJoint, Wait) with JSON schema validation, parameter bounds checking, precondition verification
- [ ] T035 [P] [US2] Implement prompt_templates.py in llm_planner/ with system prompt (constraint to action primitives), 3-5 few-shot examples (simple → complex task decomposition), JSON output template, error handling instructions
- [ ] T036 [P] [US2] Implement planner_node.py in llm_planner/ with LLM API client (OpenAI SDK for GPT-4, requests for Ollama), ROS 2 subscriber to /voice_commands, task planning function (send prompt to LLM, parse JSON response), plan validation, ROS 2 publisher to /task_plan topic
- [ ] T037 [P] [US2] Create llm_planner.launch.py in frontend_book/static/examples/llm_task_planner/launch/ to launch planner_node with configurable LLM backend (gpt-4 or llama3.1), temperature, max_tokens
- [ ] T038 [P] [US2] Create action_primitives.json in config/ with JSON schema definitions for all 8-12 action primitives (type, required parameters, optional parameters, parameter ranges, preconditions, expected_duration examples)
- [ ] T039 [P] [US2] Create llm_config.yaml in config/ with LLM selection (model: "gpt-4-turbo" or "llama3.1:8b"), API settings (temperature: 0, max_tokens: 500), timeout: 15s
- [ ] T040 [P] [US2] Create .env.example in frontend_book/static/examples/llm_task_planner/ with OPENAI_API_KEY and OLLAMA_HOST placeholders (comments: signup at platform.openai.com for GPT-4, or use localhost:11434 for Ollama)
- [ ] T041 [US2] Write README.md in frontend_book/static/examples/llm_task_planner/ with LLM setup (GPT-4 API key or Ollama installation), ROS 2 package build, launch commands, testing (echo "go to table and pick up cup" to /voice_commands, verify /task_plan output), prompt engineering tips

### Validation (Chapter 2)

- [ ] T042 [US2] Validate Chapter 2 word count (2500-4000 words), frontmatter metadata (title: "Cognitive Planning with LLMs", sidebar_position: 3, id: chapter-2-cognitive-planning)
- [ ] T043 [US2] Test planner_node.py with GPT-4 API (verify simple command "move to kitchen" → JSON [{action: "navigate", target: "kitchen"}] generated in <5s)
- [ ] T044 [US2] Test planner_node.py with Llama 3.1 via Ollama (verify complex command "make sandwich" → 8-12 action sequence with valid preconditions, generated in <15s)
- [ ] T045 [US2] Test plan validation (verify invalid actions rejected, e.g., "fly to ceiling" returns error message, impossible parameters caught before ROS 2 publish)
- [ ] T046 [US2] Measure LLM planning latency (verify simple tasks <5s for GPT-4 and Llama, complex tasks <15s for GPT-4, <20s for Llama on 16GB RAM system)
- [ ] T047 [US2] Create llm_task_planner.zip from frontend_book/static/examples/llm_task_planner/ directory (verify <100 MB size limit)

**Checkpoint US2**: Chapter 2 complete (2500-4000 words, 4-6 exercises, 2+ citations, runnable LLM planner achieves 95% valid action sequences, <5s simple / <15s complex latency, .zip download ready)

---

## Phase 5: User Story 3 - End-to-End Capstone: Autonomous Humanoid Demo (Priority: P2)

**Goal**: Create Chapter 3 (2500-4000 words) integrating voice commands, LLM planning, navigation, vision, and manipulation into complete autonomous humanoid system

**Independent Test**: Students can run full system in Isaac Sim, speak "bring me the bottle from the shelf", verify humanoid completes 6-step task (transcribe → plan → navigate → detect → grasp → return) autonomously in <2min

### Content Writing (Chapter 3)

- [ ] T048 [US3] Write Chapter 3 Introduction section (300-400 words) in frontend_book/docs/module-4/chapter-3-capstone-vla.md explaining end-to-end VLA integration, system architecture (Whisper → LLM → behavior coordinator → Nav2/Isaac ROS/gripper), capstone as portfolio demonstration, real-world deployment considerations
- [ ] T049 [US3] Write System Architecture section (600-700 words) with Mermaid diagram showing node connections (voice_command_node → llm_planner → behavior_coordinator → action_executor → Nav2/Isaac ROS/gripper), topic flow (/voice_commands, /task_plan, /execution_status), state machine overview (idle → planning → executing → completed/failed)
- [ ] T050 [US3] Write Behavior Coordination section (700-900 words) explaining state machine implementation (Python Enum states), action primitive execution (ROS 2 action client calls to Nav2, Isaac ROS, gripper), failure handling (retry logic, replanning, user notification), progress tracking (/execution_status topic), preemption support (cancel current action on new command)
- [ ] T051 [US3] Write Module Integration section (600-700 words) on connecting Whisper (Chapter 1) → LLM (Chapter 2) → Nav2 (Module 3 Ch3) → Isaac ROS perception (Module 3 Ch2) → gripper controller (new), launch file composition (all nodes in one launch), RViz debugging visualization (voice commands text, task plan viewer, execution status progress bar)
- [ ] T052 [US3] Write Runnable Example section (800-1000 words) with capstone demo tutorial: launch Isaac Sim warehouse scene, start capstone_demo.launch.py (Whisper + LLM + behavior coordinator + Nav2 + Isaac ROS + gripper + RViz), speak "pick up red cube from table", observe full task execution (transcription → 4-step plan → navigation → object detection → grasp → return), verify completion in <2min
- [ ] T053 [US3] Write Practice Exercises section (300-400 words) with 4-6 exercises: (1) Add gripper force feedback logging, (2) Implement multi-object task ("bring all bottles"), (3) Add obstacle avoidance recovery, (4) Record demo video/GIF, (5) Test system resilience (disconnect microphone, simulate detection failure), (6) Tune state machine timeouts
- [ ] T054 [US3] Write Troubleshooting section (200-300 words) addressing "Action primitive execution timeout" (increase expected_duration, check ROS 2 action server status), "Gripper fails to grasp" (verify object pose from Isaac ROS, tune grasp parameters), "System stuck in planning state" (check LLM API connection, enable fallback to cached plans), "RViz doesn't show task plan" (verify /task_plan topic publishing)
- [ ] T055 [US3] Write References section with 2+ peer-reviewed papers: PaLM-E Google 2023 (embodied multimodal language models), RT-2 DeepMind 2023 (vision-language-action), VLA systems survey (Science Robotics 2024)

### Example Code (Capstone Integration)

- [ ] T056 [P] [US3] Create vla_coordinator ROS 2 package structure in frontend_book/static/examples/vla_capstone_demo/vla_coordinator/ with package.xml, setup.py, __init__.py
- [ ] T057 [P] [US3] Implement behavior_coordinator.py in vla_coordinator/ with state machine (idle, planning, executing, completed, failed), action sequence management, ROS 2 subscriber to /task_plan, action executor calls, progress tracking publisher to /execution_status
- [ ] T058 [P] [US3] Implement action_executor.py in vla_coordinator/ with ROS 2 action clients for Nav2 (/navigate_to_pose), Isaac ROS (/detect_objects), gripper (/gripper_action), action primitive dispatch (navigate → Nav2 client, detect_object → Isaac ROS client, grasp → gripper client), timeout handling, preemption support
- [ ] T059 [P] [US3] Implement system_monitor.py in vla_coordinator/ with heartbeat publisher (all nodes alive), error logging, performance metrics (task completion time, action success rates), RViz visualization data (voice commands text, task plan JSON, execution progress percentage)
- [ ] T060 [P] [US3] Create capstone_demo.launch.py in frontend_book/static/examples/vla_capstone_demo/launch/ to start all nodes (voice_command_node from Ch1, llm_planner from Ch2, behavior_coordinator, Nav2 from Module 3, Isaac ROS from Module 3, gripper_controller, system_monitor, RViz with custom config)
- [ ] T061 [P] [US3] Create system_config.yaml in config/ with integrated system parameters (action timeouts, retry limits, state transition delays, RViz topic remapping)
- [ ] T062 [P] [US3] Create rviz_vla_debug.rviz in config/ with custom RViz panels (voice commands text display, task plan JSON viewer, execution status progress bar, Nav2 costmaps, Isaac ROS point clouds, gripper camera feed)
- [ ] T063 [P] [US3] Create humanoid_gripper.urdf in urdf/ extending simple_humanoid.urdf from Module 1 with parallel jaw gripper (2 fingers, prismatic joint, force sensor)
- [ ] T064 [US3] Write README.md in frontend_book/static/examples/vla_capstone_demo/ with Isaac Sim scene setup, full launch instructions, demo walkthrough ("speak command → observe execution"), video recording guide (OBS Studio recommended), expected performance (<2min for 5-action sequence), troubleshooting integration issues

### Validation (Chapter 3)

- [ ] T065 [US3] Validate Chapter 3 word count (2500-4000 words), frontmatter metadata (title: "Capstone: Autonomous Humanoid VLA", sidebar_position: 4, id: chapter-3-capstone-vla)
- [ ] T066 [US3] Test capstone_demo.launch.py in Isaac Sim (verify all 7+ nodes start without errors: Whisper, LLM planner, behavior coordinator, Nav2, Isaac ROS perception, gripper controller, system monitor)
- [ ] T067 [US3] Test end-to-end task execution (speak "pick up red cube from table", verify: transcription →  4-step plan generated → navigation to table → Isaac ROS detects cube → gripper grasps → navigation back, total time <2min)
- [ ] T068 [US3] Test failure recovery (simulate object detection failure, verify behavior coordinator requests LLM replanning or user clarification via /clarification_request topic)
- [ ] T069 [US3] Test RViz visualization (verify voice commands text display updates, task plan JSON viewer shows action sequence, execution status progress bar increments, Nav2 costmaps visible, Isaac ROS point cloud displayed)
- [ ] T070 [US3] Record demo video or GIF (screen capture of full system executing "bring me the bottle from the shelf" from start to completion, <2min runtime)
- [ ] T071 [US3] Create vla_capstone_demo.zip from frontend_book/static/examples/vla_capstone_demo/ directory (verify <100 MB size limit)

**Checkpoint US3**: Chapter 3 complete (2500-4000 words, 4-6 exercises, 2+ citations, runnable capstone demo completes end-to-end task in <2min, demo video/GIF recorded, .zip download ready)

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final quality assurance, integration, and deployment readiness

### Module Integration

- [ ] T072 Update Module 4 landing page (frontend_book/docs/module-4/index.md) with download links to all 3 .zip files (whisper_voice_commands.zip, llm_task_planner.zip, vla_capstone_demo.zip)
- [ ] T073 [P] Update main Docusaurus homepage (frontend_book/docs/intro.md) to reference Module 4 completion (VLA systems, LLM-driven planning, autonomous humanoid capstone)
- [ ] T074 [P] Add Module 4 to Docusaurus sidebar navigation (verify Module 4 appears after Module 3, chapters show in correct order: index → chapter-1 → chapter-2 → chapter-3)

### Validation & Testing

- [ ] T075 Run Docusaurus build (npm run build in frontend_book/) and verify no MDX compilation errors in Module 4 chapters
- [ ] T076 [P] Verify all 3 chapter word counts (Chapter 1: 2500-4000 words, Chapter 2: 2500-4000 words, Chapter 3: 2500-4000 words)
- [ ] T077 [P] Verify all chapters have 4-6 practice exercises each (Chapter 1: 4-6, Chapter 2: 4-6, Chapter 3: 4-6 = 12-18 total)
- [ ] T078 [P] Verify all chapters have 2+ peer-reviewed citations (Chapter 1: 2, Chapter 2: 2, Chapter 3: 2 = 6 total from CoRL/RSS/ICRA/NeurIPS/T-RO/Science Robotics 2020+)
- [ ] T079 [P] Verify all 3 .zip downloads exist and are <100 MB each (whisper_voice_commands.zip, llm_task_planner.zip, vla_capstone_demo.zip)
- [ ] T080 Proofread all 3 chapters for spelling/grammar, technical accuracy, consistent terminology (Whisper base model not small, Ollama not ollama-cli, GPT-4 Turbo pricing accurate)

### Documentation Links

- [ ] T081 [P] Verify all internal links work (Module 4 index → chapters, chapters → download .zips, troubleshooting → previous modules, capstone → Module 3 Nav2/Isaac ROS)
- [ ] T082 [P] Verify all external documentation links accessible (OpenAI Whisper GitHub, Ollama docs, OpenAI Platform API docs, peer-reviewed paper DOIs/arXiv links)
- [ ] T083 [P] Verify RViz visualization instructions include detailed descriptions (voice commands text panel config, task plan JSON viewer setup, execution status progress bar colors)

### Final Build & Commit

- [ ] T084 Run final Docusaurus build (npm run build) and serve locally (npm run serve) to manually test Module 4 navigation, download links, code syntax highlighting
- [ ] T085 Create git commit for Module 4 completion with descriptive message (3 chapters, 12-18 exercises, 6 citations, 3 .zip downloads, VLA capstone demo, MVP scope complete)
- [ ] T086 Update project README or documentation index to reflect Module 4 availability (VLA systems coverage, LLM API disclaimer Ollama free / GPT-4 paid, capstone portfolio demonstration)

**Checkpoint Phase 6**: Module 4 complete and production-ready. Docusaurus build successful, all chapters validated, examples tested with GPT-4 + Llama 3.1, capstone demo video recorded, ready for deployment.

---

## Task Summary

**Total Tasks**: 86 tasks
- **Phase 1 (Setup)**: 3 tasks
- **Phase 2 (Foundational)**: 3 tasks
- **Phase 3 (US1 - Voice-to-Action)**: 18 tasks
- **Phase 4 (US2 - Cognitive Planning)**: 23 tasks
- **Phase 5 (US3 - Capstone)**: 24 tasks
- **Phase 6 (Polish)**: 15 tasks

**User Story Task Counts**:
- **US1 (Chapter 1)**: 18 tasks (content writing: 7, example code: 6, validation: 5)
- **US2 (Chapter 2)**: 23 tasks (content writing: 8, example code: 9, validation: 6)
- **US3 (Chapter 3)**: 24 tasks (content writing: 8, example code: 9, validation: 7)

**Parallel Opportunities**:
- Phase 1: All 3 tasks can run in parallel (different directories)
- Phase 2: Tasks T005 and T006 can run in parallel
- Phase 3 (US1): Content tasks T007-T013 can run in parallel with example code tasks T014-T019 (different files)
- Phase 4 (US2): Content tasks T025-T032 can run in parallel with example code tasks T033-T041 (different files)
- Phase 5 (US3): Content tasks T048-T055 can run in parallel with example code tasks T056-T064 (different files)
- Phase 6: Most validation and documentation tasks marked [P] can run in parallel

**Independent Test Criteria**:
- **US1**: Launch Whisper ROS 2 node → speak command → verify /voice_commands topic published within 2s with >90% accuracy
- **US2**: Send natural language to LLM planner → verify JSON action sequence on /task_plan within 5s, 95% valid plans
- **US3**: Speak "bring me the bottle" in Isaac Sim → verify full task execution (transcribe → plan → navigate → detect → grasp → return) in <2min

**MVP Scope**: Phase 1 + Phase 2 + Phase 3 (US1) = 24 tasks
- Delivers working voice interface (Whisper ROS 2 integration)
- Students can speak commands and see transcriptions
- Foundation for LLM planning (Phase 4) and capstone (Phase 5)

**Incremental Delivery**:
1. **MVP (US1)**: Voice-to-Action chapter complete, downloadable Whisper ROS 2 package
2. **US1 + US2**: Add Cognitive Planning chapter, downloadable LLM planner package
3. **US1 + US2 + US3**: Full Module 4 with end-to-end capstone, portfolio-ready demo

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories CAN proceed in parallel (if staffed)
  - OR sequentially in priority order (US1 → US2 → US3)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1 - Voice)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1 - Planning)**: Can start after Foundational (Phase 2) - Independent of US1, but typically follows US1 for logical flow
- **User Story 3 (P2 - Capstone)**: Depends on US1 + US2 completion (integrates Whisper node from US1 and LLM planner from US2)

### Within Each User Story

- Content writing and example code can proceed in parallel (different files)
- Validation tasks depend on content + code completion
- .zip creation depends on all example code completion

### Parallel Opportunities Per Story

**User Story 1 (Voice-to-Action)**:
- Content tasks T007-T013 can all run in parallel (different sections of same chapter)
- Example code tasks T014-T019 can all run in parallel (different files/directories)
- After content + code complete: Validation tasks T020-T024 run sequentially

**User Story 2 (Cognitive Planning)**:
- Content tasks T025-T032 can all run in parallel
- Example code tasks T033-T041 can all run in parallel
- After content + code complete: Validation tasks T042-T047 run sequentially

**User Story 3 (Capstone)**:
- Content tasks T048-T055 can all run in parallel
- Example code tasks T056-T064 can all run in parallel
- After content + code complete: Validation tasks T065-T071 run sequentially

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (3 tasks)
2. Complete Phase 2: Foundational (3 tasks)
3. Complete Phase 3: User Story 1 - Voice-to-Action (18 tasks)
4. **STOP and VALIDATE**: Test Chapter 1 independently (Whisper ROS 2 node, voice commands, transcription accuracy)
5. Deploy/demo if ready (voice interface working)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready (6 tasks)
2. Add User Story 1 → Test independently → Deploy/Demo (MVP! 24 tasks total)
3. Add User Story 2 → Test independently → Deploy/Demo (47 tasks total)
4. Add User Story 3 → Test independently → Deploy/Demo (71 tasks total)
5. Add Polish → Final validation → Production ready (86 tasks total)

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (6 tasks)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (Voice-to-Action, 18 tasks)
   - **Developer B**: User Story 2 (Cognitive Planning, 23 tasks)
   - **Wait for A+B**: User Story 3 (Capstone, 24 tasks) - requires US1 + US2 integration
3. All developers: Polish (15 tasks, can parallelize validation tasks)

---

## Notes

- [P] tasks = different files, no dependencies, can run in parallel
- [Story] label (US1/US2/US3) maps task to specific chapter for traceability
- Each user story should be independently completable and testable (except US3 which integrates US1+US2)
- US1 + US2 = Both P1 priority, can run in parallel by different developers
- US3 = P2 priority, depends on US1 + US2 completion for integration
- Commit after each user story checkpoint to validate independently
- Avoid: vague tasks, same file conflicts, blocking dependencies within user stories
- Module 4 follows Module 1-3 educational content patterns (2500-4000 words, exercises, citations, .zip examples)
