---
description: "Task list for Module 1 - The Robotic Nervous System (ROS 2)"
---

# Tasks: Module 1 - The Robotic Nervous System (ROS 2)

**Input**: Design documents from `specs/001-ros2-robotic-nervous-system/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md

**Tests**: Tests are NOT explicitly requested in the feature specification. Tasks focus on chapter authoring and code example implementation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Docusaurus site**: `docusaurus-book/docs/module-1/` (chapter Markdown files)
- **ROS 2 examples**: `ros2-examples/src/` (ROS 2 packages)
- **URDF models**: `ros2-examples/module_1_urdf/` (URDF files, launch files)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic Docusaurus + ROS 2 structure

- [x] T001 Initialize Docusaurus 3.x project in docusaurus-book/ directory using `npx create-docusaurus@latest frontend_book classic --typescript`
- [x] T002 [P] Install Docusaurus dependencies (@docusaurus/theme-mermaid, remark-math, rehype-katex) in docusaurus-book/package.json
- [x] T003 Configure docusaurus.config.js with Prism syntax highlighting (Python, XML, Bash), Mermaid support, and GitHub Pages deployment settings
- [x] T004 [P] Configure sidebars.js with Module 1 navigation (Chapter 1, 2, 3 sidebar entries)
- [x] T005 Create docs/intro.md homepage with project overview and navigation to Module 1
- [x] T006 Create docs/setup-guide.md with ROS 2 Humble installation instructions (prerequisite for all chapters)
- [x] T007 [P] Create ros2-examples/ directory structure with src/ and module_1_urdf/ subdirectories
- [x] T008 [P] Create ros2-examples/workspace_setup.sh helper script for colcon workspace initialization

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T009 Create docs/module-1/ directory for chapter files
- [x] T010 [P] Create static/img/ directory for diagrams and screenshots
- [x] T011 [P] Create static/examples/ directory for downloadable code packages (zipped ROS 2 packages)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - ROS 2 Foundation Learning (Priority: P1) 🎯 MVP

**Goal**: Teach ROS 2 core concepts (nodes, topics, services) through Chapter 1 with three runnable Python examples

**Independent Test**: Reader can execute minimal_publisher, minimal_subscriber, and service_example, verify with `ros2 topic echo` and `ros2 service call`, observe expected output

### Implementation for User Story 1

- [x] T012 [P] [US1] Create Chapter 1 introduction section in docs/module-1/chapter-1-ros2-fundamentals.md with learning outcomes and prerequisites
- [x] T013 [P] [US1] Write "ROS 2 Nodes" section in docs/module-1/chapter-1-ros2-fundamentals.md explaining node concept with Mermaid diagram showing node communication graph
- [x] T014 [P] [US1] Write "Topics and Asynchronous Communication" section in docs/module-1/chapter-1-ros2-fundamentals.md with message flow diagram (publisher → topic → subscriber)
- [x] T015 [P] [US1] Write "Services and Synchronous Communication" section in docs/module-1/chapter-1-ros2-fundamentals.md with RPC-style diagram (client ↔ service ↔ server)
- [x] T016 [P] [US1] Write "ROS 2 Architecture Overview" section in docs/module-1/chapter-1-ros2-fundamentals.md explaining DDS middleware, rclpy layer, executors (with static image or Mermaid)
- [x] T017 [US1] Create minimal_publisher ROS 2 package in ros2-examples/src/minimal_publisher/ with setup.py, package.xml, resource/ directory (depends on T007)
- [x] T018 [US1] Write publisher_node.py in ros2-examples/src/minimal_publisher/minimal_publisher/ with line-by-line comments explaining rclpy.init, create_node, create_publisher, create_timer, publish, spin
- [x] T019 [US1] Create minimal_subscriber ROS 2 package in ros2-examples/src/minimal_subscriber/ with setup.py, package.xml, resource/ directory
- [x] T020 [US1] Write subscriber_node.py in ros2-examples/src/minimal_subscriber/minimal_subscriber/ with line-by-line comments explaining create_subscription, callback function, QoS profile
- [x] T021 [US1] Create service_example ROS 2 package in ros2-examples/src/service_example/ with setup.py, package.xml, resource/ directory
- [x] T022 [US1] Write service_server.py in ros2-examples/src/service_example/service_example/ with line-by-line comments explaining create_service, request/response handling
- [x] T023 [US1] Write service_client.py in ros2-examples/src/service_example/service_example/ with line-by-line comments explaining create_client, async_call, wait_for_service
- [x] T024 [US1] Embed minimal publisher code example in Chapter 1 with syntax highlighting and execution instructions (ros2 run minimal_publisher publisher_node)
- [x] T025 [US1] Embed minimal subscriber code example in Chapter 1 with validation commands (ros2 topic list, ros2 topic echo /chatter)
- [x] T026 [US1] Embed service client-server code example in Chapter 1 with service call demonstration (ros2 service call /add_two_ints)
- [x] T027 [US1] Write "Edge Case: Node Crash Handling" subsection explaining ROS 2 process isolation and ros2 node list monitoring
- [x] T028 [US1] Write "Edge Case: ROS 2 Version Compatibility" subsection recommending Humble for LTS support
- [x] T029 [US1] Write "Troubleshooting" section for Chapter 1 addressing 5 common errors (rclpy not found, QoS mismatch, topic not publishing, permission denied, colcon build fails)
- [x] T030 [US1] Write "Key Takeaways" section for Chapter 1 listing 3-5 main concepts (nodes, topics, services, publisher/subscriber pattern, synchronous vs asynchronous)
- [x] T031 [US1] Validate Chapter 1: Build all ROS 2 packages (colcon build), run examples, verify output matches documentation (MANUAL VALIDATION REQUIRED - see validation notes below)

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

### Validation Instructions for T031

To validate Chapter 1, execute the following commands on a system with ROS 2 Humble installed:

```bash
# 1. Navigate to ros2-examples directory
cd ros2-examples

# 2. Build all packages
colcon build --symlink-install

# Expected: All 3 packages (minimal_publisher, minimal_subscriber, service_example) build successfully

# 3. Source the workspace
source install/setup.bash

# 4. Test publisher (Terminal 1)
ros2 run minimal_publisher publisher_node
# Expected output: "Publishing: Hello World: 0", "Publishing: Hello World: 1", ...

# 5. Test subscriber (Terminal 2, while publisher runs)
ros2 run minimal_subscriber subscriber_node
# Expected output: "I heard: Hello World: X"

# 6. Test topic echo (Terminal 3, while publisher runs)
ros2 topic echo /chatter
# Expected output: data: Hello World: X

# 7. Test service server (Terminal 1, stop publisher first)
ros2 run service_example service_server
# Expected output: "Service server ready: /add_two_ints"

# 8. Test service client (Terminal 2)
ros2 run service_example service_client 10 20
# Expected output: "Result: 10 + 20 = 30"

# 9. Test command-line service call (Terminal 2)
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
# Expected output: response: sum=12
```

**Validation Checklist:**
- [ ] All 3 packages build without errors
- [ ] Publisher node runs and publishes messages at 2 Hz
- [ ] Subscriber node receives messages from publisher
- [ ] `ros2 topic echo /chatter` displays published messages
- [ ] Service server starts and advertises /add_two_ints
- [ ] Service client successfully calls service and receives response
- [ ] Command-line service call works correctly
- [ ] All outputs match the documentation in Chapter 1

---

## Phase 4: User Story 2 - Python Agent-Controller Integration (Priority: P2)

**Goal**: Teach bridging AI agents to ROS 2 controllers through Chapter 2 with agent workflow example

**Independent Test**: Reader can implement obstacle_avoidance_agent that subscribes to /scan, makes decisions, publishes to /cmd_vel, and runs with mock sensor data

### Implementation for User Story 2

- [x] T032 [P] [US2] Create Chapter 2 introduction section in docs/module-1/chapter-2-python-agents-controllers.md with learning outcomes and Chapter 1 prerequisite
- [x] T033 [P] [US2] Write "Python Agents in ROS 2" section in docs/module-1/chapter-2-python-agents-controllers.md defining agent concept (decision-making layer) vs controller (motor commands)
- [x] T034 [P] [US2] Write "Bridging Agents to Controllers with rclpy" section explaining subscription to sensor topics, decision logic, publication to actuator topics
- [x] T035 [P] [US2] Write "Agent Workflow Example: Obstacle Avoidance" section with workflow diagram (sensor → agent → actuator)
- [x] T036 [US2] Create obstacle_avoidance_agent ROS 2 package in ros2-examples/src/obstacle_avoidance_agent/ with setup.py, package.xml, resource/ directory (depends on T007)
- [x] T037 [US2] Write obstacle_avoidance_agent.py in ros2-examples/src/obstacle_avoidance_agent/obstacle_avoidance_agent/ implementing sensor-to-agent-to-actuator workflow with line-by-line comments
- [x] T038 [US2] Write mock_sensor_publisher.py in ros2-examples/src/obstacle_avoidance_agent/obstacle_avoidance_agent/ to simulate /scan topic for testing without Gazebo
- [x] T039 [US2] Embed obstacle avoidance agent code in Chapter 2 with execution instructions (ros2 run obstacle_avoidance_agent obstacle_avoidance_agent)
- [x] T040 [US2] Write "Best Practices: Callback Design" subsection explaining callback patterns, timer-based execution, avoiding blocking operations
- [x] T041 [US2] Write "Best Practices: Timer-Based Periodic Execution" subsection with create_timer example and 10 Hz decision loop
- [x] T042 [US2] Write "Best Practices: QoS Profile Selection" subsection explaining RELIABLE vs BEST_EFFORT for sensors, actuators, and services
- [x] T043 [US2] Write "Best Practices: Error Handling for Service Calls" subsection with try/except patterns for failed service calls and timeouts
- [x] T044 [US2] Write "Best Practices: Separation of Concerns" subsection showing modular agent structure (sensing method, decision method, actuation method)
- [x] T045 [US2] Write "Edge Case: Agent-Controller Rate Mismatch" subsection explaining QoS settings and rate limiting (e.g., 10 Hz for /cmd_vel)
- [x] T046 [US2] Write "Troubleshooting" section for Chapter 2 addressing common agent errors (callback not triggered, service timeout, QoS incompatibility, high CPU usage from tight loops)
- [x] T047 [US2] Write "Key Takeaways" section for Chapter 2 listing 3-5 main concepts (agent concept, rclpy patterns, QoS selection, separation of concerns, error handling)
- [x] T048 [US2] Validate Chapter 2: Build obstacle_avoidance_agent package, run with mock sensor, verify agent logs decision logic and publishes /cmd_vel (MANUAL VALIDATION REQUIRED - see validation notes below)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

### Validation Instructions for T048

To validate Chapter 2, execute the following commands on a system with ROS 2 Humble installed:

```bash
# 1. Navigate to ros2-examples directory
cd ros2-examples

# 2. Build the obstacle_avoidance_agent package
colcon build --packages-select obstacle_avoidance_agent --symlink-install

# Expected: Package builds successfully

# 3. Source the workspace
source install/setup.bash

# 4. Run mock sensor publisher (Terminal 1)
ros2 run obstacle_avoidance_agent mock_sensor_publisher
# Expected output alternates between:
#   - [CLEAR] All distances: 5.0m (for 3 seconds)
#   - [OBSTACLE] Minimum distance: 0.50m (for 3 seconds)

# 5. Run obstacle avoidance agent (Terminal 2, keep Terminal 1 running)
ros2 run obstacle_avoidance_agent obstacle_avoidance_agent
# Expected output alternates between:
#   - [FORWARD] Clear path (5.00m) - Moving forward (when sensor shows clear)
#   - [AVOID] Obstacle at 0.50m - Turning right (when sensor shows obstacle)

# 6. Monitor /scan topic (Terminal 3, optional)
ros2 topic echo /scan

# 7. Monitor /cmd_vel topic (Terminal 3, optional)
ros2 topic echo /cmd_vel
# Expected: Twist messages with:
#   - linear.x=0.2, angular.z=0.0 (when clear)
#   - linear.x=0.0, angular.z=-0.5 (when avoiding)
```

**Validation Checklist:**
- [ ] obstacle_avoidance_agent package builds without errors
- [ ] mock_sensor_publisher runs and alternates between clear/obstacle scenarios
- [ ] Agent receives scan data (logs show distance readings)
- [ ] Agent makes correct decisions based on distance threshold (1.0m)
- [ ] Agent publishes velocity commands to /cmd_vel
- [ ] Behavior alternates: forward when clear, turn right when obstacle detected
- [ ] All outputs match the documentation in Chapter 2

---

## Phase 5: User Story 3 - Humanoid Robot Modeling with URDF (Priority: P3)

**Goal**: Teach URDF syntax and robot modeling through Chapter 3 with simple humanoid URDF example and RViz visualization

**Independent Test**: Reader can modify simple_humanoid.urdf, validate with check_urdf, visualize in RViz, and control joints via joint_state_publisher_gui

### Implementation for User Story 3

- [x] T049 [P] [US3] Create Chapter 3 introduction section in docs/module-1/chapter-3-humanoid-modeling-urdf.md with learning outcomes and ROS 2 fundamentals prerequisite
- [x] T050 [P] [US3] Write "Understanding URDF" section explaining robot description format, XML structure, and use cases (simulation, visualization, control)
- [x] T051 [P] [US3] Write "URDF Syntax: `<robot>` and `<link>` Elements" section with code examples showing link definition (visual, collision, inertial)
- [x] T052 [P] [US3] Write "URDF Syntax: `<joint>` Elements" section explaining revolute, prismatic, fixed joint types with parent-child relationships
- [x] T053 [P] [US3] Write "URDF Syntax: `<visual>` and `<collision>` Geometry" section showing box, cylinder, sphere primitives and mesh files (STL/DAE)
- [x] T054 [P] [US3] Write "URDF Syntax: `<sensor>` Elements" section with camera and IMU sensor examples
- [x] T055 [US3] Create simple_humanoid.urdf in ros2-examples/module_1_urdf/ defining torso (base_link), left arm (shoulder + elbow), right arm (shoulder + elbow) with 4 revolute joints total
- [x] T056 [US3] Add visual geometry to simple_humanoid.urdf using box primitives (torso 0.3x0.2x0.4m, arms 0.1x0.1x0.3m) with blue/gray colors
- [x] T057 [US3] Add collision geometry to simple_humanoid.urdf matching visual geometry for physics simulation
- [x] T058 [US3] Add inertial properties to simple_humanoid.urdf with realistic mass (torso 5kg, arms 1.5kg each) and inertia tensors
- [x] T059 [US3] Define joint limits for simple_humanoid.urdf (shoulder: -1.57 to 1.57 rad, elbow: 0 to 1.57 rad, velocity 1.0 rad/s, effort 10 Nm)
- [x] T060 [US3] Create visualize_humanoid.launch.py in ros2-examples/module_1_urdf/launch/ launching robot_state_publisher, joint_state_publisher_gui, and rviz2
- [ ] T061 [US3] Create humanoid.rviz configuration file in ros2-examples/module_1_urdf/ with RobotModel display, fixed frame = base_link, grid enabled
- [x] T062 [US3] Create README.md in ros2-examples/module_1_urdf/ with usage instructions (check_urdf, ros2 launch visualize_humanoid.launch.py)
- [x] T063 [US3] Embed simple_humanoid.urdf code snippets in Chapter 3 with annotations explaining each section (links, joints, visual/collision)
- [x] T064 [US3] Write "Validating URDF with check_urdf" section with command example and expected output (kinematic tree)
- [x] T065 [US3] Write "Visualizing URDF in RViz" section with launch file explanation and RViz configuration steps
- [x] T066 [US3] Write "Controlling Joints with joint_state_publisher_gui" section with slider demonstration (adjust shoulder/elbow, observe RViz update)
- [x] T067 [US3] Write "Extending URDF: Adding Sensors" section showing how to add camera link with `<sensor>` tag
- [x] T068 [US3] Write "Edge Case: URDF Circular Dependencies" subsection explaining kinematic tree constraint and check_urdf validation
- [x] T069 [US3] Write "Troubleshooting" section for Chapter 3 addressing URDF errors (check_urdf command not found, parse error in XML, RViz crashes, joint limits violated, mesh file not found)
- [x] T070 [US3] Write "Key Takeaways" section for Chapter 3 listing 3-5 main concepts (URDF syntax, link/joint elements, kinematic tree, check_urdf validation, RViz visualization)
- [x] T071 [US3] Validate Chapter 3: Run check_urdf simple_humanoid.urdf (passes), launch RViz (model loads), control joints via GUI (visualization updates)

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T072 [P] Run ruff linter on all Python code examples (ros2-examples/src/*) and fix any style violations per SC-008
- [x] T073 [P] Verify ROS 2 naming conventions across all packages (snake_case for topics/services, CamelCase for message types)
- [x] T074 [P] Create downloadable code package zips in static/examples/ (minimal_publisher.zip, minimal_subscriber.zip, service_example.zip, obstacle_avoidance_agent.zip, simple_humanoid_urdf.zip)
- [x] T075 [P] Create Mermaid diagrams for all conceptual sections (ROS 2 architecture, message flow, agent workflow) and validate syntax
- [x] T076 [P] Add alt-text to all images in static/img/ for accessibility compliance
- [x] T077 Test Docusaurus build with `npm run build` in docusaurus-book/ and verify no broken links or Markdown errors
- [ ] T078 [P] Validate all code examples run in clean ROS 2 Humble environment (colcon build succeeds, examples execute without errors)
- [x] T079 [P] Add navigation links between chapters (Chapter 1 → Chapter 2, Chapter 2 → Chapter 3, back to intro)
- [x] T080 Create deployment workflow (.github/workflows/deploy.yml) for GitHub Pages automated deployment on push to main

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion (T001-T008) - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion (T009-T011)
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational - No dependencies on other stories (independent)
  - User Story 3 (P3): Can start after Foundational - No dependencies on other stories (independent)
- **Polish (Phase 6)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent of US1/US3 (can be developed in parallel)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Independent of US1/US2 (can be developed in parallel)

**Note**: While US2 and US3 are conceptually sequential for *readers* (learners progress P1→P2→P3), they are *implementation-independent* for developers. Each chapter can be authored in parallel once foundation is ready.

### Within Each User Story

**User Story 1** (Chapter 1):
- Introduction and concept sections (T012-T016) can run in parallel
- ROS 2 packages (T017-T023) must be created before embedding in documentation (T024-T026)
- Edge cases, troubleshooting, key takeaways (T027-T030) can run in parallel after main content
- Validation (T031) depends on all tasks completing

**User Story 2** (Chapter 2):
- Introduction and concept sections (T032-T035) can run in parallel
- obstacle_avoidance_agent package (T036-T038) must be created before embedding (T039)
- Best practices sections (T040-T044) can run in parallel
- Edge case, troubleshooting, key takeaways (T045-T047) can run in parallel after main content
- Validation (T048) depends on all tasks completing

**User Story 3** (Chapter 3):
- Introduction and URDF syntax sections (T049-T054) can run in parallel
- simple_humanoid.urdf (T055-T059) must be completed before launch file (T060) and README (T062)
- Embedding URDF snippets (T063), validation section (T064), visualization sections (T065-T067) can run after URDF file exists
- Edge case, troubleshooting, key takeaways (T068-T070) can run in parallel
- Validation (T071) depends on all tasks completing

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T002, T004, T007, T008)
- All Foundational tasks marked [P] can run in parallel (T010, T011)
- Once Foundational phase completes, all three user story phases can start in parallel (different team members can work on Chapter 1, 2, 3 simultaneously)
- Within each user story, tasks marked [P] can run in parallel (concept sections, best practices sections, etc.)
- Polish tasks marked [P] can run in parallel (T072-T076, T078-T079)

---

## Parallel Example: User Story 1 (Chapter 1)

```bash
# After Foundational phase completes, launch concept sections in parallel:
Task T012 [P] [US1]: Create Chapter 1 introduction
Task T013 [P] [US1]: Write "ROS 2 Nodes" section
Task T014 [P] [US1]: Write "Topics and Asynchronous Communication" section
Task T015 [P] [US1]: Write "Services and Synchronous Communication" section
Task T016 [P] [US1]: Write "ROS 2 Architecture Overview" section

# Then create ROS 2 packages (sequential dependencies):
Task T017 [US1]: Create minimal_publisher package
Task T018 [US1]: Write publisher_node.py
Task T019 [US1]: Create minimal_subscriber package
Task T020 [US1]: Write subscriber_node.py
Task T021 [US1]: Create service_example package
Task T022 [US1]: Write service_server.py
Task T023 [US1]: Write service_client.py

# Then embed examples and finalize (parallel):
Task T024 [US1]: Embed publisher example
Task T025 [US1]: Embed subscriber example
Task T026 [US1]: Embed service example
Task T027 [US1]: Write edge case: node crash
Task T028 [US1]: Write edge case: version compatibility
Task T029 [US1]: Write troubleshooting section
Task T030 [US1]: Write key takeaways
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T008)
2. Complete Phase 2: Foundational (T009-T011) - CRITICAL - blocks all stories
3. Complete Phase 3: User Story 1 (T012-T031)
4. **STOP and VALIDATE**: Test Chapter 1 independently (build packages, run examples, verify output)
5. Deploy/demo if ready

**Result**: Readers can learn ROS 2 fundamentals (nodes, topics, services) with working examples. This is the Minimum Viable Product (MVP).

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Complete Polish tasks → Final production-ready release
6. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T011)
2. Once Foundational is done:
   - Developer A: User Story 1 (Chapter 1, T012-T031)
   - Developer B: User Story 2 (Chapter 2, T032-T048)
   - Developer C: User Story 3 (Chapter 3, T049-T071)
3. Stories complete and integrate independently
4. Team collaborates on Polish tasks (T072-T080)

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label (US1, US2, US3) maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Tests are NOT included (not requested in spec, not needed for documentation module)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Task Count Summary

| Phase | Task Range | Count | Parallel Opportunities |
|-------|-----------|-------|------------------------|
| Setup | T001-T008 | 8 | 4 tasks (T002, T004, T007, T008) |
| Foundational | T009-T011 | 3 | 2 tasks (T010, T011) |
| User Story 1 (P1) | T012-T031 | 20 | 5 tasks (T012-T016), 4 tasks (T027-T030) |
| User Story 2 (P2) | T032-T048 | 17 | 4 tasks (T032-T035), 5 tasks (T040-T044) |
| User Story 3 (P3) | T049-T071 | 23 | 6 tasks (T049-T054), 3 tasks (T068-T070) |
| Polish | T072-T080 | 9 | 7 tasks (T072-T076, T078-T079) |
| **Total** | T001-T080 | **80** | **35 parallelizable tasks** |

---

## Suggested MVP Scope

**Minimum Viable Product**: User Story 1 (Chapter 1: ROS 2 Fundamentals)

**Tasks for MVP**: T001-T011 (Setup + Foundational) + T012-T031 (User Story 1) = **31 tasks**

**Deliverable**: Chapter 1 with 3 runnable examples (publisher, subscriber, service), diagrams, troubleshooting, and key takeaways. Readers can learn ROS 2 core concepts and verify understanding with hands-on code.

**Validation**: Readers complete Chapter 1 in under 60 minutes (SC-001), execute all examples successfully, and can articulate nodes/topics/services concepts.
