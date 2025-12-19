# Tasks: Module 2 - The Digital Twin (Gazebo & Unity)

**Input**: Design documents from `/specs/002-digital-twin-simulation/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, quickstart.md ✅

**Tests**: No automated tests requested in specification. Module deliverables are educational content (Markdown documentation + example files) validated through manual testing on Ubuntu 22.04 + ROS 2 Humble.

**Organization**: Tasks are grouped by user story (Gazebo physics simulation P1, Unity rendering P2, Sensor simulation P3) to enable independent implementation and testing of each chapter.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This is a **documentation project** extending existing Docusaurus site:
- **Documentation**: `frontend_book/docs/module-2/`
- **Static assets**: `frontend_book/static/img/module-2/`, `frontend_book/static/examples/`
- **Configuration**: `frontend_book/sidebars.js`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create Module 2 directory structure and update Docusaurus navigation

- [x] T001 Create `frontend_book/docs/module-2/` directory for Module 2 documentation
- [x] T002 Create `frontend_book/static/img/module-2/` directory for diagrams and screenshots
- [x] T003 Create `frontend_book/static/examples/` directory for downloadable example files
- [x] T004 Update `frontend_book/sidebars.js` to add Module 2 navigation with 4 entries (index, chapter-1, chapter-2, chapter-3)
- [x] T005 Create placeholder files: `frontend_book/docs/module-2/index.md`, `chapter-1-gazebo-physics.md`, `chapter-2-unity-rendering.md`, `chapter-3-sensor-simulation.md`

**Checkpoint**: Directory structure ready, sidebar navigation configured

---

## Phase 2: Foundational (Module 2 Landing Page)

**Purpose**: Create module overview page that introduces all three chapters

**⚠️ CRITICAL**: This landing page must be complete before chapter content to establish learning objectives and prerequisites

- [x] T006 Write Module 2 landing page in `frontend_book/docs/module-2/index.md` (400-600 words) including:
  - Learning objectives (what students will achieve)
  - Prerequisites (Module 1 completion, simple_humanoid.urdf, ROS 2 Humble, Gazebo 11)
  - Module structure overview (3 chapters with time estimates and difficulty levels)
  - Installation quick links (Gazebo Classic 11, Unity 2021 LTS, ROS-TCP-Connector)

**Checkpoint**: Module landing page complete - chapter work can now begin in parallel

---

## Phase 3: User Story 1 - Physics-Based Robot Simulation in Gazebo (Priority: P1) 🎯 MVP

**Goal**: Students can simulate simple_humanoid in Gazebo with realistic physics (gravity, collisions, joint dynamics)

**Independent Test**:
1. Extract `gazebo_humanoid_world.zip` from `/static/examples/`
2. Run `ros2 launch launch/gazebo_demo.launch.py`
3. Verify: Gazebo opens, robot spawns, falls with gravity, lands on ground plane
4. Run `ros2 topic list` → verify `/joint_states`, `/tf`, `/clock` topics exist
5. Publish test velocity: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once`

### Example Files for User Story 1

- [x] T007 [P] [US1] Create example directory `frontend_book/static/examples/gazebo_humanoid_world/` with subdirectory `launch/`
- [x] T008 [P] [US1] Create `simple_humanoid_gazebo.urdf` in `frontend_book/static/examples/gazebo_humanoid_world/` extending Module 1 URDF with Gazebo physics properties (`<gazebo>` tags for friction mu1/mu2, material, kp/kd damping)
- [x] T009 [P] [US1] Create `simple_humanoid.world` Gazebo world file in `frontend_book/static/examples/gazebo_humanoid_world/` with ground plane, sun lighting, physics settings (gravity -9.81, update_rate 1000 Hz)
- [x] T010 [US1] Create `gazebo_demo.launch.py` ROS 2 launch file in `frontend_book/static/examples/gazebo_humanoid_world/launch/` that starts Gazebo server and spawns simple_humanoid robot
- [x] T011 [US1] Create `README.md` in `frontend_book/static/examples/gazebo_humanoid_world/` with prerequisites, usage instructions (ros2 launch command), verification steps, and troubleshooting for common errors
- [x] T012 [US1] Test Gazebo example locally: run `gazebo simple_humanoid.world`, verify robot loads, check physics behavior (gravity, ground collision)
- [x] T013 [US1] Validate URDF syntax with `check_urdf simple_humanoid_gazebo.urdf`, ensure kinematic tree is valid
- [x] T014 [US1] Create downloadable archive `frontend_book/static/examples/gazebo_humanoid_world.zip` using PowerShell Compress-Archive command

### Documentation for User Story 1

- [x] T015 [US1] Write Chapter 1 Section 1: "Introduction to Gazebo Classic" in `frontend_book/docs/module-2/chapter-1-gazebo-physics.md` (400-600 words) covering what Gazebo is, why use physics simulation, Gazebo Classic vs Gazebo Sim decision
- [x] T016 [US1] Write Chapter 1 Section 2: "Installing Gazebo Classic 11" (300-500 words) with Ubuntu 22.04 installation commands, version verification, gazebo_ros_pkgs setup
- [x] T017 [US1] Write Chapter 1 Section 3: "Gazebo World Files" (500-700 words) explaining XML structure, physics engine settings, ground plane, lighting, model spawning with annotated example from simple_humanoid.world
- [x] T018 [US1] Write Chapter 1 Section 4: "Adding Physics to URDF Models" (600-800 words) documenting `<gazebo>` tags, friction coefficients mu1/mu2, contact stiffness kp, damping kd, material properties with side-by-side comparison of base URDF vs Gazebo-enhanced URDF
- [x] T019 [US1] Write Chapter 1 Section 5: "Launching Gazebo with ROS 2" (400-600 words) explaining Python launch files, ExecuteProcess vs Node actions, spawning entities with gazebo_ros plugin, robot_state_publisher integration
- [x] T020 [US1] Write Chapter 1 Section 6: "Testing Physics Simulation" (300-500 words) with hands-on verification commands (ros2 topic list, ros2 topic echo /joint_states, publishing velocity commands), expected outputs
- [x] T021 [US1] Write Chapter 1 Section 7: "Troubleshooting" (400-600 words) addressing top 5 Gazebo errors from research.md (model sinking through ground, controller spawner errors, topic not publishing, Gazebo crashes, missing plugins) with causes and fixes
- [x] T022 [US1] Write Chapter 1 Section 8: "Key Takeaways" (200-300 words) summarizing main concepts: Gazebo world files, URDF physics extensions, launch file structure, ROS 2 integration
- [x] T023 [US1] Add download link to Chapter 1 for `gazebo_humanoid_world.zip` with Markdown: `📦 **[gazebo_humanoid_world.zip](/examples/gazebo_humanoid_world.zip)**`
- [x] T024 [P] [US1] Create Mermaid diagram "Gazebo Simulation Pipeline" showing ROS 2 → Gazebo → Physics Engine → Collision Detection → ROS 2 Topics data flow, save in Chapter 1
- [x] T025 [P] [US1] Create static image "Gazebo Physics Parameters" diagram (draw.io or Excalidraw) illustrating gravity vector, friction forces, collision geometry, save as `frontend_book/static/img/module-2/gazebo-physics-parameters.png`
- [x] T026 [US1] Verify Chapter 1 word count is 2500-4000 words (per spec FR-010), adjust content if needed
- [x] T027 [US1] Add at least 2 peer-reviewed paper citations to Chapter 1 References section (simulation validation, physics accuracy) in IEEE format with open-access links

**Checkpoint**: Chapter 1 complete (Gazebo physics simulation). Student can download example, launch Gazebo, verify physics, understand URDF extensions.

---

## Phase 4: User Story 2 - High-Fidelity Visualization with Unity (Priority: P2)

**Goal**: Students can render simple_humanoid in Unity with photorealistic materials, lighting, and ROS 2 bridge integration

**Independent Test**:
1. Extract `unity_humanoid_scene.zip` from `/static/examples/`
2. Open Unity 2021 LTS, load `UnityProject/` folder
3. Open `Assets/Scenes/HumanoidDemo.unity`
4. Press Play → verify robot renders with realistic lighting and shadows
5. In terminal: `ros2 run ros_tcp_endpoint default_server_endpoint`
6. In Unity Play mode: verify ROS connection status in Console (TCP connected to 127.0.0.1:10000)

### Example Files for User Story 2

- [ ] T028 [P] [US2] Create example directory `frontend_book/static/examples/unity_humanoid_scene/UnityProject/` with Unity folder structure (Assets/, Packages/, ProjectSettings/)
- [ ] T029 [P] [US2] Create Unity scene `HumanoidDemo.unity` in `frontend_book/static/examples/unity_humanoid_scene/UnityProject/Assets/Scenes/` with ground plane, directional light, main camera, and placeholder for humanoid robot
- [ ] T030 [P] [US2] Configure lighting in Unity scene: directional light intensity 1.5, soft shadows enabled, ambient source set to skybox, bake lighting settings documented
- [ ] T031 [P] [US2] Create Unity C# script `ROSCameraPublisher.cs` in `frontend_book/static/examples/unity_humanoid_scene/UnityProject/Assets/Scripts/` that captures camera images and publishes to ROS 2 topic `/camera/rgb/image_raw` using ROS-TCP-Connector
- [ ] T032 [P] [US2] Create Unity C# script `ROSJointSubscriber.cs` in `frontend_book/static/examples/unity_humanoid_scene/UnityProject/Assets/Scripts/` that subscribes to `/joint_states` topic and updates Unity articulation body joint positions
- [ ] T033 [US2] Configure `Packages/manifest.json` to include ROS-TCP-Connector dependency (com.unity.robotics.ros-tcp-connector version 0.7.0+)
- [ ] T034 [US2] Create Unity prefab for `simple_humanoid` in `frontend_book/static/examples/unity_humanoid_scene/UnityProject/Assets/Robots/simple_humanoid/` by importing URDF structure with Unity materials (PBR shaders, metallic/smoothness values)
- [ ] T035 [US2] Add humanoid prefab instance to HumanoidDemo.unity scene, position at (0, 1, 0), attach ArticulationBody component for joint simulation
- [ ] T036 [US2] Create `README.md` in `frontend_book/static/examples/unity_humanoid_scene/` with Unity installation instructions, ROS-TCP-Connector setup steps, scene usage guide, ROS 2 endpoint startup commands
- [ ] T037 [US2] Test Unity scene locally: open in Unity Editor, verify rendering quality (shadows, materials), test Play mode, check for Console errors
- [ ] T038 [US2] Test ROS-Unity bridge: start `ros2 run ros_tcp_endpoint default_server_endpoint`, run Unity scene, verify TCP connection established (check Unity Console logs)
- [ ] T039 [US2] Create downloadable archive `frontend_book/static/examples/unity_humanoid_scene.zip` (estimated 20-50 MB) using PowerShell Compress-Archive

### Documentation for User Story 2

- [ ] T040 [US2] Write Chapter 2 Section 1: "Introduction to Unity for Robotics" in `frontend_book/docs/module-2/chapter-2-unity-rendering.md` (400-600 words) explaining Unity's role in robotic visualization, Unity vs Gazebo use cases comparison table, when to choose Unity (HRI, perception, demos)
- [ ] T041 [US2] Write Chapter 2 Section 2: "Installing Unity 2021 LTS" (300-500 words) with download links for Windows/macOS/Linux, Unity Hub setup, creating new 3D project, ROS-TCP-Connector package installation via Package Manager
- [ ] T042 [US2] Write Chapter 2 Section 3: "Importing URDF into Unity" (500-700 words) documenting Unity URDF import tools, converting URDF links to GameObjects, setting up ArticulationBody components for joints, material assignment
- [ ] T043 [US2] Write Chapter 2 Section 4: "Configuring Photorealistic Rendering" (600-800 words) explaining Unity lighting systems (directional, ambient, baked), material properties (albedo, metallic, smoothness), shadow settings (quality, resolution, distance), post-processing effects
- [ ] T044 [US2] Write Chapter 2 Section 5: "Unity-ROS 2 Bridge Setup" (500-700 words) covering ROS-TCP-Connector architecture, TCP endpoint configuration (IP, port 10000), publisher/subscriber script examples (ROSCameraPublisher.cs, ROSJointSubscriber.cs), message type mapping
- [ ] T045 [US2] Write Chapter 2 Section 6: "Testing Unity Scene" (400-600 words) with step-by-step verification: starting ROS 2 TCP endpoint, running Unity Play mode, checking connection status, publishing test joint commands, viewing camera feed in RViz
- [ ] T046 [US2] Write Chapter 2 Section 7: "Troubleshooting" (400-600 words) addressing Unity-specific errors from research.md (ROS-TCP connection refused, URDF import failures, rendering performance issues) with causes and solutions
- [ ] T047 [US2] Write Chapter 2 Section 8: "Key Takeaways" (200-300 words) summarizing Unity rendering pipeline, ROS-Unity bridge architecture, when Unity adds value over Gazebo
- [ ] T048 [US2] Add download link to Chapter 2 for `unity_humanoid_scene.zip` with Markdown and note about optional nature (P2 priority, can skip if hardware limited)
- [ ] T049 [P] [US2] Create Mermaid diagram "Unity-ROS Bridge Architecture" showing ROS 2 Topics ← TCP/IP → ROS-TCP-Connector → Unity GameObjects bidirectional flow, save in Chapter 2
- [ ] T050 [P] [US2] Create static screenshots of Unity UI (Scene view, Inspector panel, Game view with rendered robot) with annotations, save as `frontend_book/static/img/module-2/unity-ui-overview.png`
- [ ] T051 [US2] Verify Chapter 2 word count is 2500-4000 words (per spec FR-010), adjust content if needed
- [ ] T052 [US2] Add at least 2 peer-reviewed paper citations to Chapter 2 References section (photorealistic rendering for robotics, HRI with virtual environments) in IEEE format with open-access links

**Checkpoint**: Chapter 2 complete (Unity rendering). Student can download example, import URDF, configure Unity scene, test ROS bridge, understand rendering vs physics trade-offs.

---

## Phase 5: User Story 3 - Realistic Sensor Simulation (LiDAR, Depth Cameras, IMUs) (Priority: P3)

**Goal**: Students can configure and validate simulated sensors (LiDAR, depth camera, IMU) in Gazebo with realistic noise models

**Independent Test**:
1. Extract `gazebo_sensors.zip` from `/static/examples/`
2. Run `ros2 launch launch/sensor_demo.launch.py`
3. Verify LiDAR: `ros2 topic hz /scan` → should show ~10 Hz update rate
4. Verify depth camera: `ros2 topic echo /camera/depth/image_raw --once` → check image dimensions
5. Verify IMU: `ros2 topic echo /imu --once` → check orientation quaternion, angular velocity, linear acceleration fields
6. Visualize in RViz: `rviz2`, add LaserScan display for /scan topic, verify ray visualization

### Example Files for User Story 3

- [ ] T053 [P] [US3] Create example directory `frontend_book/static/examples/gazebo_sensors/` with subdirectory `launch/`
- [ ] T054 [P] [US3] Create `humanoid_with_lidar.urdf` in `frontend_book/static/examples/gazebo_sensors/` extending simple_humanoid_gazebo.urdf with LiDAR sensor plugin (libgazebo_ros_ray.so) configured for 240° FOV, 4m range, 640 samples, gaussian noise stddev 0.01m
- [ ] T055 [P] [US3] Create `humanoid_with_depth_camera.urdf` in `frontend_book/static/examples/gazebo_sensors/` with depth camera plugin (libgazebo_ros_camera.so) configured for 1280x720 resolution, 87° FOV, 0.3-10m range, depth noise stddev 0.007m
- [ ] T056 [P] [US3] Create `humanoid_with_imu.urdf` in `frontend_book/static/examples/gazebo_sensors/` with IMU plugin (libgazebo_ros_imu.so) configured for quaternion orientation output, ±2000°/s gyro range, ±16g accel range, realistic bias drift parameters
- [ ] T057 [US3] Create `sensor_demo.launch.py` ROS 2 launch file in `frontend_book/static/examples/gazebo_sensors/launch/` that launches Gazebo with choice of sensor URDF (command-line argument: lidar, depth, or imu), spawns robot, and optionally starts RViz with appropriate display config
- [ ] T058 [US3] Create `README.md` in `frontend_book/static/examples/gazebo_sensors/` with sensor configuration guide, launch command examples for each sensor type, ROS 2 topic inspection commands, RViz visualization instructions
- [ ] T059 [US3] Test LiDAR sensor: launch with `humanoid_with_lidar.urdf`, place test obstacles at known distances in Gazebo, verify `/scan` topic publishes LaserScan messages with range accuracy <5cm (per spec SC-006)
- [ ] T060 [US3] Test depth camera: launch with `humanoid_with_depth_camera.urdf`, point camera at objects 0.5-5m away, verify depth image values match ground truth within 10% error (per spec SC-007)
- [ ] T061 [US3] Test IMU sensor: launch with `humanoid_with_imu.urdf`, move/rotate robot in Gazebo, verify `/imu` topic publishes Imu messages with orientation (quaternion), angular velocity, and linear acceleration
- [ ] T062 [US3] Create downloadable archive `frontend_book/static/examples/gazebo_sensors.zip` using PowerShell Compress-Archive

### Documentation for User Story 3

- [ ] T063 [US3] Write Chapter 3 Section 1: "Introduction to Sensor Simulation" in `frontend_book/docs/module-2/chapter-3-sensor-simulation.md` (400-600 words) explaining why simulate sensors (SLAM development, perception testing, cost savings), sensor types overview (LiDAR, depth, IMU), simulation fidelity considerations
- [ ] T064 [US3] Write Chapter 3 Section 2: "LiDAR Sensor Simulation" (600-800 words) covering Gazebo ray sensor plugin, horizontal/vertical FOV configuration, samples and angular resolution, range limits (min/max), gaussian noise model (mean, stddev), ROS 2 topic output (sensor_msgs/LaserScan), example configuration from humanoid_with_lidar.urdf
- [ ] T065 [US3] Write Chapter 3 Section 3: "Depth Camera Simulation" (600-800 words) documenting Gazebo camera plugin for depth images, image resolution settings, horizontal FOV, clipping planes (near/far), depth format (R16), noise parameters, camera intrinsics, ROS 2 topic output (sensor_msgs/Image), example from humanoid_with_depth_camera.urdf
- [ ] T066 [US3] Write Chapter 3 Section 4: "IMU Sensor Simulation" (500-700 words) explaining Gazebo IMU plugin, orientation output (quaternion vs Euler), angular velocity range (gyroscope), linear acceleration range (accelerometer), noise modeling (bias drift, random walk), ROS 2 topic output (sensor_msgs/Imu), example from humanoid_with_imu.urdf
- [ ] T067 [US3] Write Chapter 3 Section 5: "Validating Sensor Data" (500-700 words) with hands-on verification steps: LiDAR range accuracy test (place obstacle at 1m, verify scan range), depth camera calibration check (compare depth image to known distances), IMU orientation test (rotate robot, verify quaternion changes), RViz visualization for all sensor types
- [ ] T068 [US3] Write Chapter 3 Section 6: "Sensor Fusion Concepts" (400-600 words) introducing multi-sensor integration (LiDAR + depth + IMU for SLAM), synchronized data capture, coordinate frame transforms (/tf tree), example use case: autonomous navigation with fused sensor input
- [ ] T069 [US3] Write Chapter 3 Section 7: "Troubleshooting" (400-600 words) addressing sensor-specific errors from research.md (/scan topic not publishing, depth camera blank images, IMU drift unrealistic, sensor plugin version mismatches) with debugging commands and fixes
- [ ] T070 [US3] Write Chapter 3 Section 8: "Key Takeaways" (200-300 words) summarizing Gazebo sensor plugins, noise model configuration, ROS 2 topic interfaces, when to use each sensor type
- [ ] T071 [US3] Add download link to Chapter 3 for `gazebo_sensors.zip` with Markdown
- [ ] T072 [P] [US3] Create Mermaid diagram "Sensor Simulation Data Flow" showing URDF Sensor Plugin → Gazebo Physics → Ray Tracing/Camera Rendering → Noise Model → ROS 2 Topic Publisher pipeline, save in Chapter 3
- [ ] T073 [P] [US3] Create static image "LiDAR Ray Casting Visualization" (draw.io) showing robot with LiDAR rays, obstacles at various distances, ray hit points, save as `frontend_book/static/img/module-2/lidar-ray-casting.png`
- [ ] T074 [P] [US3] Create static image "Depth Camera Frustum" (3D diagram) illustrating camera FOV, near/far clipping planes, depth image pixel mapping, save as `frontend_book/static/img/module-2/depth-camera-frustum.png`
- [ ] T075 [US3] Verify Chapter 3 word count is 2500-4000 words (per spec FR-010), adjust content if needed
- [ ] T076 [US3] Add at least 2 peer-reviewed paper citations to Chapter 3 References section (sensor noise modeling, LiDAR simulation accuracy) in IEEE format with open-access links

**Checkpoint**: Chapter 3 complete (Sensor simulation). Student can download examples, configure LiDAR/depth/IMU sensors, validate accuracy, visualize in RViz, understand noise models.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final integration, testing, and quality assurance for Module 2

- [ ] T077 [P] Verify all download links work: test `/examples/gazebo_humanoid_world.zip`, `/examples/unity_humanoid_scene.zip`, `/examples/gazebo_sensors.zip` from Docusaurus site
- [ ] T078 [P] Run Docusaurus build: `cd frontend_book && npm run build`, verify no broken links, no missing images, Module 2 pages render correctly
- [ ] T079 [P] Proofread all three chapters for spelling, grammar, clarity, technical accuracy using Grammarly or similar tool
- [ ] T080 [P] Verify all chapters cite official documentation sources (Gazebo Classic tutorials, Unity-ROS Hub, gazebo_plugins docs) with working hyperlinks
- [ ] T081 [P] Verify peer-reviewed paper citations: confirm 2 papers per chapter (6 total), all open-access links work, IEEE format correct
- [ ] T082 Test complete Module 2 user journey on clean Ubuntu 22.04 VM: extract examples, follow Chapter 1 tutorial, verify Gazebo physics works, repeat for Chapters 2-3
- [ ] T083 Run performance validation: Gazebo example runs at 20+ FPS on 8GB RAM laptop (per spec SC-002), Unity example runs at 30+ FPS with GPU (per spec SC-004)
- [ ] T084 Create 3-5 practice exercises per chapter (per spec FR-015): Chapter 1 - "Modify gravity to simulate Moon environment", Chapter 2 - "Add camera to robot head in Unity", Chapter 3 - "Configure LiDAR for 180° FOV"
- [ ] T085 Verify Module 2 landing page (index.md) accurately reflects final chapter content, update time estimates if needed
- [ ] T086 [P] Update main Docusaurus homepage to highlight Module 2 availability if not already promoted
- [ ] T087 Run final Docusaurus build and serve locally: `npm run build && npm run serve`, manually test all navigation, downloads, diagrams
- [ ] T088 Commit all Module 2 files with descriptive message: "feat: Complete Module 2 - The Digital Twin (Gazebo & Unity)"

**Checkpoint**: Module 2 fully complete, tested, and ready for deployment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - creates module landing page
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (Gazebo - P1): MVP, can proceed independently after Phase 2
  - User Story 2 (Unity - P2): Can proceed independently after Phase 2, optional for students with hardware limitations
  - User Story 3 (Sensors - P3): Can proceed independently after Phase 2, builds on Gazebo knowledge from US1
- **Polish (Phase 6)**: Depends on all three user stories being complete

### User Story Dependencies

- **User Story 1 (P1 - Gazebo)**: Can start after Foundational (Phase 2) - No dependencies on other stories - **MVP SCOPE**
- **User Story 2 (P2 - Unity)**: Can start after Foundational (Phase 2) - Independent of US1/US3 (different simulation tool)
- **User Story 3 (P3 - Sensors)**: Can start after Foundational (Phase 2) - Independent but conceptually builds on Gazebo physics from US1 (sensors run in Gazebo)

### Within Each User Story

- Example files before documentation (content writers need working examples to reference)
- URDF files before launch files (launch files reference URDFs)
- Directory creation before file creation
- Local testing before creating .zip archives
- Documentation sections can be written in parallel (different authors)
- Diagrams can be created in parallel with text content
- Final validation (word count, citations) after all sections written

### Parallel Opportunities

**Phase 1 (Setup)**: All 5 tasks can run in parallel (different directories/files)

**Phase 2 (Foundational)**: Single task, no parallelization

**Phase 3 (User Story 1 - Gazebo)**:
- T007-T014: Example file creation can run in parallel (different files: URDF, world, launch, README)
- T024-T025: Diagrams can be created in parallel with documentation writing
- T015-T023: Documentation sections can be written in parallel if multiple contributors (Section 1 vs Section 2 vs Section 3...)

**Phase 4 (User Story 2 - Unity)**:
- T028-T039: Unity scene setup, C# scripts, prefab creation can run in parallel (different files)
- T049-T050: Diagrams can be created in parallel with documentation
- T040-T048: Documentation sections can be written in parallel

**Phase 5 (User Story 3 - Sensors)**:
- T054-T056: Three sensor URDFs can be created in parallel (different sensor types)
- T072-T074: Three diagrams (data flow, LiDAR, depth camera) can be created in parallel
- T063-T071: Documentation sections can be written in parallel

**Phase 6 (Polish)**: T077-T081 can run in parallel (different validation tasks)

**ENTIRE USER STORIES**: US1, US2, US3 can run in parallel after Phase 2 completes (different chapters, different example files)

---

## Parallel Example: User Story 1 (Gazebo Physics)

```bash
# Launch all example file creation tasks together:
Task: "Create simple_humanoid_gazebo.urdf with Gazebo physics properties"
Task: "Create simple_humanoid.world Gazebo world file"
Task: "Create README.md in gazebo_humanoid_world/"

# Launch diagram creation in parallel with documentation:
Task: "Create Mermaid diagram Gazebo Simulation Pipeline"
Task: "Create static image Gazebo Physics Parameters diagram"

# Launch documentation sections in parallel (if multiple writers):
Task: "Write Chapter 1 Section 1: Introduction to Gazebo Classic"
Task: "Write Chapter 1 Section 3: Gazebo World Files"
Task: "Write Chapter 1 Section 4: Adding Physics to URDF Models"
```

---

## Parallel Example: All User Stories After Foundational

```bash
# Once Phase 2 (Module landing page) is complete, all user stories can start:
Task Group 1: "User Story 1 - Gazebo Physics (Chapter 1)" → T007-T027
Task Group 2: "User Story 2 - Unity Rendering (Chapter 2)" → T028-T052
Task Group 3: "User Story 3 - Sensor Simulation (Chapter 3)" → T053-T076

# This enables 3-person team to work in parallel:
# - Developer A: Chapter 1 (Gazebo examples + documentation)
# - Developer B: Chapter 2 (Unity scene + documentation)
# - Developer C: Chapter 3 (Sensor URDFs + documentation)
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T005) → 5 tasks
2. Complete Phase 2: Foundational (T006) → 1 task
3. Complete Phase 3: User Story 1 - Gazebo (T007-T027) → 21 tasks
4. **STOP and VALIDATE**:
   - Test `gazebo_humanoid_world.zip` extraction and launch
   - Verify Chapter 1 renders in Docusaurus
   - Verify word count 2500-4000
   - Confirm student can follow tutorial end-to-end
5. Deploy MVP (Module 2 with Chapter 1 only) if ready

**MVP Task Count**: 27 tasks total for minimal viable module

### Incremental Delivery

1. Complete Setup + Foundational → Module structure ready (6 tasks)
2. Add User Story 1 (Gazebo) → Test independently → **Deploy MVP** (27 tasks total)
3. Add User Story 2 (Unity) → Test independently → Deploy updated module (52 tasks total)
4. Add User Story 3 (Sensors) → Test independently → Deploy complete module (76 tasks total)
5. Polish & Cross-Cutting → Final quality pass → **Deploy production-ready module** (88 tasks total)

Each increment adds value without breaking previous content.

### Parallel Team Strategy

With 3 developers after completing Setup + Foundational:

1. Team completes Phase 1 + Phase 2 together (6 tasks)
2. Once Foundational complete (module landing page exists):
   - **Developer A**: User Story 1 - Gazebo (T007-T027) → 21 tasks
   - **Developer B**: User Story 2 - Unity (T028-T052) → 25 tasks
   - **Developer C**: User Story 3 - Sensors (T053-T076) → 24 tasks
3. All three chapters complete independently, then integrate
4. Team collaborates on Phase 6: Polish (T077-T088) → 12 tasks

**Total Time Saved**: Approximately 66% reduction vs sequential implementation

---

## Task Summary

**Total Tasks**: 88

**Breakdown by Phase**:
- Phase 1 (Setup): 5 tasks
- Phase 2 (Foundational): 1 task
- Phase 3 (User Story 1 - Gazebo): 21 tasks
- Phase 4 (User Story 2 - Unity): 25 tasks
- Phase 5 (User Story 3 - Sensors): 24 tasks
- Phase 6 (Polish): 12 tasks

**Breakdown by User Story**:
- User Story 1 (P1 - Gazebo Physics): 21 tasks (MVP scope)
- User Story 2 (P2 - Unity Rendering): 25 tasks (optional for students with hardware limits)
- User Story 3 (P3 - Sensor Simulation): 24 tasks (builds on Gazebo from US1)

**Parallel Opportunities Identified**:
- 15+ tasks marked [P] can run in parallel within phases
- All 3 user stories (70 tasks) can run in parallel after Foundational phase
- Diagrams can be created in parallel with documentation writing
- Documentation sections within each chapter can be parallelized

**Independent Test Criteria**:
- ✅ User Story 1: Extract Gazebo example, launch, verify physics, test ROS 2 topics
- ✅ User Story 2: Open Unity scene, verify rendering quality, test ROS bridge connection
- ✅ User Story 3: Launch sensor examples, verify topic publishing, check accuracy in RViz

**Suggested MVP Scope**: User Story 1 only (Gazebo Physics simulation) = 27 tasks

---

## Notes

- [P] tasks = different files, no dependencies within phase
- [Story] label maps task to specific user story for traceability (US1, US2, US3)
- Each user story delivers a complete, independently testable chapter
- No automated tests requested - validation through manual testing on Ubuntu 22.04 + ROS 2 Humble
- Example files created before documentation (writers need working examples to reference)
- Word count validation per chapter: 2500-4000 words (spec FR-010)
- Citations required: 2 peer-reviewed papers per chapter minimum (spec FR-012)
- Commit after each logical group (e.g., after completing one chapter's examples, after finishing one documentation section)
- Stop at any checkpoint to validate chapter independently before proceeding
