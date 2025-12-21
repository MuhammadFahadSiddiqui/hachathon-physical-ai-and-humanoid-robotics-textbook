# Implementation Plan: Module 3 - The AI-Robot Brain (NVIDIA Isaac™)

**Branch**: `003-isaac-ai-brain` | **Date**: 2025-12-19 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/003-isaac-ai-brain/spec.md`

## Summary

Create Module 3 educational content covering NVIDIA's Isaac ecosystem for advanced robotics perception, simulation, and navigation. The module consists of 3 technical chapters (2500-4000 words each) as Markdown files in Docusaurus format, covering Isaac Sim photorealistic simulation, Isaac ROS GPU-accelerated perception, and Nav2 bipedal navigation. Includes runnable examples (.zip downloads), practice exercises, peer-reviewed citations, and troubleshooting sections. Target audience: students who have completed Module 1 (ROS 2 fundamentals) and Module 2 (Gazebo/Unity simulation) and have access to NVIDIA RTX GPU hardware.

## Technical Context

**Content Format**: Markdown (.md) with Docusaurus frontmatter (YAML metadata)
**Primary Technologies**: NVIDIA Isaac Sim 2023.1.1, Isaac ROS humble, ROS 2 Humble, Nav2, CUDA 12.2
**Target Platform**: Ubuntu 22.04 LTS with NVIDIA RTX GPU (RTX 2060+ recommended, 8GB+ VRAM)
**Example Code Languages**: Python 3.10+ (ROS 2 nodes, launch files), YAML (Nav2 configs), USD (Isaac Sim scenes)
**Testing**: Manual validation of examples on RTX 3060 (12GB VRAM) baseline hardware
**Performance Goals**: Isaac Sim 30+ FPS rendering, Isaac ROS Visual SLAM 30 Hz, Nav2 navigation &lt;30s for 5m goal
**Constraints**:
- 2500-4000 words per chapter
- 2+ peer-reviewed citations per chapter (ICRA/IROS/RSS/T-RO/IJRR 2018+)
- All examples must run on free/open-source software (Isaac Sim free tier, ROS 2 open-source)
- RTX GPU hardware required (document CPU fallback limitations)
**Scale/Scope**: 3 chapters, 9-15 practice exercises, 3 downloadable .zip archives (&lt;50 MB each)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Spec-Driven, AI-First Development

✅ **PASS**: Feature was specified via `/sp.specify` with complete user stories, requirements, and success criteria. Planning via `/sp.plan` in progress. Tasks will be generated via `/sp.tasks` before implementation.

### Principle II: Accuracy and Content-Grounded Responses

✅ **PASS** (adapted): While this principle applies to RAG chatbot behavior (not applicable to Module 3), the equivalent for educational content is **factual accuracy**. All Isaac Sim/Isaac ROS/Nav2 content will be sourced from official NVIDIA documentation and peer-reviewed papers, not speculation or outdated information.

### Principle III: Clear, Developer-Focused Writing

✅ **PASS**: Spec defines success criteria for clarity (SC-008: 80% exercise completion rate, SC-009: 90% positive feedback on prerequisites). Each chapter includes:
- Conceptual explanations of Isaac ecosystem
- Runnable code examples (Isaac Sim Python scripts, Isaac ROS launch files, Nav2 YAML configs)
- Step-by-step tutorials for installation and setup
- Troubleshooting sections addressing common GPU/CUDA errors

### Principle IV: Reproducible and Production-Ready Setup

✅ **PASS**: All examples will be tested on RTX 3060 baseline hardware. Downloadable .zip archives include pre-configured scenes and launch files. Installation instructions validated on clean Ubuntu 22.04 environments. Environment variables (CUDA paths, ROS workspaces) documented in setup sections.

### Principle V: Free-Tier and Open-Source Compliant

✅ **PASS**:
- **Isaac Sim**: Free tier available (Omniverse Individual license, no cost for educational use)
- **Isaac ROS**: Open-source Apache 2.0 license
- **ROS 2 Humble**: Open-source (Apache 2.0)
- **Nav2**: Open-source (Apache 2.0)
- **CUDA 12.2**: Free NVIDIA toolkit (requires RTX GPU hardware, but software is free)
- **Pre-trained Models**: YOLOv8 COCO weights (open-source)

⚠️ **HARDWARE REQUIREMENT DISCLAIMER**: RTX GPU (RTX 2060+) is required hardware. While the GPU itself costs money, this is documented as a prerequisite, not a paid software service. CPU alternatives are explicitly noted as non-viable for GPU-accelerated perception (aligned with educational honesty, not paywalling features).

**Re-check after Phase 1**: Verify no paid services introduced during research/design phase.

## Project Structure

### Documentation (this feature)

```text
specs/003-isaac-ai-brain/
├── plan.md              # This file (/sp.plan output)
├── research.md          # Phase 0: Technology research, best practices
├── data-model.md        # Phase 1: Learning outcome model, chapter structure
├── quickstart.md        # Phase 1: Module 3 setup quick reference
├── contracts/           # Phase 1: Example code contracts (Python type hints, YAML schemas)
└── tasks.md             # Phase 2: /sp.tasks output (implementation checklist)
```

### Content Files (repository)

```text
frontend_book/docs/module-3/
├── index.md                          # Module 3 landing page (learning objectives, prerequisites, hardware requirements)
├── chapter-1-isaac-sim.md            # Chapter 1: Isaac Sim photorealistic simulation (2500-4000 words)
├── chapter-2-isaac-ros.md            # Chapter 2: Isaac ROS GPU perception (2500-4000 words)
└── chapter-3-nav2-bipedal.md         # Chapter 3: Nav2 bipedal navigation (2500-4000 words)

frontend_book/static/examples/
├── isaac_sim_humanoid/               # Isaac Sim example (exported to .zip)
│   ├── humanoid_warehouse.usd        # Pre-configured Isaac Sim scene
│   ├── scripts/
│   │   ├── import_urdf.py            # URDF import script
│   │   ├── generate_synthetic_data.py # Synthetic data generation
│   │   └── export_coco.py            # COCO format export
│   └── README.md                     # Setup and usage instructions
│
├── isaac_ros_perception/             # Isaac ROS example (exported to .zip)
│   ├── launch/
│   │   ├── visual_slam.launch.py     # Visual SLAM node launch
│   │   ├── dnn_inference.launch.py   # YOLOv8 inference launch
│   │   └── stereo_depth.launch.py    # Stereo depth processing
│   ├── config/
│   │   └── visual_slam_params.yaml   # SLAM parameters
│   ├── models/
│   │   └── yolov8_coco.engine        # TensorRT engine (or download script)
│   └── README.md                     # Installation and RViz visualization
│
└── nav2_bipedal_humanoid/            # Nav2 example (exported to .zip)
    ├── launch/
    │   └── nav2_humanoid.launch.py   # Nav2 bringup for bipedal robot
    ├── config/
    │   ├── nav2_params.yaml          # Nav2 configuration
    │   ├── footstep_planner.yaml     # Bipedal footstep planner params
    │   └── costmap_params.yaml       # Costmap configuration
    ├── urdf/
    │   └── simple_humanoid_nav.urdf  # Humanoid with LiDAR sensor
    └── README.md                     # Launch instructions and RViz setup
```

**Structure Decision**: Educational content creation, not software development. Structure follows existing Module 1 and Module 2 patterns:
- Chapter Markdown files in `frontend_book/docs/module-3/`
- Example code in `frontend_book/static/examples/` with subdirectories per chapter
- Each example includes README with setup instructions, troubleshooting, and validation steps
- .zip archives created from example directories for user download

## Complexity Tracking

> No Constitution violations. All checks passed. This section intentionally left empty.

---

## Phase 0: Research & Technology Decisions

**Objective**: Resolve all unknowns from Technical Context, research best practices for Isaac Sim/Isaac ROS/Nav2, consolidate findings in `research.md`.

### Research Tasks

1. **Isaac Sim Installation and Licensing**
   - **Unknown**: Verify Isaac Sim 2023.1.1 free tier availability for educational use
   - **Research**: NVIDIA Omniverse Individual license terms, Isaac Sim download process for Ubuntu 22.04
   - **Outcome**: Document installation steps, license requirements, GPU driver/CUDA dependencies

2. **Isaac Sim Synthetic Data Generation Best Practices**
   - **Unknown**: Optimal configuration for photorealistic warehouse scenes (lighting, materials, camera parameters)
   - **Research**: NVIDIA Isaac Sim documentation on domain randomization, COCO export formats
   - **Outcome**: Document PBR material setup, camera intrinsics configuration, ground-truth annotation export

3. **Isaac ROS Installation and Dependencies**
   - **Unknown**: Isaac ROS humble branch installation procedure, CUDA 12.2 compatibility, TensorRT version requirements
   - **Research**: NVIDIA Isaac ROS GitHub repos (isaac_ros_common, isaac_ros_visual_slam, isaac_ros_dnn_inference)
   - **Outcome**: Document Docker container option vs. native installation, dependency resolution

4. **Isaac ROS Performance Benchmarking**
   - **Unknown**: Realistic FPS expectations for Visual SLAM, object detection, stereo depth on RTX 3060
   - **Research**: NVIDIA Isaac ROS performance documentation, community benchmarks on different RTX GPUs
   - **Outcome**: Document expected performance metrics, VRAM usage, CPU vs GPU speedup ratios

5. **Nav2 Bipedal Planner Configuration**
   - **Unknown**: Availability of bipedal footstep planners in Nav2, or need for custom planner plugin
   - **Research**: Nav2 planner plugins (NavFn, Smac, TEB), humanoid robotics navigation papers
   - **Outcome**: Determine whether to use simplified Nav2 DWB planner with bipedal footprint or recommend third-party footstep planner

6. **Peer-Reviewed Paper Selection**
   - **Unknown**: Specific papers for synthetic data (Chapter 1), GPU SLAM (Chapter 2), bipedal planning (Chapter 3)
   - **Research**: IEEE Xplore, Google Scholar searches for recent ICRA/IROS/T-RO papers (2018+)
   - **Outcome**: Select 2+ papers per chapter, verify open-access availability or provide DOI links

7. **Example Testing Hardware Requirements**
   - **Unknown**: Minimum RTX GPU for Isaac Sim 30 FPS, minimum VRAM for Isaac ROS Visual SLAM
   - **Research**: NVIDIA Isaac Sim system requirements, Isaac ROS memory profiling
   - **Outcome**: Document minimum (RTX 2060, 6GB VRAM) vs. recommended (RTX 3060, 12GB VRAM) specs

### Research Agent Dispatch

For each research task above, launch specialized research agents or perform direct documentation lookups:

- **Isaac Sim**: Review NVIDIA Omniverse documentation, Isaac Sim user manual (2023.1.1 version)
- **Isaac ROS**: Review GitHub repos (https://github.com/NVIDIA-ISAAC-ROS), check installation tutorials
- **Nav2**: Review ROS 2 Navigation documentation, search for bipedal planner plugins
- **Papers**: Search IEEE Xplore, arXiv for "synthetic data robotics", "GPU accelerated SLAM", "bipedal motion planning"

**Output**: `research.md` with consolidated findings, decisions, and rationale for all unknowns.

---

## Phase 1: Design & Content Outline

**Prerequisites**: `research.md` complete with all unknowns resolved.

### 1. Extract Learning Outcomes → `data-model.md`

Instead of software entities, Module 3 has **learning outcomes** as the "data model":

**Learning Outcome Model**:
- **Outcome ID**: LO-M3-### (e.g., LO-M3-001)
- **Chapter**: Isaac Sim / Isaac ROS / Nav2
- **Description**: What students will be able to do (e.g., "Install Isaac Sim and import URDF")
- **Prerequisites**: Prior learning outcomes (Module 1 ROS 2, Module 2 sensors)
- **Validation Method**: Practice exercise, success criteria metric (e.g., SC-001)
- **Code Examples**: Python scripts, YAML configs, launch files

**Chapter Structure Model**:
- **Chapter Title**: Descriptive name (e.g., "Isaac Sim - Photorealistic Simulation")
- **Word Count**: 2500-4000 words
- **Sections**: Introduction, Installation, Core Concepts, Runnable Example, Practice Exercises (3-5), Troubleshooting, References (2+ papers)
- **Downloadable Example**: .zip archive with pre-configured files
- **Success Criteria**: Mapped from spec (e.g., SC-001, SC-002 for Chapter 1)

`data-model.md` will document:
- Learning outcome progression (LO-M3-001 → LO-M3-002 → ...)
- Chapter content structure (sections, word count distribution)
- Example code structure (Python modules, YAML schemas, USD scene components)

### 2. Generate Code Contracts → `/contracts/`

Educational content "contracts" are **example code interfaces**:

**Python Type Hints** (for Isaac Sim/Isaac ROS scripts):
```python
# contracts/isaac_sim_api.py
from typing import Dict, List, Tuple
from pathlib import Path

def import_urdf_to_scene(urdf_path: Path, scene_path: Path) -> bool:
    """Import URDF robot into Isaac Sim scene.

    Args:
        urdf_path: Absolute path to simple_humanoid.urdf
        scene_path: Absolute path to output .usd scene file

    Returns:
        True if import successful, False otherwise
    """
    pass

def generate_synthetic_data(
    scene_path: Path,
    output_dir: Path,
    num_images: int,
    camera_config: Dict[str, float]
) -> List[Path]:
    """Generate synthetic RGB/depth/segmentation images.

    Args:
        scene_path: Isaac Sim .usd scene file
        output_dir: Directory to save images
        num_images: Number of images to generate
        camera_config: Camera intrinsics (width, height, fov, etc.)

    Returns:
        List of generated image file paths
    """
    pass
```

**YAML Schemas** (for Nav2 configuration):
```yaml
# contracts/nav2_params_schema.yaml
footstep_planner:
  step_length: float  # meters (e.g., 0.4)
  step_width: float   # meters (e.g., 0.2)
  step_height: float  # meters (e.g., 0.15)
  swing_time: float   # seconds (e.g., 0.8)

robot_footprint:
  type: string  # "circle" or "polygon"
  radius: float # meters (for circle type, e.g., 0.3)

costmap:
  resolution: float   # meters/cell (e.g., 0.05)
  inflation_radius: float  # meters (e.g., 0.55)
```

`/contracts/` directory will contain:
- `isaac_sim_api.py` - Python type stubs for Isaac Sim example scripts
- `isaac_ros_api.py` - Python type stubs for Isaac ROS launch files
- `nav2_params_schema.yaml` - YAML schema for Nav2 configuration

### 3. Generate Quickstart Guide → `quickstart.md`

Module 3 setup reference for students who want to jump straight to examples:

**Content**:
- **Prerequisites Checklist**: Ubuntu 22.04, RTX GPU (nvidia-smi output check), ROS 2 Humble sourced, Module 1 & 2 completed
- **Installation Steps**: Isaac Sim 2023.1.1 (Omniverse Launcher), CUDA 12.2, Isaac ROS docker container
- **Validation Commands**: `nvidia-smi`, `ros2 --version`, `isaac-sim --version`
- **Quick Test**: Launch Isaac Sim, load empty scene, verify 30+ FPS
- **Example Downloads**: Links to 3 .zip archives with one-command launch instructions
- **Troubleshooting Quick Fixes**: GPU driver issues, CUDA path not found, ROS 2 workspace not sourced

`quickstart.md` will be standalone reference for students who want minimal setup instructions.

### 4. Agent Context Update

Run agent context update script to add Isaac Sim, Isaac ROS, Nav2 technologies to Claude's context file:

```bash
.specify/scripts/powershell/update-agent-context.ps1 -AgentType claude
```

**Technologies to Add**:
- NVIDIA Isaac Sim 2023.1.1 (Omniverse-based robot simulator)
- NVIDIA Isaac ROS humble (GPU-accelerated ROS 2 packages)
- Nav2 (ROS 2 navigation stack for autonomous robots)
- CUDA 12.2 (NVIDIA GPU computing toolkit)
- TensorRT (NVIDIA deep learning inference optimizer)
- YOLOv8 (object detection model architecture)

**Output**: Updated `.claude/settings.local.json` or equivalent agent-specific context file.

---

## Phase 2: Not Executed (Delegated to `/sp.tasks`)

This plan terminates after Phase 1 (Design & Content Outline). The `/sp.tasks` command will generate the implementation task breakdown based on this plan.

**Expected `/sp.tasks` Output**:
- Task list for writing Chapter 1 (Isaac Sim) - 8-12 tasks
- Task list for writing Chapter 2 (Isaac ROS) - 8-12 tasks
- Task list for writing Chapter 3 (Nav2) - 8-12 tasks
- Task list for creating example .zip archives - 3-6 tasks
- Task list for Module 3 landing page and integration - 2-4 tasks
- Task list for validation and final polish - 4-6 tasks

**Total Estimated Tasks**: 35-50 tasks (similar scope to Module 2 which had 88 tasks across 6 phases).

---

## Post-Phase-1 Constitution Re-Check

### Principle I: Spec-Driven, AI-First Development

✅ **PASS**: Plan completed via `/sp.plan`. Next phase is `/sp.tasks` for task breakdown, followed by `/sp.implement` for execution. Workflow followed correctly.

### Principle II: Accuracy and Content-Grounded Responses (Educational Accuracy)

✅ **PASS**: Research phase will source all content from official NVIDIA Isaac documentation and peer-reviewed papers. No speculative or outdated information will be included.

### Principle III: Clear, Developer-Focused Writing

✅ **PASS**: `data-model.md` learning outcome model ensures each chapter has runnable examples, practice exercises, and troubleshooting sections. Code contracts define clear interfaces for example scripts.

### Principle IV: Reproducible and Production-Ready Setup

✅ **PASS**: `quickstart.md` provides validation commands and environment checks. All examples will be tested on RTX 3060 baseline hardware before release.

### Principle V: Free-Tier and Open-Source Compliant

✅ **PASS**: All software is free/open-source (Isaac Sim free tier, Isaac ROS Apache 2.0, ROS 2 open-source, Nav2 open-source). RTX GPU hardware requirement is documented as prerequisite, not paywalled feature.

**Final Verdict**: Constitution compliant. No violations introduced during planning phase.

---

## Risks & Mitigation

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Isaac Sim 2023.1.1 API changes in future versions | High (examples break) | Medium | Document exact version requirement, provide version-pinned installation instructions |
| RTX GPU unavailable to students | High (cannot complete module) | Medium | Clearly document RTX requirement in prerequisites, suggest cloud GPU alternatives (Google Colab, Paperspace) as fallback |
| Isaac ROS installation complexity (CUDA/TensorRT dependencies) | High (setup failures) | High | Provide Docker container option as primary installation method, native install as advanced option |
| Nav2 lacks mature bipedal footstep planner | Medium (simplified example) | High | Use Nav2 DWB planner with circular footprint, document limitation, link to advanced footstep planning research |
| Synthetic-to-real domain gap (Isaac Sim data doesn't generalize) | Medium (model performance) | Medium | Chapter 1 discusses domain randomization techniques, set realistic expectations about sim-to-real transfer |
| Peer-reviewed papers behind paywalls | Medium (citation access) | Low | Select open-access papers from arXiv or conference websites when possible, provide DOI for institutional access |

---

## Next Steps

1. **Execute Phase 0**: Research all unknowns, document findings in `research.md`
2. **Execute Phase 1**: Generate `data-model.md`, `/contracts/`, `quickstart.md`, update agent context
3. **Run `/sp.tasks`**: Generate implementation task breakdown
4. **Execute Implementation**: Write 3 chapters, create examples, test on RTX hardware
5. **Quality Assurance**: Validate word counts, citations, example functionality, build Docusaurus site
6. **Commit & Publish**: Merge to main branch, deploy to GitHub Pages

**Estimated Timeline** (if implemented):
- Phase 0 Research: 2-4 hours (documentation review, paper selection)
- Phase 1 Design: 2-3 hours (learning outcome model, code contracts, quickstart guide)
- Chapter 1 Implementation: 6-8 hours (2500-4000 words + Isaac Sim example)
- Chapter 2 Implementation: 6-8 hours (2500-4000 words + Isaac ROS example)
- Chapter 3 Implementation: 6-8 hours (2500-4000 words + Nav2 example)
- Testing & Polish: 4-6 hours (validate examples, proofread, build checks)

**Total Effort**: 26-37 hours (similar to Module 2 scope).

---

**Plan Complete**. Ready for Phase 0 execution or `/sp.tasks` command to generate task breakdown.
