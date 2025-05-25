# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context
**Dual-language ROS 2 humble template** with C++ and Python implementations, Docker containerization, and Foxglove visualization. Production-ready template for robotics development.

## Architecture
- **Languages**: C++ and Python (equivalent functionality)
- **Framework**: ROS 2 humble with colcon build system
- **Containers**: Docker multi-stage build with ros:humble-ros-base
- **Networking**: Host networking for ROS communication
- **Visualization**: Foxglove bridge (WebSocket port 8765)
- **Deployment**: 4 Docker Compose profiles (default, python, dual, multi-node)

## Quick Commands
```bash
# Build workspace
colcon build --symlink-install && source install/setup.bash

# Docker deployment options
docker compose up                    # C++ only (default)
docker compose --profile python up  # Python only
docker compose --profile dual up    # Both languages

# Manual Docker build
docker build -t ros-template:humble .

# Launch nodes individually
ros2 launch ros_template_node template.launch.py      # C++
ros2 launch py_template_node py_template.launch.py    # Python
```

## Package Structure
```
src/
├── ros_template_node/     # C++ package (ament_cmake)
└── py_template_node/      # Python package (ament_python)
```

## Development Rules
- **File Size**: Max 500 lines; refactor if needed
- **Dependencies**: Add to package.xml + CMakeLists.txt (C++) or setup.py (Python)
- **Docker**: Rebuild image after dependency changes
- **Testing**: Verify both languages work independently and together

## Language-Specific Patterns

### C++ (`ros_template_node`)
- Use modern C++17: `auto`, smart pointers, range-based loops
- RAII for resource management
- Default topic prefix: `template`, rate: 2.0 Hz

### Python (`py_template_node`)  
- Follow PEP 8 style guidelines
- Use type hints where beneficial
- Default topic prefix: `py_template`, rate: 1.0 Hz

## Workflow Rules
### 📋 Task Management
- Read `PLANNING.md` and `TASK.md` before starting work
- Add new tasks to `TASK.md` with date stamps
- Mark tasks complete immediately after finishing

### 🔧 Code Changes
- Test both C++ and Python implementations when making changes
- Update documentation for new features or dependencies
- Verify all Docker Compose profiles work after modifications

### 🐳 Docker Best Practices
- Use `--network host` for ROS communication
- Install dependencies in Dockerfile, not running containers
- Test image builds after package changes

### 📚 Documentation
- Update README.md for new features or setup changes
- Comment complex logic explaining "why", not "what"
- Ensure mid-level developers can understand the code

### 🚨 Critical Rules
- Never assume library availability - verify first
- Confirm file paths exist before referencing
- Ask questions if context is unclear
- Don't delete code unless explicitly instructed

## Basic Rules
### 🔄 Project Awareness & Context
- **Always read `PLANNING.md`** at the start of a new conversation to understand the project's architecture, goals, style, and constraints.
- **Check `TASK.md`** before starting a new task. If the task isn't listed, add it with a brief description and today's date.
- **Use consistent naming conventions, file structure, and architecture patterns** as described in `PLANNING.md`.

### 🧱 Code Structure & Modularity
- **Never create a file longer than 500 lines of code.** If a file approaches this limit, refactor by splitting it into modules or helper files.
- **Organize code into clearly separated modules**, grouped by feature or responsibility.

### ✅ Task Completion
- **Mark completed tasks in `TASK.md`** immediately after finishing them.
- Add new sub-tasks or TODOs discovered during development to `TASK.md` under a "Discovered During Work" section.

### 📚 Documentation & Explainability
- **Update `README.md`** when new features are added, dependencies change, or setup steps are modified.
- **Comment non-obvious code** and ensure everything is understandable to a mid-level developer.
- When writing complex logic, **add an inline comment** explaining the why, not just the what.

### 🧠 AI Behavior Rules
- **Never assume missing context. Ask questions if uncertain.**
- **Never hallucinate libraries or functions** – only use known, verified packages and libraries.
- **Always confirm file paths and module names** exist before referencing them in code.
- **Never delete or overwrite existing code** unless explicitly instructed to or if part of a task from `TASK.md`.