# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Context
This is a ROS 2 humble package template designed for containerized robotics development. The template creates a general-purpose foundation for ROS 2 projects using colcon build system with Docker containerization.

## Architecture Overview
- **ROS Framework**: ROS 2 humble with colcon build system
- **Language**: C++ (primary)
- **Containerization**: Docker with ros:humble-ros-base base image
- **Networking**: Host networking mode for ROS communication
- **Visualization**: Foxglove bridge integration with WebSocket support
- **Package Pattern**: Standard ROS 2 workspace structure under `src/`

## Essential Commands
```bash
# Build ROS package with colcon
colcon build

# Source the workspace
source install/setup.bash

# Build Docker image
docker build -t ros-template:humble .

# Run with Docker Compose
docker-compose up

# Launch ROS nodes
ros2 launch <package_name> <launch_file>
```

## Development Patterns
- **Node Structure**: Implement publishers, subscribers, and timer-based operations
- **Message Handling**: Use appropriate queue sizes based on message frequency
- **Resource Management**: Use RAII with smart pointers (`std::shared_ptr`, `std::unique_ptr`)
- **Real-time Constraints**: Always consider robotics real-time requirements

## Docker Requirements
- Always use `--network host` for ROS communication between containers
- Rebuild Docker image when adding new dependencies
- Install packages in Dockerfile, not in running containers
- Use multi-stage builds for optimization

## File Management
- Keep C++ files under 500 lines; refactor into modules if needed
- Use `#pragma once` for header guards
- Follow task tracking in TASK.md
- Update PLANNING.md for architectural changes

## C++ Standards
- Use modern C++ features: auto, range-based loops, structured bindings
- Prefer smart pointers over raw pointers
- Implement proper constructor/destructor patterns for resource management

## Basic Rules
### 🔄 Project Awareness & Context
- **Always read `PLANNING.md`** at the start of a new conversation to understand the project's architecture, goals, style, and constraints.
- **Check `TASK.md`** before starting a new task. If the task isn’t listed, add it with a brief description and today's date.
- **Use consistent naming conventions, file structure, and architecture patterns** as described in `PLANNING.md`.

### 🧱 Code Structure & Modularity
- **Never create a file longer than 500 lines of code.** If a file approaches this limit, refactor by splitting it into modules or helper files.
- **Organize code into clearly separated modules**, grouped by feature or responsibility.

### ✅ Task Completion
- **Mark completed tasks in `TASK.md`** immediately after finishing them.
- Add new sub-tasks or TODOs discovered during development to `TASK.md` under a “Discovered During Work” section.

### 📚 Documentation & Explainability
- **Update `README.md`** when new features are added, dependencies change, or setup steps are modified.
- **Comment non-obvious code** and ensure everything is understandable to a mid-level developer.
- When writing complex logic, **add an inline comment** explaining the why, not just the what.

### 🧠 AI Behavior Rules
- **Never assume missing context. Ask questions if uncertain.**
- **Never hallucinate libraries or functions** – only use known, verified packages and libraries.
- **Always confirm file paths and module names** exist before referencing them in code.
- **Never delete or overwrite existing code** unless explicitly instructed to or if part of a task from `TASK.md`.