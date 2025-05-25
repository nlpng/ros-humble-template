# Project Planning: ROS 2 humble package template with Docker container and catkin package build

## Project Overview
We're building a simple ROS (Robotic Operating System) 2 humble package template. The ROS package template should be general enough to be used for other projects as well with simple modifications. The ROS package will be built inside docker container, the function of the package is to launch a simple ROS 2 node that subscribe to topics and publish topics. This ROS 2 package template using colcon build system. Always consider ROS-specific patterns, message passing, and real-time constraints when writing code.

## Current Implementation Status
The template includes:
- Complete ROS 2 node with publisher/subscriber patterns
- Docker containerization with host networking
- Foxglove bridge integration for visualization
- Configurable launch parameters
- Timer-based periodic operations

## Architecture

### Core Components:
1. **Docker Environment**
   - Base image: ros:humble-ros-base
   - Network: Host networking (network_mode: host) for ROS communication
   - Single container setup with docker-compose
   - Image tag: ros-template:humble

2. **ROS Package Pattern**
   - colcon build system with C++
   - Package name: ros_template_node
   - Node implementation with publishers, subscribers, and timers
   - Launch file with configurable parameters
   - Foxglove bridge integration for visualization
   - ROS 2 humble workspace structure:
    ```
    workspace_folder/
    src/
      cpp_package_1/
          CMakeLists.txt
          include/cpp_package_1/
          package.xml
          src/

      py_package_1/
          package.xml
          resource/py_package_1
          setup.cfg
          setup.py
          py_package_1/
    ```

### Technology Stack:
- **Language**: C++
- **ROS Framework**: ROS 2 humble
- **ROS build system**: colcon
- **Docker base image**: ros:humble-ros-base
- **Visualization**: Foxglove bridge with WebSocket support
- **Container orchestration**: Docker Compose with host networking

## Development Process

The development follows a task-based approach. Implementation the followings:

1. **ROS Package Setup**: Complete colcon workspace with ros_template_node package
2. **Dockerfile Configuration**: Multi-stage build with ros:humble-ros-base base
3. **Docker Compose**: Single service configuration with host networking
4. **Node Implementation**: **Think** of examples for Publishers, subscribers, timers, and message handling
5. **Launch Configuration**: Create launch file to start the ROS 2 node with Foxglove integration
6. **Visualization**: Foxglove bridge for web-based monitoring

## Design Principles

1. **Modularity**: Keep components decoupled for easier maintenance
2. **Simplicity**: Focus on making the system easy to understand and modify
3. **Optimization**: Optimize the Docker image size to be small and compact

## C++ Code Style & Standards

- Use features: auto, range-based for loops, smart pointers, structured bindings
- Smart pointers: Prefer `std::shared_ptr` and `std::unique_ptr` over raw pointers
- Header guards: Use `#pragma once` instead of traditional guards
- RAII: Use constructors/destructors for resource management

## Notes

When implementing this project, make sure to:
- Mark tasks complete in the **TASK.md** file as you finish them
- Always consider real-time constraints in robotics applications
- Use appropriate queue sizes for publishers/subscribers based on message frequency
- Consider using docker-compose for complex multi-node setups
- REMEMBER to rebuild Docker image when adding new dependencies
- ALWAYS use --network host for ROS communication between containers
- ALWAYS commit the changes with compact and concise messages

When implementing this project, make sure **NOT** to:
 - DON'T install packages inside running containers (use Dockerfile)
