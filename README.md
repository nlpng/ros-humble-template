# ROS 2 Humble Package Template

A reusable ROS 2 humble package template with Docker containerization, demonstrating publisher/subscriber patterns, timer functionality, and Foxglove bridge integration for web-based visualization.

## Features

- **Dual Language Support**: Complete C++ and Python ROS 2 implementations
- **ROS 2 Nodes**: Publishers, subscribers, and timers in both C++ and Python
- **Docker Integration**: Containerized with `ros:humble-ros-base` base image
- **Foxglove Bridge**: Web-based visualization support via WebSocket
- **Configurable Launch**: Parameters for publish rate, topic prefix, and Foxglove settings
- **Multi-stage Build**: Optimized Docker image for production deployment
- **Host Networking**: Proper ROS 2 communication between containers
- **Multiple Deployment Options**: Run C++, Python, or both nodes simultaneously

## Quick Start

### Using Docker Compose (Recommended)

1. **Run C++ template (default)**:
   ```bash
   docker compose up --build
   ```

2. **Run Python template**:
   ```bash
   docker compose --profile python up --build
   ```

3. **Run both C++ and Python templates**:
   ```bash
   docker compose --profile dual up --build
   ```

4. **Access Foxglove Studio**:
   - Open [Foxglove Studio](https://studio.foxglove.dev) in your browser
   - Connect to WebSocket: `ws://localhost:8765`
   - Subscribe to topics: `/template/status`, `/py_template/status`, etc.

5. **Test command publishing**:
   ```bash
   # C++ node
   docker exec -it ros_template_node bash
   ros2 topic pub /template/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
   
   # Python node
   ros2 topic pub /py_template/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
   ```

### Using Native ROS 2

1. **Build the workspace**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Launch C++ node**:
   ```bash
   ros2 launch ros_template_node template.launch.py
   ```

3. **Launch Python node**:
   ```bash
   ros2 launch py_template_node py_template.launch.py
   ```

4. **Launch both nodes**:
   ```bash
   # Terminal 1
   ros2 launch ros_template_node template.launch.py topic_prefix:=cpp_template
   
   # Terminal 2
   ros2 launch py_template_node py_template.launch.py topic_prefix:=py_template
   ```

## Project Structure

```
ros-humble-template/
├── src/
│   ├── ros_template_node/          # C++ Package
│   │   ├── package.xml             # Package dependencies and metadata
│   │   ├── CMakeLists.txt         # Build configuration for colcon
│   │   ├── src/template_node.cpp  # Main C++ node implementation
│   │   └── launch/template.launch.py # C++ launch file
│   └── py_template_node/           # Python Package
│       ├── package.xml             # Python package dependencies
│       ├── setup.py               # Python package setup
│       ├── setup.cfg              # Python package configuration
│       ├── py_template_node/      # Python source directory
│       │   ├── __init__.py        # Python package init
│       │   └── py_template_node.py # Main Python node implementation
│       ├── launch/py_template.launch.py # Python launch file
│       └── resource/py_template_node # Resource marker file
├── Dockerfile                     # Multi-stage Docker build for both packages
├── docker-compose.yml            # Container orchestration with multiple profiles
├── CLAUDE.md                     # AI assistant guidance
├── PLANNING.md                   # Architecture and design documentation
└── README.md                     # This file
```

## Node Functionality

Both C++ (`ros_template_node`) and Python (`py_template_node`) demonstrate common ROS 2 patterns:

### Publishers (per node)
- `/{prefix}/status` (`std_msgs/String`) - Node status messages
- `/{prefix}/counter` (`std_msgs/Int32`) - Incrementing counter
- `/{prefix}/temperature` (`sensor_msgs/Temperature`) - Simulated sensor data

### Subscribers (per node)
- `/{prefix}/cmd_vel` (`geometry_msgs/Twist`) - Command velocity input

### Parameters (per node)
- `publish_rate` (double) - Publishing frequency in Hz
  - C++ default: 2.0 Hz
  - Python default: 1.0 Hz
- `topic_prefix` (string) - Prefix for all topics
  - C++ default: "template"
  - Python default: "py_template"

## Configuration

### Launch Parameters

**C++ Node:**
```bash
ros2 launch ros_template_node template.launch.py \
  publish_rate:=5.0 \
  topic_prefix:=my_robot \
  enable_foxglove:=true \
  foxglove_port:=8765
```

**Python Node:**
```bash
ros2 launch py_template_node py_template.launch.py \
  publish_rate:=3.0 \
  topic_prefix:=my_py_robot \
  enable_foxglove:=false
```

### Docker Compose Profiles

- **Default**: C++ node with Foxglove bridge
  ```bash
  docker compose up
  ```
- **Python**: Python node only
  ```bash
  docker compose --profile python up
  ```
- **Dual**: Both C++ and Python nodes simultaneously
  ```bash
  docker compose --profile dual up
  ```
- **Multi-node**: Fast C++ instance with different parameters
  ```bash
  docker compose --profile multi-node up
  ```

## Development

### Building and Testing Locally

**Using Docker (Recommended)**:
```bash
# Build and test everything
./test_docker.sh

# Or manually build for testing
docker build -t ros-template:humble .
```

**Native ROS 2 (if available)**:
```bash
# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build with testing
colcon build --cmake-args -DBUILD_TESTING=ON

# Run tests
source install/setup.bash
colcon test
colcon test-result --verbose

# Run nodes
ros2 run ros_template_node template_node
ros2 run py_template_node py_template_node
```

### Customization

This template can be easily adapted for other projects:

**For C++ packages:**
1. **Rename the package**: Update `package.xml`, `CMakeLists.txt`, and folder names
2. **Modify node functionality**: Edit `src/template_node.cpp` with your specific logic
3. **Update dependencies**: Add required packages to `package.xml` and `CMakeLists.txt`

**For Python packages:**
1. **Rename the package**: Update `package.xml`, `setup.py`, and folder names
2. **Modify node functionality**: Edit `py_template_node/py_template_node.py` with your specific logic
3. **Update dependencies**: Add required packages to `package.xml`

**Common:**
4. **Configure Docker**: Modify `Dockerfile` for additional dependencies
5. **Choose language**: Use either C++, Python, or both based on your needs

## Docker Commands

```bash
# Build image manually
docker build -t ros-template:humble .

# Run single container
docker run --network host --rm -it ros-template:humble

# View logs
docker-compose logs -f

# Execute commands in running container
docker exec -it ros_template_node bash
```

## ROS 2 Commands

```bash
# List active topics
ros2 topic list

# Monitor topic data
ros2 topic echo /template/status      # C++ node
ros2 topic echo /py_template/status   # Python node

# Check node information
ros2 node info /template_node         # C++ node
ros2 node info /py_template_node      # Python node

# View parameter values
ros2 param list /template_node        # C++ node
ros2 param list /py_template_node     # Python node
```

## Troubleshooting

### Common Issues

1. **Foxglove connection fails**: Ensure port 8765 is not blocked by firewall
2. **Topics not visible**: Check ROS_DOMAIN_ID matches between containers/hosts
3. **Build errors**: Verify all dependencies are installed with `rosdep`

### Debugging

```bash
# Check container logs
docker-compose logs ros_template

# Access container shell
docker exec -it ros_template_node bash

# Verify ROS 2 installation
ros2 doctor
```

## Requirements

- Docker and Docker Compose
- ROS 2 Humble (for native builds)
- Foxglove Studio (for visualization)

## Project Evolution

This template has evolved through comprehensive development phases:

1. **Initial C++ Implementation**: Complete ROS 2 C++ package with Docker integration
2. **Bug Fixes & Optimization**: Resolved build issues and improved Docker configuration  
3. **Python Enhancement**: Added equivalent Python implementation for dual language support
4. **Comprehensive Testing**: Verified all deployment scenarios and updated documentation

### What Makes This Template Special

- **Language Flexibility**: Choose C++, Python, or both based on your project needs
- **Production Ready**: Fully containerized with optimized multi-stage builds
- **Educational**: Perfect for learning ROS 2 patterns in both languages
- **Well Documented**: Comprehensive examples and clear customization guidance
- **Deployment Options**: Multiple Docker Compose profiles for different use cases

## License

Apache-2.0