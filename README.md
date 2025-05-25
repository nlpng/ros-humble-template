# ROS 2 Humble Package Template

A reusable ROS 2 humble package template with Docker containerization, demonstrating publisher/subscriber patterns, timer functionality, and Foxglove bridge integration for web-based visualization.

## Features

- **ROS 2 Node**: Complete implementation with publishers, subscribers, and timers
- **Docker Integration**: Containerized with `ros:humble-ros-base` base image
- **Foxglove Bridge**: Web-based visualization support via WebSocket
- **Configurable Launch**: Parameters for publish rate, topic prefix, and Foxglove settings
- **Multi-stage Build**: Optimized Docker image for production deployment
- **Host Networking**: Proper ROS 2 communication between containers

## Quick Start

### Using Docker Compose (Recommended)

1. **Build and run the template**:
   ```bash
   docker-compose up --build
   ```

2. **Access Foxglove Studio**:
   - Open [Foxglove Studio](https://studio.foxglove.dev) in your browser
   - Connect to WebSocket: `ws://localhost:8765`
   - Subscribe to topics: `/template/status`, `/template/counter`, `/template/temperature`

3. **Test command publishing**:
   ```bash
   # In another terminal
   docker exec -it ros_template_node bash
   ros2 topic pub /template/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
   ```

### Using Native ROS 2

1. **Build the workspace**:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Launch the node**:
   ```bash
   ros2 launch ros_template_node template.launch.py
   ```

## Project Structure

```
ros-humble-template/
├── src/ros_template_node/
│   ├── package.xml              # Package dependencies and metadata
│   ├── CMakeLists.txt          # Build configuration for colcon
│   ├── src/template_node.cpp   # Main node implementation
│   └── launch/template.launch.py  # Launch file with parameters
├── Dockerfile                  # Multi-stage Docker build
├── docker-compose.yml         # Container orchestration with host networking
├── CLAUDE.md                  # AI assistant guidance
├── PLANNING.md               # Architecture and design documentation
└── README.md                 # This file
```

## Node Functionality

The `template_node` demonstrates common ROS 2 patterns:

### Publishers
- `/template/status` (`std_msgs/String`) - Node status messages
- `/template/counter` (`std_msgs/Int32`) - Incrementing counter
- `/template/temperature` (`sensor_msgs/Temperature`) - Simulated sensor data

### Subscribers
- `/template/cmd_vel` (`geometry_msgs/Twist`) - Command velocity input

### Parameters
- `publish_rate` (double, default: 1.0) - Publishing frequency in Hz
- `topic_prefix` (string, default: "template") - Prefix for all topics

## Configuration

### Launch Parameters

```bash
ros2 launch ros_template_node template.launch.py \
  publish_rate:=5.0 \
  topic_prefix:=my_robot \
  enable_foxglove:=true \
  foxglove_port:=8765
```

### Docker Compose Profiles

- **Default**: Single node with Foxglove bridge
- **Multi-node**: Run multiple instances with different configurations
  ```bash
  docker-compose --profile multi-node up
  ```

## Development

### Building Locally

1. **Install dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

2. **Build with colcon**:
   ```bash
   colcon build --symlink-install
   ```

3. **Source and run**:
   ```bash
   source install/setup.bash
   ros2 run ros_template_node template_node
   ```

### Customization

This template can be easily adapted for other projects:

1. **Rename the package**: Update `package.xml`, `CMakeLists.txt`, and folder names
2. **Modify node functionality**: Edit `src/template_node.cpp` with your specific logic
3. **Update dependencies**: Add required packages to `package.xml` and `CMakeLists.txt`
4. **Configure Docker**: Modify `Dockerfile` for additional dependencies

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
ros2 topic echo /template/status

# Check node information
ros2 node info /template_node

# View parameter values
ros2 param list /template_node
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

## License

Apache-2.0