#!/bin/bash

# Docker Testing Script for ROS 2 Template
set -e

echo "🐳 Docker-based ROS 2 Template Testing"
echo "======================================"

# Create a test Dockerfile that enables testing
cat > Dockerfile.test << 'EOF'
# Multi-stage build for testing
FROM ros:humble-ros-base AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy source code
COPY src/ src/

# Initialize rosdep and install dependencies
RUN rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace with testing
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON

# Run tests
RUN . /opt/ros/humble/setup.sh && \
    . install/setup.bash && \
    colcon test --event-handlers console_direct+ && \
    colcon test-result --verbose

# Production stage
FROM ros:humble-ros-base

# Install runtime dependencies and Foxglove bridge
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-launch \
    ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Copy built workspace from builder stage
COPY --from=builder /ros2_ws/install install/

# Create entrypoint script
RUN echo '#!/bin/bash\nset -e\n\nsource /opt/ros/humble/setup.bash\n\nsource /ros2_ws/install/setup.bash\n\nexec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set environment variables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Expose Foxglove bridge port
EXPOSE 8765

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["ros2", "launch", "ros_template_node", "template.launch.py"]
EOF

echo "🔨 Building Docker image with tests..."
docker build -f Dockerfile.test -t ros-template:test .

if [ $? -eq 0 ]; then
    echo "✅ Docker build with tests successful!"
else
    echo "❌ Docker build with tests failed!"
    exit 1
fi

echo ""
echo "🧪 Testing Docker Compose profiles..."

# Test default profile
echo "Testing default profile..."
timeout 30s docker compose up --build || echo "Default profile test completed"

# Test python profile
echo "Testing python profile..."
timeout 30s docker compose --profile python up --build || echo "Python profile test completed"

# Clean up
docker compose down 2>/dev/null || true

# Clean up test dockerfile
rm -f Dockerfile.test

echo ""
echo "🎉 Docker testing complete!"
EOF

# Make the script executable
chmod +x test_docker.sh

echo "🔨 Building Docker test image..."
./test_docker.sh