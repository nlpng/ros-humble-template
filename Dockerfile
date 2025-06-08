# Multi-stage build for optimized image size
FROM ros:humble-ros-base AS builder

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    libspdlog-dev \
    nlohmann-json3-dev \
    pkg-config \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python structured logging and performance monitoring dependencies
RUN pip3 install structlog python-json-logger psutil

# Create workspace
WORKDIR /ros2_ws

# Copy source code
COPY src/ src/

# Initialize rosdep and install dependencies
RUN rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Production stage
FROM ros:humble-ros-base

# Install runtime dependencies and Foxglove bridge
RUN apt-get update && apt-get install -y \
    ros-humble-foxglove-bridge \
    ros-humble-launch \
    ros-humble-launch-ros \
    libspdlog1 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python structured logging and performance monitoring dependencies in runtime stage
RUN pip3 install structlog python-json-logger psutil

# Create workspace
WORKDIR /ros2_ws

# Copy built workspace from builder stage
COPY --from=builder /ros2_ws/install install/

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS 2 humble\n\
source /opt/ros/humble/setup.bash\n\
\n\
# Source workspace\n\
source /ros2_ws/install/setup.bash\n\
\n\
# Execute command\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

# Set environment variables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Expose Foxglove bridge port
EXPOSE 8765

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["/bin/bash"]