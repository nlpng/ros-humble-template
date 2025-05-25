#!/bin/bash

# ROS 2 Template Testing Script
set -e

echo "🔬 Running ROS 2 Template Tests"
echo "=================================="

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ Sourced ROS 2 Humble"
else
    echo "❌ ROS 2 Humble not found. Please install ROS 2 Humble."
    exit 1
fi

# Build packages
echo ""
echo "🔨 Building packages..."
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-args -DBUILD_TESTING=ON

if [ $? -eq 0 ]; then
    echo "✅ Build successful"
else
    echo "❌ Build failed"
    exit 1
fi

# Source workspace
source install/setup.bash

# Run tests
echo ""
echo "🧪 Running tests..."

# C++ tests
echo ""
echo "📋 C++ Package Tests:"
echo "---------------------"
colcon test --packages-select ros_template_node --event-handlers console_direct+

# Python tests
echo ""
echo "🐍 Python Package Tests:"
echo "------------------------"
colcon test --packages-select py_template_node --event-handlers console_direct+

# Display test results
echo ""
echo "📊 Test Results:"
echo "=================="
colcon test-result --verbose

# Check for test failures
if colcon test-result; then
    echo ""
    echo "🎉 All tests passed!"
else
    echo ""
    echo "❌ Some tests failed. Check the output above."
    exit 1
fi

echo ""
echo "✨ Testing complete!"