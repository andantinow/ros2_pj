#!/bin/bash
# Clean build script for ROS 2 workspace

echo "Cleaning build directories..."

# Remove problematic symlink directory if it exists
if [ -d "build/vehicle_model_msgs/ament_cmake_python/vehicle_model_msgs/vehicle_model_msgs" ]; then
    echo "Removing problematic directory..."
    rm -rf build/vehicle_model_msgs/ament_cmake_python/vehicle_model_msgs/vehicle_model_msgs
fi

# Option 1: Clean only vehicle_model_msgs
echo "Cleaning vehicle_model_msgs build and install directories..."
rm -rf build/vehicle_model_msgs install/vehicle_model_msgs

echo "Clean complete. You can now run: colcon build --symlink-install"

