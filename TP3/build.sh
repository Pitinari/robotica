#!/bin/bash

# Build script for stereo_vision package
# This script suppresses CMake development warnings that don't affect functionality

echo "Building stereo_vision package..."
colcon build --packages-select stereo_vision --cmake-args -Wno-dev

echo "Build complete!"
