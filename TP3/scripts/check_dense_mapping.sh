#!/bin/bash

echo "=== Checking Dense Mapping Status ==="
echo ""

echo "1. Checking if dense_mapper_node is running..."
ros2 node list | grep dense_mapper && echo "✓ Node is running" || echo "✗ Node is NOT running"
echo ""

echo "2. Checking required topics..."
echo "   /stereo/disparity_raw:"
timeout 2s ros2 topic hz /stereo/disparity_raw 2>&1 | head -3 || echo "   ✗ Not publishing or no data"
echo ""

echo "   /stereo/left/rect:"
timeout 2s ros2 topic hz /stereo/left/rect 2>&1 | head -3 || echo "   ✗ Not publishing or no data"
echo ""

echo "   /imu0:"
timeout 2s ros2 topic hz /imu0 2>&1 | head -3 || echo "   ✗ Not publishing or no data"
echo ""

echo "   /leica/position:"
timeout 2s ros2 topic hz /leica/position 2>&1 | head -3 || echo "   ✗ Not publishing or no data"
echo ""

echo "3. Checking dense_mapping output topics..."
echo "   /dense_mapping/point_cloud:"
timeout 2s ros2 topic hz /dense_mapping/point_cloud 2>&1 | head -3 || echo "   ✗ Not publishing"
echo ""

echo "   /dense_mapping/camera_pose:"
timeout 2s ros2 topic hz /dense_mapping/camera_pose 2>&1 | head -3 || echo "   ✗ Not publishing"
echo ""

echo "4. Checking topic types..."
echo "   /stereo/disparity_raw type:"
ros2 topic type /stereo/disparity_raw 2>&1 || echo "   ✗ Topic doesn't exist"
echo ""

echo "=== Done ==="

