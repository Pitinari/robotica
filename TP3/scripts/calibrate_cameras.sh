#!/bin/bash

# Simple Camera Calibration Script for EuRoC Dataset
# Starts bag playback and launches stereo camera calibration

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    cat << EOF
Simple Stereo Camera Calibration Script

Usage: $0 <bag_file>

EXAMPLE:
    $0 ./data/calib_ros2

This script will:
    1. Start playing the ROS2 bag file in loop mode
    2. Launch the stereo camera calibrator GUI
    3. You perform the calibration in the GUI
    4. Save the results when done

CALIBRATION PATTERN:
    - Checkerboard size: 7x6 (internal corners)
    - Square size: 0.06 meters

EOF
}

# Check arguments
if [ $# -lt 1 ]; then
    print_error "Missing required argument: bag_file"
    echo ""
    show_usage
    exit 1
fi

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    show_usage
    exit 0
fi

BAG_FILE="$1"

# Validate bag file
if [ ! -d "$BAG_FILE" ]; then
    print_error "Bag file not found: $BAG_FILE"
    exit 1
fi

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    print_info "ROS2 not sourced. Sourcing ROS2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
fi

# Check if camera_calibration is installed
if ! ros2 pkg list | grep -q camera_calibration; then
    print_error "camera_calibration package not found"
    print_info "Install with: sudo apt install ros-${ROS_DISTRO}-camera-calibration"
    exit 1
fi

print_info "========================================="
print_info "Stereo Camera Calibration"
print_info "========================================="
print_info "Bag file: $BAG_FILE"
print_info "Checkerboard: 7x6 (internal corners)"
print_info "Square size: 0.06 meters"
print_info "========================================="
echo ""

# Function to play bag in background
play_bag() {
    print_info "Starting bag playback in loop mode..."
    ros2 bag play "$BAG_FILE" --loop --clock --rate 0.5 > /dev/null 2>&1 &
    BAG_PID=$!
    print_success "Bag playback started (PID: $BAG_PID)"
    echo "$BAG_PID" > /tmp/calibration_bag.pid
}

# Function to stop bag playback
stop_bag() {
    if [ -f /tmp/calibration_bag.pid ]; then
        BAG_PID=$(cat /tmp/calibration_bag.pid)
        if ps -p $BAG_PID > /dev/null 2>&1; then
            print_info "Stopping bag playback (PID: $BAG_PID)..."
            kill $BAG_PID 2>/dev/null || true
            rm /tmp/calibration_bag.pid
        fi
    fi
}

# Setup cleanup on exit
trap stop_bag EXIT INT TERM

# Start bag playback
play_bag

# Wait a bit for bag to start
sleep 3

# Check if topics are being published
print_info "Checking if images are being published..."
if timeout 5s ros2 topic list | grep -q "/cam0/image_raw"; then
    print_success "Topics are available"
else
    print_error "Topics not found. Make sure the bag is playing correctly"
fi

echo ""
print_info "========================================="
print_info "Starting Stereo Camera Calibration"
print_info "========================================="
echo ""
print_info "Instructions:"
print_info "1. Move the checkerboard to cover different areas"
print_info "2. Cover corners, edges, and center of both images"
print_info "3. Vary the distance and angle of the pattern"
print_info "4. Wait for all bars (X, Y, Size, Skew) to turn green"
print_info "5. Click 'Calibrate' button and wait"
print_info "6. Click 'Save' or 'Commit' to save calibration"
print_info "7. Close the window when done"
echo ""

# Run camera calibration
print_info "Launching stereo camera calibration GUI..."
echo ""

ros2 run camera_calibration cameracalibrator \
    --size 7x6 \
    --square 0.06 \
    --no-service-check \
    --approximate 0.1 \
    right:=/cam1/image_raw \
    left:=/cam0/image_raw \
    left_camera:=/cam0 \
    right_camera:=/cam1

CALIB_RESULT=$?

# Stop bag playback
stop_bag

echo ""
if [ $CALIB_RESULT -eq 0 ]; then
    print_success "Calibration completed"
    echo ""
    print_info "Calibration files saved to: /tmp/calibrationdata.tar.gz"
    print_info "Extract and copy the YAML files to your config directory"
else
    print_error "Calibration failed or was cancelled"
    exit 1
fi

echo ""
print_success "Done!"
