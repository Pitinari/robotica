#!/bin/bash

# Convert EuRoC MAV dataset to ROS2 bag format using rosbags-convert
# This script handles the conversion from ROS1 bag format to ROS2

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored messages
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    cat << EOF
EuRoC Dataset to ROS2 Bag Conversion Script

Usage: $0 <dataset_path> [output_path]

ARGUMENTS:
    dataset_path    Path to EuRoC dataset directory (containing mav0 folder)
    output_path     Output directory for ROS2 bag (default: <dataset_path>_ros2)

EXAMPLES:
    # Convert with default output name
    $0 ./data/MH_01_easy

    # Convert with custom output name
    $0 ./data/MH_01_easy ./data/MH_01_easy_rosbag2

    # Convert calibration sequence
    $0 ./data/calib ./data/calib_ros2

REQUIREMENTS:
    - rosbags-convert must be installed: pip install rosbags
    - Dataset must contain ROS1 bag file (.bag)

NOTES:
    - The script automatically searches for .bag files in the dataset
    - Conversion may take several minutes depending on dataset size
    - Output will be in ROS2 bag format (SQLite3 database)

ALTERNATIVE:
    You can also download pre-converted ROS2 bags from:
    https://docs.openvins.com/gs-datasets.html

EOF
}

# Check arguments
if [ $# -lt 1 ]; then
    print_error "Missing required argument: dataset_path"
    echo ""
    show_usage
    exit 1
fi

if [ "$1" == "-h" ] || [ "$1" == "--help" ]; then
    show_usage
    exit 0
fi

DATASET_PATH="$1"
OUTPUT_PATH="${2:-${DATASET_PATH}_ros2}"

# Validate dataset path (can be a directory or a .bag file)
if [ ! -d "$DATASET_PATH" ] && [ ! -f "$DATASET_PATH" ]; then
    print_error "Dataset path not found: $DATASET_PATH"
    exit 1
fi

print_info "========================================="
print_info "EuRoC to ROS2 Bag Conversion"
print_info "========================================="
print_info "Dataset path: $DATASET_PATH"
print_info "Output path:  $OUTPUT_PATH"
print_info "========================================="
echo ""

# Check if rosbags-convert is installed
print_info "Checking for rosbags-convert..."
if ! command -v rosbags-convert &> /dev/null; then
    print_error "rosbags-convert not found"
    print_info "Install with: pip install rosbags"
    print_info "Or: python3 -m pip install rosbags"
    exit 1
fi

print_success "rosbags-convert found"

# Look for ROS1 bag file
print_info "Searching for ROS1 bag file..."

ROS1_BAG=""

# Check if the path itself is a .bag file
if [ -f "$DATASET_PATH" ] && [[ "$DATASET_PATH" == *.bag ]]; then
    ROS1_BAG="$DATASET_PATH"
    print_info "Using provided bag file directly"
else
    # Search in root directory
    for bag in "$DATASET_PATH"/*.bag; do
        if [ -f "$bag" ]; then
            ROS1_BAG="$bag"
            break
        fi
    done

    # If not found, search in mav0 directory
    if [ -z "$ROS1_BAG" ]; then
        if [ -d "$DATASET_PATH/mav0" ]; then
            for bag in "$DATASET_PATH/mav0"/*.bag; do
                if [ -f "$bag" ]; then
                    ROS1_BAG="$bag"
                    break
                fi
            done
        fi
    fi

    # If still not found, check for nested bag files
    if [ -z "$ROS1_BAG" ]; then
        ROS1_BAG=$(find "$DATASET_PATH" -name "*.bag" -type f | head -1)
    fi
fi

if [ -z "$ROS1_BAG" ] || [ ! -f "$ROS1_BAG" ]; then
    print_error "No ROS1 bag file found in dataset"
    echo ""
    print_info "Expected locations:"
    print_info "  - $DATASET_PATH/*.bag"
    print_info "  - $DATASET_PATH/mav0/*.bag"
    echo ""
    print_warning "Note: EuRoC dataset format may vary"
    print_info "You may need to:"
    print_info "  1. Download the dataset in bag format from the official site"
    print_info "  2. Or download pre-converted ROS2 bags from:"
    print_info "     https://docs.openvins.com/gs-datasets.html"
    exit 1
fi

print_success "Found ROS1 bag: $ROS1_BAG"

# Get bag file size
BAG_SIZE=$(du -h "$ROS1_BAG" | cut -f1)
print_info "Bag size: $BAG_SIZE"

# Check if output already exists
if [ -d "$OUTPUT_PATH" ]; then
    print_warning "Output directory already exists: $OUTPUT_PATH"
    read -p "Overwrite? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        print_info "Conversion cancelled"
        exit 0
    fi
    print_info "Removing existing output directory..."
    rm -rf "$OUTPUT_PATH"
fi

# Convert using rosbags-convert
echo ""
print_info "Starting conversion (this may take several minutes)..."
print_info "Command: rosbags-convert --src \"$ROS1_BAG\" --dst \"$OUTPUT_PATH\""
echo ""

# Run conversion with progress
if rosbags-convert --src "$ROS1_BAG" --dst "$OUTPUT_PATH"; then
    echo ""
    print_success "Conversion completed successfully!"
    
    # Show output info
    if [ -d "$OUTPUT_PATH" ]; then
        OUTPUT_SIZE=$(du -sh "$OUTPUT_PATH" | cut -f1)
        print_info "Output size: $OUTPUT_SIZE"
        
        # Count files
        FILE_COUNT=$(find "$OUTPUT_PATH" -type f | wc -l)
        print_info "Files created: $FILE_COUNT"
        
        # List main files
        print_info "Output contents:"
        ls -lh "$OUTPUT_PATH" | tail -n +2 | awk '{print "  - " $9 " (" $5 ")"}'
    fi
    
    echo ""
    print_info "========================================="
    print_info "Verification"
    print_info "========================================="
    
    # Check if ROS2 is available to verify
    if command -v ros2 &> /dev/null; then
        print_info "Checking bag info..."
        echo ""
        ros2 bag info "$OUTPUT_PATH" || print_warning "Could not read bag info (ROS2 may not be sourced)"
    else
        print_warning "ROS2 not found - skipping verification"
        print_info "To verify later, run: ros2 bag info $OUTPUT_PATH"
    fi
    
    echo ""
    print_info "========================================="
    print_info "Next Steps"
    print_info "========================================="
    print_info "To play the converted bag:"
    print_info "  ros2 bag play $OUTPUT_PATH"
    echo ""
    print_info "To play with clock and slower rate:"
    print_info "  ros2 bag play $OUTPUT_PATH --clock --rate 0.5"
    echo ""
    print_info "To use with stereo vision pipeline:"
    print_info "  ros2 launch stereo_vision stereo_vision.launch.py bag_file:=$OUTPUT_PATH"
    echo ""
    
else
    echo ""
    print_error "Conversion failed!"
    print_info "Please check the error messages above"
    exit 1
fi

print_success "Done!"

