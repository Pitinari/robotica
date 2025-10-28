#!/bin/bash

# Script to download EuRoC MAV dataset using aria2c for faster downloads
# Usage: ./download_euroc.sh [dataset_name]
# Example: ./download_euroc.sh MH_01_easy

set -e

DATASET_NAME=${1:-"MH_01_easy"}
DATASET_URL="http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/${DATASET_NAME}/${DATASET_NAME}.zip"
DOWNLOAD_DIR="./data"

# Get number of CPU cores
NUM_CORES=$(nproc)

echo "Downloading EuRoC dataset: ${DATASET_NAME}"
echo "URL: ${DATASET_URL}"
echo "Using ${NUM_CORES} connections for download"

# Check if aria2c is installed
if ! command -v aria2c &> /dev/null; then
    echo "aria2c not found, installing..."
    sudo apt-get update && sudo apt-get install -y aria2
fi

# Create download directory
mkdir -p ${DOWNLOAD_DIR}

# Download dataset
cd ${DOWNLOAD_DIR}
if [ ! -f "${DATASET_NAME}.zip" ]; then
    echo "Downloading with aria2c..."
    aria2c -x ${NUM_CORES} -s ${NUM_CORES} --file-allocation=none --continue=true ${DATASET_URL}
else
    echo "Dataset already downloaded"
fi

# Extract
if [ ! -d "${DATASET_NAME}" ]; then
    echo "Extracting..."
    unzip ${DATASET_NAME}.zip
else
    echo "Dataset already extracted"
fi

echo "Done! Dataset location: ${DOWNLOAD_DIR}/${DATASET_NAME}"
echo ""
echo "Dataset structure:"
echo "  ${DOWNLOAD_DIR}/${DATASET_NAME}/mav0/         - Main data directory"
echo "  ${DOWNLOAD_DIR}/cam_checkerboard.bag          - Calibration bag (if downloaded separately)"
echo ""
echo "To convert to ROS2 bag format:"
echo "  # For main dataset bag (if exists)"
echo "  ./scripts/convert_to_rosbag.sh ${DOWNLOAD_DIR}/${DATASET_NAME}"
echo ""
echo "  # For calibration bag"
echo "  ./scripts/convert_to_rosbag.sh ./dataset/cam_checkerboard.bag ./data/calib_ros2"

