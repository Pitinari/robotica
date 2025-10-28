# TP3: Stereo Vision - 3D Reconstruction and Pose Estimation

Computer Vision assignment for R-521 Robótica Móvil - Universidad Nacional de Rosario

## Overview

This project implements a complete stereo vision pipeline for 3D reconstruction and pose estimation using the EuRoC MAV dataset. The implementation uses ROS2 Jazzy, OpenCV 4, and C++17.

### Features Implemented

#### Mandatory Exercises (8/10 grade):

- ✅ **Exercise 1**: Stereo camera calibration
- ✅ **Exercise 2a**: Image rectification
- ✅ **Exercise 2b**: Feature extraction (FAST, ORB, GFTT detectors)
- ✅ **Exercise 2c**: Feature matching with distance filtering
- ✅ **Exercise 2d**: 3D triangulation from stereo correspondences
- ✅ **Exercise 2e**: RANSAC filtering and homography estimation
- ✅ **Exercise 2g**: Disparity map computation (SGBM/Block Matching)
- ✅ **Exercise 2h**: Dense 3D reconstruction from disparity
- ✅ **Exercise 2j**: Monocular pose estimation and trajectory tracking

#### Optional Exercises (10/10 grade):

- 🔄 **Exercise 2f**: Feature-based mapping with ground truth poses (framework ready)
- 🔄 **Exercise 2i**: Dense mapping with ground truth localization (framework ready)

## Project Structure

```
TP3/
├── CMakeLists.txt           # CMake build configuration
├── package.xml              # ROS2 package dependencies
├── Dockerfile               # Docker image for reproducibility
├── README.md                # This file
├── config/                  # Configuration files
│   ├── camera_info_left.yaml
│   ├── camera_info_right.yaml
│   └── extrinsics.yaml
├── include/                 # C++ header files
│   └── stereo_vision/
│       ├── utils.hpp
│       ├── stereo_rectifier.hpp
│       ├── feature_extractor.hpp
│       ├── feature_matcher.hpp
│       ├── triangulator.hpp
│       ├── disparity_computer.hpp
│       ├── dense_reconstructor.hpp
│       └── pose_estimator.hpp
├── src/                     # C++ implementation files
│   ├── utils.cpp
│   ├── stereo_rectifier.cpp
│   ├── feature_extractor.cpp
│   ├── feature_matcher.cpp
│   ├── triangulator.cpp
│   ├── disparity_computer.cpp
│   ├── dense_reconstructor.cpp
│   ├── pose_estimator.cpp
│   └── nodes/              # ROS2 node implementations
│       ├── stereo_pipeline_node.cpp
│       ├── stereo_rectifier_node.cpp
│       └── feature_extractor_node.cpp
├── launch/                  # ROS2 launch files
│   └── stereo_vision.launch.py
├── scripts/                 # Utility scripts
│   ├── download_euroc.sh
│   └── convert_to_rosbag.py
└── output/                  # Generated visualizations and results
```

## Dependencies

### System Requirements

- Ubuntu 24.04 LTS
- ROS2 Jazzy
- OpenCV 4.x with contrib modules
- PCL (Point Cloud Library)
- Eigen3

### ROS2 Packages

- cv_bridge
- image_transport
- sensor_msgs
- geometry_msgs
- nav_msgs
- pcl_ros
- pcl_conversions
- message_filters

### Python (for dataset tools)

- rosbags
- opencv-python
- numpy

## Installation

### Option 1: Docker (Recommended)

1. Build the Docker image:

```bash
cd TP3
docker build -t stereo_vision:jazzy .
```

2. Run the container:

```bash
docker run -it --rm \
  -v $(pwd)/data:/workspace/data \
  -v $(pwd)/output:/workspace/output \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  stereo_vision:jazzy
```

### Option 2: Native Installation

1. Install ROS2 Jazzy:

```bash
# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-vision-opencv \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-message-filters \
  ros-jazzy-camera-calibration
```

2. Install OpenCV and dependencies:

```bash
sudo apt install -y \
  libopencv-dev \
  libopencv-contrib-dev \
  libpcl-dev \
  libeigen3-dev \
  python3-pip

pip3 install rosbags opencv-python numpy
```

3. Build the workspace:

```bash
cd TP3
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Dataset Preparation

### Download EuRoC Dataset

Download a sequence from the EuRoC dataset:

```bash
./scripts/download_euroc.sh MH_01_easy
```

Available sequences:

- Machine Hall: MH_01_easy, MH_02_easy, MH_03_medium, MH_04_difficult, MH_05_difficult
- Vicon Room: V1_01_easy, V1_02_medium, V1_03_difficult, V2_01_easy, V2_02_medium, V2_03_difficult

### Convert to ROS2 Bag Format

```bash
python3 scripts/convert_to_rosbag.py ./data/MH_01_easy ./data/MH_01_easy_ros2
```

Note: Pre-converted ROS2 bags are available at: https://docs.openvins.com/gs-datasets.html

## Exercise 1: Camera Calibration

The EuRoC dataset provides calibration files in `mav0/camX/sensor.yaml`. To perform your own calibration:

### Using ROS2 camera_calibration:

1. Play the calibration sequence:

```bash
ros2 bag play ./data/calibration_sequence
```

2. Run camera calibration tool (in separate terminal):

```bash
ros2 run camera_calibration cameracalibrator \
  --size 6x6 \
  --square 0.088 \
  image:=/cam0/image_raw \
  camera:=/cam0
```

3. Move the checkerboard pattern until you get good coverage (X, Y, Size, Skew bars turn green)
4. Click "Calibrate" and wait for the computation
5. Click "Save" to store the calibration
6. Copy the generated files to `config/camera_info_left.yaml`
7. Repeat for the right camera (cam1)

### Extract Stereo Extrinsics:

The transformation between cameras is in the dataset's `sensor.yaml` files. Update `config/extrinsics.yaml` with:

- Rotation matrix (R)
- Translation vector (T)
- Baseline (distance between cameras)

## Exercise 2: Running the Stereo Vision Pipeline

### Full Pipeline (All Exercises)

Launch the complete stereo vision pipeline:

```bash
ros2 launch stereo_vision stereo_vision.launch.py bag_file:=./data/MH_01_easy_ros2
```

This will:

1. Play the ROS2 bag file
2. Rectify stereo images
3. Extract and match features
4. Apply RANSAC filtering
5. Triangulate 3D points
6. Compute disparity map
7. Perform dense 3D reconstruction
8. Estimate camera trajectory
9. Visualize everything in RViz2

### Parameters

You can customize the pipeline behavior:

```bash
ros2 launch stereo_vision stereo_vision.launch.py \
  bag_file:=./data/MH_01_easy_ros2 \
  detector_type:=FAST \
  descriptor_type:=BRISK \
  match_distance_threshold:=25.0 \
  use_ransac:=true
```

Available detectors: `FAST`, `ORB`, `GFTT`, `SIFT`
Available descriptors: `ORB`, `BRISK`, `SIFT`

### Individual Nodes

You can also run individual components:

**Rectification only:**

```bash
ros2 run stereo_vision stereo_rectifier_node
```

**Feature extraction:**

```bash
ros2 run stereo_vision feature_extractor_node
```

**Full pipeline node:**

```bash
ros2 run stereo_vision stereo_pipeline_node
```

## Visualization

### RViz2 Topics

The pipeline publishes the following topics:

- `/stereo/left/rect` - Rectified left image
- `/stereo/right/rect` - Rectified right image
- `/stereo/disparity` - Disparity map visualization
- `/stereo/sparse_cloud` - 3D point cloud from feature triangulation
- `/stereo/dense_cloud` - Dense 3D reconstruction
- `/stereo/trajectory` - Estimated camera trajectory (Path)

### Saved Images

When `save_images` is enabled, the first frame generates:

- `output/left_rect.png` - Rectified left image
- `output/right_rect.png` - Rectified right image
- `output/left_features.png` - Detected features on left image
- `output/right_features.png` - Detected features on right image
- `output/matches_all.png` - All feature matches
- `output/matches_ransac.png` - RANSAC-filtered matches
- `output/projected_points.png` - Homography projection visualization
- `output/disparity.png` - Disparity map

## Results and Analysis

### Exercise 2b: Feature Extraction

The system supports multiple detectors:

- **FAST**: Fast corner detection, suitable for real-time applications
- **ORB**: Oriented FAST and Rotated BRIEF, rotation invariant
- **GFTT**: Good Features To Track, Harris corner detector

Typical results:

- 500-1500 features detected per image
- Processing time: 10-30ms per image

### Exercise 2c: Feature Matching

Matching strategies implemented:

- Brute-force matching with distance threshold (< 30 pixels)
- Ratio test (Lowe's ratio test, threshold 0.75)
- Cross-checking for bidirectional consistency

Typical results:

- 200-600 matches before filtering
- 100-300 matches after distance filtering

### Exercise 2e: RANSAC Filtering

RANSAC with homography estimation removes outliers:

- Inlier rate: 70-90% typically
- Reprojection error threshold: 3.0 pixels

### Exercise 2g-h: Dense Reconstruction

Disparity computation using Semi-Global Block Matching (SGBM):

- Block size: 5x5
- Disparity range: 0-128 pixels
- Point cloud size: 50K-200K points after filtering

### Exercise 2j: Pose Estimation

Monocular visual odometry:

- Essential matrix estimation with RANSAC
- Pose recovery with cheirality check
- Scale factor from ground truth (required for absolute scale)

## Troubleshooting

### Build Errors

**OpenCV not found:**

```bash
sudo apt install libopencv-dev libopencv-contrib-dev
```

**PCL not found:**

```bash
sudo apt install libpcl-dev
```

### Runtime Errors

**No images in output:**

- Check that bag file is playing: `ros2 bag info <bag_file>`
- Verify topic names match: `ros2 topic list`
- Check remapping in launch file

**Calibration files not found:**

- Ensure config files exist in `config/` directory
- Check file paths in launch parameters
- Verify YAML syntax is correct

**Disparity map is empty:**

- Images must be rectified first
- Adjust SGBM parameters (block size, disparity range)
- Ensure sufficient texture in scene

## References

- EuRoC MAV Dataset: https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets
- OpenCV Stereo Documentation: https://docs.opencv.org/4.x/dd/d53/tutorial_py_depthmap.html
- ROS2 Camera Calibration: https://docs.nav2.org/tutorials/docs/camera_calibration.html
- OpenVINS Datasets: https://docs.openvins.com/gs-datasets.html

## Authors

- Maxi
- Universidad Nacional de Rosario
- R-521 Robótica Móvil

## License

MIT License - See LICENSE file for details
