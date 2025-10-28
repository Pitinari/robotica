# Quick Start Guide

Get the stereo vision pipeline running in 5 minutes!

## Prerequisites

- Ubuntu 24.04
- 8GB RAM minimum
- 10GB free disk space

## Installation Steps

### 1. Install ROS2 Jazzy (if not already installed)

```bash
# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install
sudo apt update
sudo apt install -y ros-jazzy-desktop \
  ros-jazzy-cv-bridge \
  ros-jazzy-vision-opencv \
  ros-jazzy-pcl-ros \
  ros-jazzy-message-filters
```

### 2. Install Dependencies

```bash
sudo apt install -y \
  libopencv-dev \
  libopencv-contrib-dev \
  libpcl-dev \
  libeigen3-dev \
  python3-pip

pip3 install rosbags opencv-python numpy
```

### 3. Build the Package

```bash
cd /home/maxi/lcc/robotica/TP3
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 4. Download Sample Data

```bash
# Download a small dataset
./scripts/download_euroc.sh MH_01_easy

# Or download pre-converted ROS2 bag from OpenVINS
wget https://docs.openvins.com/downloads/euroc/V1_01_easy.zip
unzip V1_01_easy.zip -d ./data/
```

### 5. Run the Pipeline

```bash
# Terminal 1: Launch the pipeline (will auto-start RViz)
ros2 launch stereo_vision stereo_vision.launch.py bag_file:=./data/V1_01_easy

# Or use downloaded dataset (convert first if needed)
# python3 scripts/convert_to_rosbag.py ./data/MH_01_easy ./data/MH_01_easy_ros2
# ros2 launch stereo_vision stereo_vision.launch.py bag_file:=./data/MH_01_easy_ros2
```

### 6. View Results

- RViz will open automatically showing:

  - Rectified images
  - Feature matches
  - 3D point clouds
  - Camera trajectory

- Check the `output/` directory for saved visualizations:
  - `left_rect.png` - Rectified image
  - `left_features.png` - Detected features
  - `matches_all.png` - All matches
  - `matches_ransac.png` - Filtered matches
  - `disparity.png` - Disparity map

## Troubleshooting

### Build fails with "OpenCV not found"

```bash
sudo apt install libopencv-dev libopencv-contrib-dev
```

### Build fails with "PCL not found"

```bash
sudo apt install libpcl-dev
```

### No output when running

Check that topics are being published:

```bash
ros2 topic list
ros2 topic echo /stereo/left/rect --once
```

### Images look distorted

Update calibration files in `config/` directory with your camera parameters.

## Next Steps

1. **Customize parameters:** Edit `launch/stereo_vision.launch.py` to change:

   - Feature detector (FAST, ORB, GFTT)
   - Matching threshold
   - Disparity parameters

2. **Run on your own data:**

   - Calibrate your stereo camera (see CALIBRATION.md)
   - Record ROS2 bag: `ros2 bag record /cam0/image_raw /cam1/image_raw`
   - Run pipeline on your bag

3. **Analyze results:**
   - Compare trajectory with ground truth
   - Evaluate reconstruction quality
   - Generate report figures

## Docker Alternative

If you prefer Docker:

```bash
# Build
docker build -t stereo_vision:jazzy .

# Run
docker run -it --rm \
  -v $(pwd)/data:/workspace/data \
  -v $(pwd)/output:/workspace/output \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  stereo_vision:jazzy

# Inside container
ros2 launch stereo_vision stereo_vision.launch.py bag_file:=/workspace/data/V1_01_easy
```

## Support

For issues or questions:

1. Check README.md for detailed documentation
2. Check CALIBRATION.md for calibration help
3. Review the assignment PDF (tp.pdf)
