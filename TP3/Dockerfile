FROM ubuntu:24.04

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    git \
    build-essential \
    cmake \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Add ROS2 Jazzy repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-vision-opencv \
    ros-jazzy-pcl-ros \
    ros-jazzy-pcl-conversions \
    ros-jazzy-message-filters \
    ros-jazzy-camera-calibration \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install OpenCV with contrib modules
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    libopencv-contrib-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install PCL and Eigen
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies for dataset conversion
RUN pip3 install --no-cache-dir \
    rosbags \
    opencv-python \
    numpy

# Create workspace
RUN mkdir -p /workspace/src
WORKDIR /workspace

# Copy the stereo_vision package
COPY . /workspace/src/stereo_vision/

# Source ROS2 and build workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /workspace && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Setup entrypoint
RUN echo '#!/bin/bash\n\
    set -e\n\
    source /opt/ros/jazzy/setup.bash\n\
    source /workspace/install/setup.bash\n\
    exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

