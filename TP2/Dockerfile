# Usa la imagen oficial de Ubuntu 24.04 para ARM64
FROM ubuntu:24.04

# Evita prompts interactivos durante la instalación
ENV DEBIAN_FRONTEND=noninteractive

# 1. System setup: Set locale
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 2. Enable required repositories
RUN apt-get update && apt-get install -y \
    curl \
    gnupg \
    lsb-release \
    software-properties-common \
    dpkg
RUN add-apt-repository universe

# # Instala las fuentes de ROS 2 usando el método recomendado
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
RUN curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
RUN dpkg -i /tmp/ros2-apt-source.deb


# 3. Install development tools
RUN apt-get update && apt-get install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  ros-dev-tools

# 4. Get ROS 2 code
RUN mkdir -p /root/ros2_jazzy/src
WORKDIR /root/ros2_jazzy
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos src

# 5. Install dependencies using rosdep
RUN apt-get upgrade -y
RUN rosdep init && rosdep update
RUN rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Instala dependencias de Python de tu proyecto
RUN pip3 install numpy matplotlib transforms3d

# Configura el punto de entrada
CMD ["/bin/bash"]