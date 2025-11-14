# Dockerfile

FROM ubuntu:20.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    libboost-all-dev \
    libeigen3-dev \
    git \
    && rm -rf /var/lib/apt/lists/*

# Clone GTSAM repository
RUN git clone https://github.com/borglab/gtsam.git /gtsam

# Build GTSAM
WORKDIR /gtsam
RUN mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install -j$(nproc)

# Set the working directory for the project
WORKDIR /gtsam-graph-slam

# Copy the project files into the container
COPY . .

# Build the project
RUN mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc)

# Command to run the application
CMD ["./build/main"]