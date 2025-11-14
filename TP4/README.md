# GTSAM Graph-SLAM Project

This project implements 2D and 3D Graph-SLAM algorithms using the GTSAM library. The goal is to analyze datasets in G2O format and optimize trajectories based on the provided poses and edges.

## Project Structure

- **src/**: Contains the source code for the application.
  - **main.cpp**: Entry point of the application.
  - **graph_slam_2d.cpp**: Implementation of the 2D Graph-SLAM algorithm.
  - **graph_slam_3d.cpp**: Implementation of the 3D Graph-SLAM algorithm.
  - **g2o_reader.cpp**: Functions to read G2O files and extract poses and edges.
  - **utils/**: Contains utility functions for visualization and file operations.
    - **visualization.cpp**: Utility functions for visualizing SLAM results.
    - **file_utils.cpp**: Functions for handling file operations.

- **include/**: Header files for the project.
  - **graph_slam_2d.h**: Declarations for the 2D Graph-SLAM implementation.
  - **graph_slam_3d.h**: Declarations for the 3D Graph-SLAM implementation.
  - **g2o_reader.h**: Declarations for G2O reading functions.
  - **utils/**: Header files for utility functions.
    - **visualization.h**: Declarations for visualization utilities.
    - **file_utils.h**: Declarations for file utility functions.

- **data/**: Contains the G2O datasets used for testing.
  - **input_INTEL_g2o.g2o**: 2D G2O dataset for testing.
  - **parking-garage.g2o**: 3D G2O dataset for testing.

- **results/**: Directory for storing results.
  - **plots/**: Contains plots generated from SLAM results.
  - **logs/**: Stores log files generated during execution.

- **Dockerfile**: Instructions to build a Docker image for the project environment.

- **CMakeLists.txt**: Configuration file for CMake to build the project.

## Setup Instructions

1. **Clone the Repository**:
   Clone the repository from GitHub:
   ```
   git clone https://github.com/borglab/gtsam.git
   ```

2. **Install Dependencies**:
   Ensure you have the following dependencies installed:
   - Eigen
   - Boost (>= 1.58)
   - CMake (>= 3.0)
   - A modern C++ compiler (GCC >= 4.7.3)

   You can install Boost and CMake on Ubuntu using:
   ```
   sudo apt-get install libboost-all-dev cmake
   ```

3. **Build the Project**:
   Navigate to the project directory and create a build directory:
   ```
   cd gtsam-graph-slam
   mkdir build
   cd build
   cmake ..
   make -j10
   sudo make install -j10
   ```

## Usage

After building the project, you can run the application using the compiled binary. The application will read the G2O datasets and perform the Graph-SLAM optimization, generating visualizations and logs of the results.

## Algorithms Implemented

- **2D Graph-SLAM**: Optimizes trajectories in a 2D space using G2O data.
- **3D Graph-SLAM**: Optimizes trajectories in a 3D space using G2O data.

## Results

The results of the SLAM algorithms, including optimized trajectories and visualizations, will be stored in the `results/` directory.