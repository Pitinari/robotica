cmake_minimum_required(VERSION 3.10)
project(GTSAMGraphSLAM)

set(CMAKE_CXX_STANDARD 14)

# Find GTSAM
find_package(GTSAM REQUIRED)

# Include directories
include_directories(include ${GTSAM_INCLUDE_DIRS})

# Source files
set(SOURCES
    src/main.cpp
    src/graph_slam_2d.cpp
    src/graph_slam_3d.cpp
    src/g2o_reader.cpp
    src/utils/visualization.cpp
    src/utils/file_utils.cpp
)

# Create the executable
add_executable(GTSAMGraphSLAM ${SOURCES})

# Link GTSAM libraries
target_link_libraries(GTSAMGraphSLAM ${GTSAM_LIBRARIES})