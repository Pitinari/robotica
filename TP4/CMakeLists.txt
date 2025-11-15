cmake_minimum_required(VERSION 3.10)
project(GTSAMGraphSLAM)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set GTSAM installation path
set(GTSAM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../gtsam/build")

# Find GTSAM
find_package(GTSAM REQUIRED HINTS ${GTSAM_DIR})

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${GTSAM_INCLUDE_DIR}
)

# Source files
set(SOURCES
    src/main.cpp
    src/graph_slam_2d.cpp
    src/graph_slam_3d.cpp
    src/g2o_reader.cpp
    src/utils/file_utils.cpp
)

# Create the executable
add_executable(graph_slam ${SOURCES})

# Link GTSAM libraries
target_link_libraries(graph_slam gtsam)

# Print configuration info
message(STATUS "GTSAM found at: ${GTSAM_DIR}")
message(STATUS "GTSAM include dir: ${GTSAM_INCLUDE_DIR}")