cmake_minimum_required(VERSION 3.8)

# Suppress CMake policy warnings globally
cmake_policy(SET CMP0144 NEW)
cmake_policy(SET CMP0074 NEW)

project(stereo_vision)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
# PCL is optional for basic functionality
find_package(pcl_ros QUIET)
find_package(pcl_conversions QUIET)

# Find OpenCV with contrib modules
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL QUIET COMPONENTS common io filters)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  /opt/ros/jazzy/include
)

# Set dependencies list
set(dependencies
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  message_filters
)

# Add PCL dependencies if available
if(pcl_ros_FOUND)
  list(APPEND dependencies pcl_ros)
endif()
if(pcl_conversions_FOUND)
  list(APPEND dependencies pcl_conversions)
endif()

# Create stereo vision library
add_library(stereo_vision_lib SHARED
  src/stereo_rectifier.cpp
  src/feature_extractor.cpp
  src/feature_matcher.cpp
  src/triangulator.cpp
  src/disparity_computer.cpp
  src/dense_reconstructor.cpp
  src/pose_estimator.cpp
  src/utils.cpp
)

target_link_libraries(stereo_vision_lib
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

# Add PCL libraries if available
if(PCL_FOUND)
  target_link_libraries(stereo_vision_lib ${PCL_LIBRARIES})
endif()

ament_target_dependencies(stereo_vision_lib ${dependencies})

# Stereo Rectifier Node (original - proper implementation)
add_executable(stereo_rectifier_node src/nodes/stereo_rectifier_node.cpp)
target_link_libraries(stereo_rectifier_node stereo_vision_lib ${OpenCV_LIBRARIES})
ament_target_dependencies(stereo_rectifier_node ${dependencies})

# Feature Extractor Node
add_executable(feature_extractor_node src/nodes/feature_extractor_node.cpp)
target_link_libraries(feature_extractor_node stereo_vision_lib ${OpenCV_LIBRARIES})
ament_target_dependencies(feature_extractor_node ${dependencies})

# Feature Matcher Node
add_executable(feature_matcher_node src/nodes/feature_matcher_node.cpp)
target_link_libraries(feature_matcher_node stereo_vision_lib ${OpenCV_LIBRARIES})
ament_target_dependencies(feature_matcher_node ${dependencies})

# Triangulator Node
add_executable(triangulator_node src/nodes/triangulator_node.cpp)
target_link_libraries(triangulator_node stereo_vision_lib ${OpenCV_LIBRARIES})
ament_target_dependencies(triangulator_node ${dependencies})

# Mapping Node
add_executable(mapping_node src/nodes/mapping_node.cpp)
target_link_libraries(mapping_node stereo_vision_lib ${OpenCV_LIBRARIES})
if(PCL_FOUND)
  target_link_libraries(mapping_node ${PCL_LIBRARIES})
endif()
ament_target_dependencies(mapping_node ${dependencies})

# Disparity Node
add_executable(disparity_node src/nodes/disparity_node.cpp)
target_link_libraries(disparity_node stereo_vision_lib ${OpenCV_LIBRARIES})
ament_target_dependencies(disparity_node ${dependencies})

# Dense Reconstruction Node
add_executable(dense_reconstruction_node src/nodes/dense_reconstruction_node.cpp)
target_link_libraries(dense_reconstruction_node stereo_vision_lib ${OpenCV_LIBRARIES})
if(PCL_FOUND)
  target_link_libraries(dense_reconstruction_node ${PCL_LIBRARIES})
endif()
ament_target_dependencies(dense_reconstruction_node ${dependencies})

# Pose Estimator Node (Monocular Visual Odometry)
add_executable(pose_estimation_node src/nodes/pose_estimation_node.cpp)
target_link_libraries(pose_estimation_node stereo_vision_lib ${OpenCV_LIBRARIES})
ament_target_dependencies(pose_estimation_node ${dependencies})

# Feature Mapper Node (Optional)
# add_executable(feature_mapper_node src/nodes/feature_mapper_node.cpp)
# target_link_libraries(feature_mapper_node stereo_vision_lib ${OpenCV_LIBRARIES})
# if(PCL_FOUND)
#   target_link_libraries(feature_mapper_node ${PCL_LIBRARIES})
# endif()
# ament_target_dependencies(feature_mapper_node ${dependencies})

# Dense Mapper Node
add_executable(dense_mapper_node src/nodes/dense_mapper_node.cpp)
target_link_libraries(dense_mapper_node stereo_vision_lib ${OpenCV_LIBRARIES})
if(PCL_FOUND)
  target_link_libraries(dense_mapper_node ${PCL_LIBRARIES})
endif()
ament_target_dependencies(dense_mapper_node ${dependencies})

# Stereo Pipeline Node (combines multiple steps)
# add_executable(stereo_pipeline_node src/nodes/stereo_pipeline_node.cpp)
# target_link_libraries(stereo_pipeline_node stereo_vision_lib ${OpenCV_LIBRARIES})
# if(PCL_FOUND)
#   target_link_libraries(stereo_pipeline_node ${PCL_LIBRARIES})
# endif()
# ament_target_dependencies(stereo_pipeline_node ${dependencies})

# Install targets
install(TARGETS
  stereo_vision_lib
  stereo_rectifier_node
  feature_extractor_node
  feature_matcher_node
  triangulator_node
  mapping_node
  disparity_node
  dense_reconstruction_node
  pose_estimation_node
  dense_mapper_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

