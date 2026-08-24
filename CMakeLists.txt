cmake_minimum_required(VERSION 3.8)
project(sapphire)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(SQLite3 REQUIRED)

set(GTSAM_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_TIMING_ALWAYS OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_UNSTABLE OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
set(GTSAM_INSTALL_MATLAB_TOOLBOX OFF CACHE BOOL "" FORCE)
set(GTSAM_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GTSAM_ENABLE_BOOST_SERIALIZATION OFF CACHE BOOL "" FORCE)
set(GTSAM_USE_BOOST_FEATURES OFF CACHE BOOL "" FORCE)
set(GTSAM_USE_SYSTEM_EIGEN ON CACHE BOOL "" FORCE)
set(GTSAM_WITH_TBB OFF CACHE BOOL "" FORCE)
set(GTSAM_SUPPORT_NESTED_DISSECTION OFF CACHE BOOL "" FORCE)
set(GTSAM_FORCE_STATIC_LIB ON CACHE BOOL "" FORCE)

include(thirdparty/gtsam-4.3a2/minimal.cmake)

set(ROS_DEPENDENCIES
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_conversions
)

include_directories(
  src
  src/common
  src/odometry
  src/mapping
  ${PCL_INCLUDE_DIRS}
)

add_executable(sapphire
  src/pipeline.cpp
  src/mapping/pose_graph.cpp
)
ament_target_dependencies(sapphire ${ROS_DEPENDENCIES})
target_link_libraries(sapphire
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  gtsam
  OpenMP::OpenMP_CXX
  small_gicp::small_gicp
  Threads::Threads
  spdlog::spdlog
  SQLite::SQLite3
)

install(TARGETS
  sapphire
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
