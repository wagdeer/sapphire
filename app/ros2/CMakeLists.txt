cmake_minimum_required(VERSION 3.16)
project(sapphire_ros2 VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RevisionedMapCorrection.msg" "msg/RevisionedMapPose.msg"
  "msg/MapPublicationStatus.msg" "msg/RevisionedNavigationGrid.msg"
  DEPENDENCIES geometry_msgs nav_msgs)
rosidl_get_typesupport_target(b3_typesupport ${PROJECT_NAME} rosidl_typesupport_cpp)

set(SAPPHIRE_CORE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/../..")
if(NOT EXISTS "${SAPPHIRE_CORE_SOURCE}/CMakeLists.txt")
  message(FATAL_ERROR
    "sapphire_ros2 requires the repository Sapphire core project at "
    "${SAPPHIRE_CORE_SOURCE}"
  )
endif()

add_subdirectory(
  "${SAPPHIRE_CORE_SOURCE}"
  "${CMAKE_CURRENT_BINARY_DIR}/sapphire_core"
)

add_executable(sapphire
  src/main.cpp
  src/sapphire_node.cpp
)
add_dependencies(sapphire ${PROJECT_NAME}__rosidl_typesupport_fastrtps_cpp ${PROJECT_NAME}__rosidl_typesupport_introspection_cpp)
target_compile_features(sapphire PRIVATE cxx_std_17)
target_include_directories(sapphire PRIVATE
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(sapphire
  sapphire::core
  ${b3_typesupport}
  opencv_imgcodecs
  opencv_imgproc
  ${PCL_LIBRARIES}
)
ament_target_dependencies(sapphire
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_conversions
)

if(SAPPHIRE_ENABLE_LTO AND SAPPHIRE_IPO_SUPPORTED)
  set_property(
    TARGET sapphire
    PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE
  )
endif()

install(TARGETS sapphire DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz_cfg/ DESTINATION share/${PROJECT_NAME}/rviz_cfg)
install(FILES "${SAPPHIRE_CORE_SOURCE}/config/mid360.toml" "${SAPPHIRE_CORE_SOURCE}/config/dual.toml" "${SAPPHIRE_CORE_SOURCE}/config/mid360_visual.toml" "${SAPPHIRE_CORE_SOURCE}/config/hilti2022_exp09.toml"
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
  include(tests/CMakeLists.txt)
endif()

ament_package()
