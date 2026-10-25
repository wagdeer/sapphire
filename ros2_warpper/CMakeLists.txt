cmake_minimum_required(VERSION 3.16)
project(sapphire_ros2 VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

get_filename_component(SAPPHIRE_REPOSITORY "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(SAPPHIRE_CORE_SOURCE "${SAPPHIRE_REPOSITORY}/core")
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
set_target_properties(sapphire PROPERTIES INSTALL_RPATH "$ORIGIN/..")
target_compile_features(sapphire PRIVATE cxx_std_17)
target_include_directories(sapphire PRIVATE
  include
)
target_link_libraries(sapphire
  sapphire::core
)
ament_target_dependencies(sapphire
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
)

if(SAPPHIRE_ENABLE_LTO AND SAPPHIRE_IPO_SUPPORTED)
  set_property(
    TARGET sapphire
    PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE
  )
endif()

install(TARGETS sapphire DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch FILES_MATCHING PATTERN "*.py" PATTERN "__pycache__" EXCLUDE)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(FILES
  "${SAPPHIRE_CORE_SOURCE}/config/sensors.toml"
  "${SAPPHIRE_CORE_SOURCE}/config/params.toml"
  DESTINATION share/${PROJECT_NAME}/config
)

include(CTest)
if(BUILD_TESTING AND EXISTS "${SAPPHIRE_REPOSITORY}/tests/ros2_tests.cmake")
  include("${SAPPHIRE_REPOSITORY}/tests/ros2_tests.cmake")
endif()

ament_package()
