cmake_minimum_required(VERSION 3.16)
project(sapphire_core VERSION 0.1.0 LANGUAGES C CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(SAPPHIRE_ENABLE_LTO "Enable IPO/LTO for Release builds" ON)

set(SAPPHIRE_IPO_SUPPORTED FALSE)
if(SAPPHIRE_ENABLE_LTO)
  include(CheckIPOSupported)
  check_ipo_supported(
    RESULT SAPPHIRE_IPO_SUPPORTED
    OUTPUT SAPPHIRE_IPO_ERROR
    LANGUAGES CXX
  )
  if(NOT SAPPHIRE_IPO_SUPPORTED)
    message(WARNING "IPO/LTO is unavailable: ${SAPPHIRE_IPO_ERROR}")
  endif()
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(SQLite3 REQUIRED)
find_package(tomlplusplus 3.4.0 REQUIRED CONFIG)

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

add_library(sapphire_core STATIC
  src/pipeline.cpp
  src/parameters.cpp
  src/mapping/pose_graph.cpp
)
add_library(sapphire::core ALIAS sapphire_core)

target_compile_features(sapphire_core PUBLIC cxx_std_17)
target_include_directories(sapphire_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/common>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/odometry>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/mapping>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/common>
    $<INSTALL_INTERFACE:include/odometry>
    $<INSTALL_INTERFACE:include/mapping>
)
target_link_libraries(sapphire_core
  PUBLIC
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    Threads::Threads
    spdlog::spdlog
  PRIVATE
    gtsam
    small_gicp::small_gicp
    SQLite::SQLite3
    tomlplusplus::tomlplusplus
)

if(SAPPHIRE_ENABLE_LTO AND SAPPHIRE_IPO_SUPPORTED)
  set_property(
    TARGET sapphire_core
    PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE
  )
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(sapphire_core_test tests/synchronizer_test.cpp)
  target_link_libraries(sapphire_core_test PRIVATE sapphire::core)
  target_compile_definitions(sapphire_core_test PRIVATE
    SAPPHIRE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(NAME sapphire_core_test COMMAND sapphire_core_test)
endif()

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
file(GLOB_RECURSE SAPPHIRE_FORMAT_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/common/*.[ch]pp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/mapping/*.[ch]pp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/odometry/*.[ch]pp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/pipeline.*"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/parameters.*"
  "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.[ch]pp"
)
if(CLANG_FORMAT_EXECUTABLE)
  add_custom_target(format
    COMMAND "${CLANG_FORMAT_EXECUTABLE}" -i ${SAPPHIRE_FORMAT_FILES}
    COMMENT "Formatting Sapphire core sources"
    VERBATIM
  )
else()
  add_custom_target(format
    COMMAND "${CMAKE_COMMAND}" -E echo "clang-format not found; skipping"
    VERBATIM
  )
endif()

install(TARGETS sapphire_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES
  src/pipeline.hpp
  src/parameters.h
  DESTINATION include
)
install(DIRECTORY
  src/common
  src/odometry
  src/mapping
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
install(DIRECTORY thirdparty/lie
  DESTINATION include/thirdparty
  FILES_MATCHING PATTERN "*.hpp"
)
install(FILES thirdparty/nanoflann.hpp
  DESTINATION include/thirdparty
)
install(DIRECTORY config/
  DESTINATION share/sapphire_core/config
  FILES_MATCHING PATTERN "*.toml"
)
