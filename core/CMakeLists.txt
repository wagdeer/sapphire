cmake_minimum_required(VERSION 3.18)

if(NOT DEFINED CMAKE_CUDA_COMPILER)
  find_program(CMAKE_CUDA_COMPILER
    NAMES nvcc
    HINTS
      "$ENV{CUDA_HOME}/bin"
      "$ENV{CUDA_PATH}/bin"
      /usr/local/cuda/bin
    REQUIRED
  )
endif()

set(CMAKE_CUDA_ARCHITECTURES 89 CACHE STRING "CUDA architectures for Sapphire 3D-BBS")
project(sapphire_core VERSION 0.1.0 LANGUAGES C CXX CUDA)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_EXTENSIONS OFF)
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

# One core library, with headers and implementations grouped by function.
include(GNUInstallDirs)
set(SAPPHIRE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}")
get_filename_component(SAPPHIRE_REPOSITORY "${SAPPHIRE_SOURCE}/.." ABSOLUTE)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(OpenMP REQUIRED)
find_package(small_gicp REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(tomlplusplus 3.4.0 REQUIRED CONFIG)
find_package(SQLite3 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(CUDAToolkit REQUIRED)
add_library(sapphire_core SHARED)
set_target_properties(sapphire_core PROPERTIES PREFIX "" OUTPUT_NAME "sapphire_core" INSTALL_RPATH_USE_LINK_PATH TRUE)
add_library(sapphire::core ALIAS sapphire_core)
target_include_directories(sapphire_core PUBLIC
  $<BUILD_INTERFACE:${SAPPHIRE_SOURCE}>
  $<BUILD_INTERFACE:${SAPPHIRE_SOURCE}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(sapphire_core PUBLIC
  Eigen3::Eigen Boost::headers ${OpenCV_LIBS} OpenMP::OpenMP_CXX
  small_gicp::small_gicp Threads::Threads spdlog::spdlog tomlplusplus::tomlplusplus)
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS "${SAPPHIRE_SOURCE}/src/*.cpp")
target_sources(sapphire_core PRIVATE ${CORE_SOURCES})
include(thirdparty/gtsam-4.3a2/minimal.cmake)
add_library(sapphire::gtsam ALIAS gtsam)
include(thirdparty/g3reg/pagor.cmake)
configure_file(src/database/schema.sql.in DatabaseSchema.sql @ONLY)
file(READ "${CMAKE_CURRENT_BINARY_DIR}/DatabaseSchema.sql" SCHEMA_HEX HEX)
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/DatabaseSchema_sql.h" "#pragma once\nnamespace sapphire { static const char * DATABASESCHEMA_SQL = \"${SCHEMA_HEX}\"; }\n")
target_sources(sapphire_core PRIVATE
  "${SAPPHIRE_SOURCE}/thirdparty/3d_bbs/src/voxelmaps.cpp"
  "${SAPPHIRE_SOURCE}/thirdparty/3d_bbs/src/bbs3d.cu")
target_include_directories(sapphire_core PUBLIC
  $<BUILD_INTERFACE:${SAPPHIRE_SOURCE}/thirdparty/3d_bbs/include>
  $<INSTALL_INTERFACE:include/thirdparty/3d_bbs>
  PRIVATE "${SAPPHIRE_SOURCE}/thirdparty" "${CMAKE_CURRENT_BINARY_DIR}")
target_compile_definitions(sapphire_core PUBLIC EIGEN_DONT_PARALLELIZE)
target_link_libraries(sapphire_core PUBLIC sapphire::gtsam CUDA::cudart
  PRIVATE ZLIB::ZLIB SQLite::SQLite3 ${CMAKE_DL_LIBS})
install(DIRECTORY "${SAPPHIRE_SOURCE}/thirdparty/3d_bbs/include/" DESTINATION include/thirdparty/3d_bbs
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cuh")
if(SAPPHIRE_ENABLE_LTO AND SAPPHIRE_IPO_SUPPORTED)
  set_property(TARGET sapphire_core PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif()

install(TARGETS sapphire_core
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(TARGETS gtsam ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY thirdparty/gtsam-4.3a2/gtsam DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.inl")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/gtsam/config.h" "${CMAKE_CURRENT_BINARY_DIR}/gtsam/dllexport.h"
  DESTINATION include/gtsam)
install(DIRECTORY thirdparty/lie DESTINATION include/thirdparty FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(FILES thirdparty/nanoflann/nanoflann.hpp DESTINATION include/thirdparty/nanoflann)
install(DIRECTORY include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")

include(CTest)
if(BUILD_TESTING)
  add_subdirectory("${SAPPHIRE_REPOSITORY}/tests" tests)
endif()

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
file(GLOB_RECURSE SAPPHIRE_FORMAT_FILES CONFIGURE_DEPENDS
  "${SAPPHIRE_SOURCE}/include/*.[ch]pp"
  "${SAPPHIRE_SOURCE}/src/*.[ch]pp"
  "${SAPPHIRE_REPOSITORY}/ros2_warpper/*.[ch]pp"
  "${SAPPHIRE_REPOSITORY}/tests/*.[ch]pp"
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

install(DIRECTORY config/
  DESTINATION share/sapphire_core/config
  FILES_MATCHING PATTERN "*.toml"
)

# Optional post-run segment inspection/metric joining, not a sensor adapter.
add_executable(sapphire_segments apps/segments.cpp)
target_link_libraries(sapphire_segments PRIVATE sapphire::core SQLite::SQLite3)
set_target_properties(sapphire_segments PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
install(TARGETS sapphire_segments RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
