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
set(CMAKE_CUDA_ARCHITECTURES 89 CACHE STRING "CUDA architectures for Sapphire 3D-BBS")
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

# One core library; frontend/backend directories contribute their owned sources.
include(GNUInstallDirs)
set(SAPPHIRE_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}")
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d highgui photo videoio calib3d)
find_package(OpenMP REQUIRED)
find_package(small_gicp REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(tomlplusplus 3.4.0 REQUIRED CONFIG)
find_package(SQLite3 REQUIRED)
find_package(ZLIB REQUIRED)
add_library(sapphire_core STATIC)
add_library(sapphire::core ALIAS sapphire_core)
target_include_directories(sapphire_core PUBLIC
  $<BUILD_INTERFACE:${SAPPHIRE_SOURCE}>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(sapphire_core PUBLIC
  Eigen3::Eigen Boost::headers ${OpenCV_LIBS} OpenMP::OpenMP_CXX
  small_gicp::small_gicp Threads::Threads spdlog::spdlog tomlplusplus::tomlplusplus)
add_subdirectory(thirdparty gtsam)
add_subdirectory(frontend)
add_subdirectory(backend)
add_subdirectory(app)
if(SAPPHIRE_ENABLE_LTO AND SAPPHIRE_IPO_SUPPORTED)
  set_property(TARGET sapphire_core PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif()

install(TARGETS sapphire_core ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY thirdparty/lie DESTINATION include/thirdparty FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
install(FILES thirdparty/nanoflann.hpp DESTINATION include/thirdparty)
# Retire artifacts from the former independently installed module packages.
install(CODE [[
  foreach(name common tools)
    file(REMOVE_RECURSE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/include/${name}")
  endforeach()
  foreach(name common tools frontend backend gtsam)
    file(REMOVE_RECURSE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/sapphire_${name}")
    file(REMOVE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libsapphire_${name}.a")
  endforeach()
]])

include(CTest)
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
  include(tests/CMakeLists.txt)
endif()

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
file(GLOB_RECURSE SAPPHIRE_FORMAT_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/backend/*.[ch]pp"
  "${CMAKE_CURRENT_SOURCE_DIR}/frontend/*.[ch]pp"
  "${CMAKE_CURRENT_SOURCE_DIR}/app/*.[ch]pp"
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

install(DIRECTORY config/
  DESTINATION share/sapphire_core/config
  FILES_MATCHING PATTERN "*.toml"
)

# Explicitly opt in: experimental Gaussian-to-PAGOR adapter, not production GEM.
option(SAPPHIRE_BUILD_G3REG_EXPERIMENTS "Build isolated G3Reg registration experiments" OFF)
if(BUILD_TESTING AND SAPPHIRE_BUILD_G3REG_EXPERIMENTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/g3reg/CMakeLists.txt")
  add_subdirectory(tests/g3reg)
endif()
