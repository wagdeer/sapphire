cmake_minimum_required(VERSION 3.18)
project(bbs3d LANGUAGES CXX CUDA)

find_package(Eigen3 REQUIRED)
find_package(CUDAToolkit REQUIRED)

set(CMAKE_CUDA_ARCHITECTURES 89 CACHE STRING "CUDA architectures for 3D-BBS")

add_library(gpu_bbs3d
  src/bbs3d.cu
  src/voxelmaps.cpp
)
target_compile_features(gpu_bbs3d PUBLIC cxx_std_17 cuda_std_17)
target_include_directories(gpu_bbs3d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gpu_bbs3d PUBLIC Eigen3::Eigen CUDA::cudart)
