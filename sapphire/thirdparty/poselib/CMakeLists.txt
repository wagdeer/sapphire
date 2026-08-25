cmake_minimum_required(VERSION 3.10)
project(PoseLibPlus LANGUAGES CXX)

find_package(Eigen3 REQUIRED)

option(POSELIBPLUS_BUILD_TESTS "Build PoseLibPlus synthetic tests." OFF)

set(MAX_STURM_RECURSION_DEPTH_LIMIT 300 CACHE STRING
    "Maximum recursion depth used by the five-point Sturm solver.")

add_library(poselibplus STATIC
    PoseLib/pose_estimation.cpp
    PoseLib/solvers/p3p.cc
    PoseLib/solvers/gp3p.cc
    PoseLib/solvers/relpose_5pt.cc
    PoseLib/misc/essential.cc
    PoseLib/misc/re3q3.cc
    PoseLib/misc/univariate.cc
    PoseLib/robust/sampling.cc
    PoseLib/robust/robust_loss.cc
)
add_library(poselibplus::poselibplus ALIAS poselibplus)

set_target_properties(poselibplus PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

target_include_directories(poselibplus PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

target_compile_definitions(poselibplus PUBLIC
    MAX_STURM_RECURSION_DEPTH_LIMIT=${MAX_STURM_RECURSION_DEPTH_LIMIT})

target_link_libraries(poselibplus PUBLIC Eigen3::Eigen)