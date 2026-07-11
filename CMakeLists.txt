cmake_minimum_required(VERSION 3.16)
project(sapphire VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies (only what the skeleton actually uses)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(PCL REQUIRED)
find_package(absl REQUIRED)

find_package(small_gicp REQUIRED)

# TOML configuration parser. Prefer a system package and fall back to the
# pinned upstream release for environments where Ubuntu does not package it.
find_package(tomlplusplus 3.4.0 QUIET)
if(NOT tomlplusplus_FOUND)
    include(FetchContent)
    FetchContent_Declare(tomlplusplus
        GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
        GIT_TAG v3.4.0
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(tomlplusplus)
endif()

# Library
add_library(sapphire STATIC
    src/config.cpp
    src/odometry/pipeline.cpp
    src/odometry/deskew.cpp
    src/odometry/registration.cpp
    src/odometry/submap.cpp
    src/imu_init.cpp
)
target_include_directories(sapphire PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external/preintegration>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(sapphire PUBLIC
    Eigen3::Eigen
    spdlog::spdlog
    ${PCL_LIBRARIES}
    absl::flat_hash_map
    small_gicp::small_gicp
    tomlplusplus::tomlplusplus
)

include(CTest)
if(BUILD_TESTING)
    add_executable(sapphire_config_test
        tests/config_test.cpp
    )
    target_link_libraries(sapphire_config_test PRIVATE sapphire)
    target_compile_definitions(sapphire_config_test PRIVATE
        SAPPHIRE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    add_test(NAME sapphire_config_test COMMAND sapphire_config_test)

    add_executable(sapphire_deskew_test
        tests/deskew_test.cpp
    )
    target_link_libraries(sapphire_deskew_test PRIVATE sapphire)
    add_test(NAME sapphire_deskew_test COMMAND sapphire_deskew_test)

    add_executable(sapphire_pipeline_test
        tests/pipeline_test.cpp
    )
    target_link_libraries(sapphire_pipeline_test PRIVATE sapphire)
    add_test(NAME sapphire_pipeline_test COMMAND sapphire_pipeline_test)

    add_executable(sapphire_submap_test
        tests/submap_test.cpp
    )
    target_link_libraries(sapphire_submap_test PRIVATE sapphire)
    add_test(NAME sapphire_submap_test COMMAND sapphire_submap_test)
endif()

# TODO: add as modules are implemented:
#   find_package(TBB)        → submap/odometry parallelism
#   find_package(GTSAM)      → PoseGraph ISAM2
# Install
install(TARGETS sapphire EXPORT sapphireTargets)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY cfg/ DESTINATION share/sapphire/cfg)
install(EXPORT sapphireTargets
    FILE sapphireTargets.cmake
    DESTINATION lib/cmake/sapphire
)
