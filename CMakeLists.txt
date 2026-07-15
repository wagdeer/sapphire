cmake_minimum_required(VERSION 3.16)
project(sapphire VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build." FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(SAPPHIRE_ENABLE_LTO
    "Enable IPO/LTO for Sapphire Release targets when supported" ON)

set(SAPPHIRE_IPO_SUPPORTED FALSE)
if(SAPPHIRE_ENABLE_LTO)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT SAPPHIRE_IPO_SUPPORTED
        OUTPUT SAPPHIRE_IPO_ERROR LANGUAGES CXX)
    if(NOT SAPPHIRE_IPO_SUPPORTED)
        message(WARNING
            "SAPPHIRE_ENABLE_LTO is ON, but IPO/LTO is unavailable: "
            "${SAPPHIRE_IPO_ERROR}")
    endif()
endif()

function(sapphire_enable_release_ipo target)
    if(SAPPHIRE_ENABLE_LTO AND SAPPHIRE_IPO_SUPPORTED)
        set_property(TARGET ${target}
            PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    endif()
endfunction()

# Dependencies (only what the skeleton actually uses)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

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
    src/backend/pose_graph.cpp
    src/odometry/pipeline.cpp
    src/odometry/deskew.cpp
    src/odometry/observer.cpp
    src/odometry/registration.cpp
    src/odometry/submap.cpp
    src/odometry/voxel_filter.cpp
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
    gtsam
    OpenMP::OpenMP_CXX
    small_gicp::small_gicp
    Threads::Threads
    tomlplusplus::tomlplusplus
)
sapphire_enable_release_ipo(sapphire)

include(CTest)
if(BUILD_TESTING)
    add_executable(sapphire_config_test
        tests/config_test.cpp
    )
    target_link_libraries(sapphire_config_test PRIVATE sapphire)
    target_compile_definitions(sapphire_config_test PRIVATE
        SAPPHIRE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    sapphire_enable_release_ipo(sapphire_config_test)
    add_test(NAME sapphire_config_test COMMAND sapphire_config_test)

    add_executable(sapphire_deskew_test
        tests/deskew_test.cpp
    )
    target_link_libraries(sapphire_deskew_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_deskew_test)
    add_test(NAME sapphire_deskew_test COMMAND sapphire_deskew_test)

    add_executable(sapphire_ring_buffer_test
        tests/ring_buffer_test.cpp
    )
    target_link_libraries(sapphire_ring_buffer_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_ring_buffer_test)
    add_test(
        NAME sapphire_ring_buffer_test
        COMMAND sapphire_ring_buffer_test
    )

    add_executable(sapphire_preintegration_test
        tests/preintegration_test.cpp
    )
    target_link_libraries(sapphire_preintegration_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_preintegration_test)
    add_test(
        NAME sapphire_preintegration_test
        COMMAND sapphire_preintegration_test
    )

    add_executable(sapphire_mean_only_gal3_integrator_test
        tests/mean_only_gal3_integrator_test.cpp
    )
    target_link_libraries(
        sapphire_mean_only_gal3_integrator_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_mean_only_gal3_integrator_test)
    add_test(
        NAME sapphire_mean_only_gal3_integrator_test
        COMMAND sapphire_mean_only_gal3_integrator_test
    )

    add_executable(sapphire_pipeline_test
        tests/pipeline_test.cpp
    )
    target_link_libraries(sapphire_pipeline_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_pipeline_test)
    add_test(NAME sapphire_pipeline_test COMMAND sapphire_pipeline_test)

    add_executable(sapphire_pose_graph_test
        tests/pose_graph_test.cpp
    )
    target_link_libraries(sapphire_pose_graph_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_pose_graph_test)
    add_test(NAME sapphire_pose_graph_test COMMAND sapphire_pose_graph_test)

    add_executable(sapphire_observer_test
        tests/observer_test.cpp
    )
    target_link_libraries(sapphire_observer_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_observer_test)
    add_test(NAME sapphire_observer_test COMMAND sapphire_observer_test)

    add_executable(sapphire_registration_test
        tests/registration_test.cpp
    )
    target_link_libraries(sapphire_registration_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_registration_test)
    add_test(NAME sapphire_registration_test COMMAND sapphire_registration_test)

    add_executable(sapphire_submap_test
        tests/submap_test.cpp
    )
    target_link_libraries(sapphire_submap_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_submap_test)
    add_test(NAME sapphire_submap_test COMMAND sapphire_submap_test)

    add_executable(sapphire_voxel_filter_test
        tests/voxel_filter_test.cpp
    )
    target_link_libraries(sapphire_voxel_filter_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_voxel_filter_test)
    add_test(
        NAME sapphire_voxel_filter_test
        COMMAND sapphire_voxel_filter_test
    )
endif()

# TODO: add as modules are implemented:
#   find_package(TBB)        → submap/odometry parallelism
# Install
install(TARGETS sapphire EXPORT sapphireTargets)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY cfg/ DESTINATION share/sapphire/cfg)
install(EXPORT sapphireTargets
    FILE sapphireTargets.cmake
    DESTINATION lib/cmake/sapphire
)
