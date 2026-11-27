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
find_package(PCL REQUIRED COMPONENTS common)
find_package(GTSAM REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)
find_package(TBB REQUIRED)

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
    src/io/tum.cpp
    src/backend/pose_graph.cpp
    src/backend/submap.cpp
    src/backend/submap_arxiv.cpp
    src/mapping/voxelmap.cpp
    src/odometry/ieskf.cpp
    src/mapping/occupancy_grid.cpp
    src/odometry/pipeline.cpp
    src/odometry/registration.cpp
    src/odometry/voxel_filter.cpp
)
target_include_directories(sapphire PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/external>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/sapphire/external>
)
target_link_libraries(sapphire PUBLIC
    Eigen3::Eigen
    spdlog::spdlog
    pcl_common
    gtsam
    OpenMP::OpenMP_CXX
    small_gicp::small_gicp
    Threads::Threads
    tomlplusplus::tomlplusplus
    PRIVATE TBB::tbb
)
sapphire_enable_release_ipo(sapphire)

option(SAPPHIRE_BUILD_BENCHMARKS "Build the NCLT replay benchmark" OFF)
if(SAPPHIRE_BUILD_BENCHMARKS)
    add_executable(sapphire_nclt_replay benchmarks/nclt_replay.cpp)
    target_link_libraries(sapphire_nclt_replay PRIVATE sapphire TBB::tbb)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(sapphire_voxelmap_io_test tests/voxelmap_io_test.cpp)
    target_link_libraries(sapphire_voxelmap_io_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_voxelmap_io_test)
    add_test(NAME sapphire_voxelmap_io_test COMMAND sapphire_voxelmap_io_test)

    add_executable(sapphire_submap_test tests/submap_test.cpp)
    target_link_libraries(sapphire_submap_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_submap_test)
    add_test(NAME sapphire_submap_test COMMAND sapphire_submap_test)

    add_executable(sapphire_registration_test tests/registration_test.cpp)
    target_link_libraries(sapphire_registration_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_registration_test)
    add_test(NAME sapphire_registration_test COMMAND sapphire_registration_test)

    add_executable(sapphire_pipeline_imu_boundary_test tests/pipeline_imu_boundary_test.cpp)
    target_link_libraries(sapphire_pipeline_imu_boundary_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_pipeline_imu_boundary_test)
    add_test(NAME sapphire_pipeline_imu_boundary_test COMMAND sapphire_pipeline_imu_boundary_test)

    add_executable(sapphire_cloud_ingest_test
        tests/cloud_ingest_test.cpp
    )
    target_include_directories(sapphire_cloud_ingest_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    sapphire_enable_release_ipo(sapphire_cloud_ingest_test)
    add_test(NAME sapphire_cloud_ingest_test COMMAND sapphire_cloud_ingest_test)

    add_executable(sapphire_tum_test
        tests/tum_test.cpp
    )
    target_link_libraries(sapphire_tum_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_tum_test)
    add_test(NAME sapphire_tum_test COMMAND sapphire_tum_test)

    add_executable(sapphire_config_test
        tests/config_test.cpp
    )
    target_link_libraries(sapphire_config_test PRIVATE sapphire)
    target_compile_definitions(sapphire_config_test PRIVATE
        SAPPHIRE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    )
    sapphire_enable_release_ipo(sapphire_config_test)
    add_test(NAME sapphire_config_test COMMAND sapphire_config_test)

    add_executable(sapphire_ring_buffer_test
        tests/ring_buffer_test.cpp
    )
    target_link_libraries(sapphire_ring_buffer_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_ring_buffer_test)
    add_test(
        NAME sapphire_ring_buffer_test
        COMMAND sapphire_ring_buffer_test
    )

    add_executable(sapphire_occupancy_grid_test
        tests/occupancy_grid_test.cpp
    )
    target_link_libraries(sapphire_occupancy_grid_test PRIVATE sapphire)
    sapphire_enable_release_ipo(sapphire_occupancy_grid_test)
    add_test(
        NAME sapphire_occupancy_grid_test
        COMMAND sapphire_occupancy_grid_test
    )

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

# Install
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/sapphireConfig.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/sapphireConfig.cmake"
    INSTALL_DESTINATION lib/cmake/sapphire)
write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/sapphireConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion)
install(TARGETS sapphire EXPORT sapphireTargets)
install(DIRECTORY include/ DESTINATION include
    PATTERN "benchmarks" EXCLUDE)
install(DIRECTORY external/tsl/
    DESTINATION include/sapphire/external/tsl
    FILES_MATCHING PATTERN "*.h")
install(DIRECTORY cfg/ DESTINATION share/sapphire/cfg)
install(DIRECTORY licenses/ DESTINATION share/sapphire/licenses)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/sapphireConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/sapphireConfigVersion.cmake"
    DESTINATION lib/cmake/sapphire)
install(EXPORT sapphireTargets
    FILE sapphireTargets.cmake
    DESTINATION lib/cmake/sapphire
)
