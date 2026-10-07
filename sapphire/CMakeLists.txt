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

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs features2d calib3d video)
find_package(ZLIB REQUIRED)
find_package(OpenMP REQUIRED)
find_package(small_gicp REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(SQLite3 REQUIRED)
include(CheckCXXSourceCompiles)
include(CMakePushCheckState)
cmake_push_check_state(RESET)
set(CMAKE_REQUIRED_LIBRARIES SQLite::SQLite3)
check_cxx_source_compiles("#include <sqlite3.h>
int main() { return sqlite3_deserialize(nullptr, \"main\", nullptr, 0, 0, SQLITE_DESERIALIZE_READONLY); }
" SAPPHIRE_SQLITE_HAS_DESERIALIZE)
cmake_pop_check_state()
if(NOT SAPPHIRE_SQLITE_HAS_DESERIALIZE)
  message(FATAL_ERROR "Sapphire resume eligibility requires SQLite with sqlite3_deserialize support (without SQLITE_OMIT_DESERIALIZE).")
endif()
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

# CPU binary descriptor index; keep upstream options local.
function(sapphire_add_faiss)
  set(BUILD_TESTING OFF)
  set(BUILD_SHARED_LIBS OFF)
  foreach(feature GPU PYTHON MKL SVS METAL CUVS ROCM)
    set(FAISS_ENABLE_${feature} OFF)
  endforeach()
  set(FAISS_OPT_LEVEL generic)
  add_subdirectory(thirdparty/faiss EXCLUDE_FROM_ALL)
endfunction()
sapphire_add_faiss()

# Four implementation modules. The pipeline is the composition root only.
add_library(sapphire_tools STATIC
  src/tools/parallel_executor.cpp
)
add_library(sapphire::tools ALIAS sapphire_tools)
target_compile_features(sapphire_tools PUBLIC cxx_std_17)
target_include_directories(sapphire_tools PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sapphire_tools PUBLIC Threads::Threads)

add_library(sapphire_common STATIC
  src/parameters.cpp
  src/common/camera/calibration.cpp
  src/common/camera/pinhole.cpp
  src/common/camera/kb4.cpp
  src/common/camera/camera.cpp
  src/common/visual_frame.cpp
)
add_library(sapphire::common ALIAS sapphire_common)
target_compile_features(sapphire_common PUBLIC cxx_std_17)
target_include_directories(sapphire_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/3d_bbs/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/thirdparty/3d_bbs>
)
target_link_libraries(sapphire_common PUBLIC
  sapphire::tools
  Eigen3::Eigen Boost::headers CUDA::cudart opencv_core OpenMP::OpenMP_CXX
  small_gicp::small_gicp Threads::Threads spdlog::spdlog
  PRIVATE opencv_imgcodecs opencv_calib3d tomlplusplus::tomlplusplus
)

add_library(sapphire_frontend STATIC
  src/frontend/ricp/ricp.cpp
  src/frontend/common/synchronizer.cpp
)
add_library(sapphire::frontend ALIAS sapphire_frontend)
target_link_libraries(sapphire_frontend PUBLIC sapphire::common gtsam)

add_library(sapphire_backend STATIC
  src/backend/visual/feature/feature_block.cpp
  src/backend/visual/feature/visual_features.cpp
  src/backend/visual/feature/visual_tracker.cpp
  src/backend/visual/solver/visual_stereo.cpp
  src/backend/visual/feature/visual_keyframe.cpp
  src/backend/storage/scene_prefetch.cpp
  src/backend/storage/map_database.cpp
  src/backend/storage/visual_observation_archive.cpp
  src/backend/visual/feature/scene_features.cpp
  src/backend/visual/faiss/descriptor_archive.cpp
  src/backend/visual/feature/scene_archive.cpp
  src/backend/visual/visual_loop.cpp
  src/backend/visual/solver/visual_pnp.cpp
  src/backend/grid/flat_ground.cpp
  src/backend/grid/cape_ground.cpp
  src/backend/grid/ground_estimator.cpp
  src/backend/grid/rolling_grid.cpp
  src/backend/registration/loop_closure.cpp
  src/backend/registration/revisit_coverage.cpp
  src/backend/graph/pose_graph.cpp
  thirdparty/cape/CAPE.cpp
  thirdparty/cape/PlaneSeg.cpp
  thirdparty/cape/CylinderSeg.cpp
  thirdparty/cape/Histogram.cpp
  thirdparty/3d_bbs/src/bbs3d.cu
  thirdparty/3d_bbs/src/voxelmaps.cpp
)
add_library(sapphire::backend ALIAS sapphire_backend)
target_include_directories(sapphire_backend PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cape)
target_link_libraries(sapphire_backend PUBLIC sapphire::common
  PRIVATE gtsam faiss ZLIB::ZLIB opencv_features2d opencv_calib3d
  opencv_imgproc opencv_imgcodecs opencv_video SQLite::SQLite3
)

add_library(sapphire_core STATIC src/pipeline.cpp)
add_library(sapphire::core ALIAS sapphire_core)
target_link_libraries(sapphire_core PUBLIC sapphire::frontend sapphire::backend
  PRIVATE opencv_imgproc
)
if(SAPPHIRE_ENABLE_LTO AND SAPPHIRE_IPO_SUPPORTED)
  set_property(TARGET sapphire_tools sapphire_common sapphire_frontend sapphire_backend sapphire_core
    PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME sapphire_module_boundaries_test
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/module_boundaries_test.py
      ${CMAKE_CURRENT_SOURCE_DIR}/src)
  foreach(test ricp frontend_preprocess visual_image_attributes voxel_covariance visual_kernels imu_estimator point_observation odometry_rejection visual_mapping map_storage map_open map_eligibility historical_reconstruction writable_storage fresh_attachment continuation rolling_grid visual_tracker visual_pnp visual_camera visual_stereo visual_observation_archive automatic_association revisit_coverage)
    add_executable(sapphire_${test}_test tests/${test}_test.cpp)
    if(test MATCHES "^(ricp|frontend_preprocess|imu_estimator|point_observation|voxel_covariance)$")
      target_link_libraries(sapphire_${test}_test PRIVATE sapphire::frontend)
    elseif(test MATCHES "^(visual_kernels|visual_tracker|visual_camera|visual_stereo|visual_pnp|visual_observation_archive)$")
      target_link_libraries(sapphire_${test}_test PRIVATE sapphire::backend)
    else()
      target_link_libraries(sapphire_${test}_test PRIVATE sapphire::core)
    endif()
    target_compile_options(sapphire_${test}_test PRIVATE -UNDEBUG)
    add_test(NAME sapphire_${test}_test COMMAND sapphire_${test}_test)
  endforeach()
  set_tests_properties(sapphire_visual_image_attributes_test PROPERTIES TIMEOUT 90 LABELS "gpu")
  target_link_libraries(sapphire_visual_stereo_test PRIVATE opencv_imgcodecs)
  add_test(NAME sapphire_automatic_frontend_test COMMAND sapphire_automatic_association_test --frontend)
  add_test(NAME sapphire_scene_refresh_test COMMAND sapphire_automatic_association_test --refresh)
  target_link_options(sapphire_automatic_association_test PRIVATE "-Wl,--wrap=sqlite3_exec")
  set_tests_properties(sapphire_scene_refresh_test PROPERTIES TIMEOUT 180 LABELS "gpu")
  set_tests_properties(sapphire_automatic_frontend_test PROPERTIES TIMEOUT 120 LABELS "gpu")
  set_tests_properties(sapphire_automatic_association_test PROPERTIES TIMEOUT 120 LABELS "gpu")
  add_executable(sapphire_publication_test tests/publication_test.cpp)
  target_link_libraries(sapphire_publication_test PRIVATE sapphire::core gtsam)
  target_compile_options(sapphire_publication_test PRIVATE -O0)
  target_link_options(sapphire_publication_test PRIVATE "-Wl,--wrap=sqlite3_exec")
  add_test(NAME sapphire_publication_test COMMAND sapphire_publication_test)
  set_tests_properties(sapphire_publication_test PROPERTIES TIMEOUT 180 LABELS "gpu")
  target_link_libraries(sapphire_continuation_test PRIVATE gtsam)
  target_compile_options(sapphire_continuation_test PRIVATE -O0)
  target_link_options(sapphire_continuation_test PRIVATE "-Wl,--wrap=sqlite3_exec")
  set_tests_properties(sapphire_continuation_test PROPERTIES TIMEOUT 240 LABELS "gpu")
  add_test(NAME sapphire_metric_archive_continuation_test COMMAND sapphire_continuation_test --geometry-cycle)
  set_tests_properties(sapphire_metric_archive_continuation_test PROPERTIES TIMEOUT 60 LABELS "gpu")
  add_test(NAME sapphire_geometry_map_test COMMAND sapphire_continuation_test --geometry-cycle)
  set_tests_properties(sapphire_geometry_map_test PROPERTIES TIMEOUT 120 LABELS "gpu")
  target_link_libraries(sapphire_fresh_attachment_test PRIVATE gtsam)
  target_compile_options(sapphire_fresh_attachment_test PRIVATE -O0)
  target_link_options(sapphire_fresh_attachment_test PRIVATE "-Wl,--wrap=sqlite3_exec,--wrap=sqlite3_open_v2,--wrap=sqlite3_column_int64")
  set_tests_properties(sapphire_fresh_attachment_test PROPERTIES TIMEOUT 180 LABELS "gpu")
  target_link_libraries(sapphire_writable_storage_test PRIVATE gtsam)
  target_compile_options(sapphire_writable_storage_test PRIVATE -O0)
  target_link_options(sapphire_writable_storage_test PRIVATE "-Wl,--wrap=open,--wrap=close,--wrap=sqlite3_exec,--wrap=sqlite3_open_v2,--wrap=sqlite3_file_control,--wrap=sqlite3_vfs_find,--wrap=sqlite3_wal_checkpoint_v2,--wrap=sqlite3_step,--wrap=sqlite3_db_mutex")
  set_tests_properties(sapphire_writable_storage_test PROPERTIES TIMEOUT 120)
  add_test(NAME sapphire_writable_pipeline_test COMMAND sapphire_writable_storage_test --pipeline-gpu)
  set_tests_properties(sapphire_writable_pipeline_test PROPERTIES TIMEOUT 90 LABELS "gpu")
  target_link_libraries(sapphire_historical_reconstruction_test PRIVATE gtsam)
  target_link_options(sapphire_historical_reconstruction_test PRIVATE "-Wl,--wrap=cudaMalloc,--wrap=cudaGetDeviceCount,--wrap=cudaFree")
  set_tests_properties(sapphire_historical_reconstruction_test PROPERTIES TIMEOUT 90)
  add_test(NAME sapphire_historical_gpu_writer_test COMMAND sapphire_historical_reconstruction_test --gpu-cycle)
  set_tests_properties(sapphire_historical_gpu_writer_test PROPERTIES TIMEOUT 90 LABELS "gpu")
  # Fault injection and lifetime observation stay in the test executable, not the storage API.
  target_link_options(sapphire_map_eligibility_test PRIVATE
    "-Wl,--wrap=mmap,--wrap=mprotect,--wrap=munmap,--wrap=close"
    "-Wl,--wrap=sqlite3_open_v2,--wrap=sqlite3_deserialize,--wrap=sqlite3_close"
  )
  set_tests_properties(sapphire_map_eligibility_test PROPERTIES TIMEOUT 30)

  add_executable(sapphire_core_test tests/synchronizer_test.cpp)
  target_link_libraries(sapphire_core_test PRIVATE sapphire::core)
  target_compile_definitions(sapphire_core_test PRIVATE
    SAPPHIRE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
  add_test(NAME sapphire_visual_laser_loop_test COMMAND sapphire_visual_mapping_test --laser)
  add_test(NAME sapphire_core_test COMMAND sapphire_core_test)

  add_executable(sapphire_retrieval_loop_test tests/retrieval_loop_test.cpp)
  target_link_libraries(sapphire_retrieval_loop_test PRIVATE sapphire::core)
  add_test(NAME sapphire_retrieval_loop_test COMMAND sapphire_retrieval_loop_test)

  add_executable(sapphire_pyramid_voxel_test tests/pyramid_voxel_test.cpp)
  target_link_libraries(sapphire_pyramid_voxel_test PRIVATE sapphire::core)
  add_test(NAME sapphire_pyramid_voxel_test COMMAND sapphire_pyramid_voxel_test)

  add_executable(sapphire_loop_pipeline_verification_test tests/loop_pipeline_verification_test.cpp)
  target_link_libraries(sapphire_loop_pipeline_verification_test PRIVATE sapphire::core)
  add_test(NAME sapphire_loop_pipeline_verification_test COMMAND sapphire_loop_pipeline_verification_test)
  set_tests_properties(sapphire_loop_pipeline_verification_test PROPERTIES TIMEOUT 10)

  add_executable(sapphire_frontend_replay tests/frontend_replay.cpp)
  target_link_libraries(sapphire_frontend_replay PRIVATE sapphire::core)

  add_executable(sapphire_visual_retrieval_eval tests/visual_retrieval_eval.cpp)
  target_link_libraries(sapphire_visual_retrieval_eval PRIVATE sapphire::core)

  add_executable(sapphire_visual_tracking_eval tests/visual_tracking_eval.cpp)
  target_link_libraries(sapphire_visual_tracking_eval PRIVATE sapphire::core)

  add_executable(sapphire_real_submap_loop_eval tests/real_submap_loop_eval.cpp)
  target_link_libraries(sapphire_real_submap_loop_eval PRIVATE sapphire::core)
endif()

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format)
file(GLOB_RECURSE SAPPHIRE_FORMAT_FILES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/common/*.[ch]pp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/*.[ch]pp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/tools/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/backend/*.[ch]pp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/*.[ch]pp"
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

install(TARGETS sapphire_tools sapphire_common sapphire_frontend sapphire_backend sapphire_core
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
  src/tools
  src/frontend
  src/backend
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
install(DIRECTORY thirdparty/3d_bbs/include/
  DESTINATION include/thirdparty/3d_bbs
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h" PATTERN "*.cuh"
)
install(DIRECTORY thirdparty/lie
  DESTINATION include/thirdparty
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
install(FILES thirdparty/nanoflann.hpp
  DESTINATION include/thirdparty
)
install(DIRECTORY config/
  DESTINATION share/sapphire_core/config
  FILES_MATCHING PATTERN "*.toml"
)
