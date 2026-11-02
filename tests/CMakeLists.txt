find_package(Python3 REQUIRED COMPONENTS Interpreter)
add_test(NAME sapphire_module_boundaries_test COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/module_boundaries_test.py ${SAPPHIRE_REPOSITORY})
foreach(test recovery local_map parallel_executor lm_workspace handoff ricp frontend_preprocess imu_estimator point_observation voxel_covariance voxel_map_lifecycle odometry_rejection s3e_initializer)
  add_executable(sapphire_${test}_test ${test}_test.cpp)
  target_link_libraries(sapphire_${test}_test PRIVATE sapphire::core)
  target_compile_options(sapphire_${test}_test PRIVATE -UNDEBUG)
  add_test(NAME sapphire_${test}_test COMMAND sapphire_${test}_test)
endforeach()
set_tests_properties(sapphire_parallel_executor_test sapphire_lm_workspace_test PROPERTIES TIMEOUT 30)
foreach(test path_budget legacy_geometry lidar lidar_input parameters state_group db_worker link_storage statistics)
  add_executable(sapphire_backend_${test}_test backend/${test}_test.cpp)
  target_link_libraries(sapphire_backend_${test}_test PRIVATE sapphire::core gtsam SQLite::SQLite3)
  target_compile_options(sapphire_backend_${test}_test PRIVATE -UNDEBUG)
  add_test(NAME sapphire_backend_${test}_test COMMAND sapphire_backend_${test}_test)
endforeach()
add_executable(sapphire_pipeline_backend_test pipeline_backend_test.cpp)
target_link_libraries(sapphire_pipeline_backend_test PRIVATE sapphire::core)
add_test(NAME sapphire_backend_persistence_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/backend_persistence_test.py
  $<TARGET_FILE:sapphire_backend_lidar_test> $<TARGET_FILE:sapphire_pipeline_backend_test>
  ${SAPPHIRE_SOURCE}/config/params.toml)
set_tests_properties(sapphire_backend_persistence_test PROPERTIES TIMEOUT 360)
add_executable(sapphire_frontend_replay frontend_replay.cpp)
target_link_libraries(sapphire_frontend_replay PRIVATE sapphire::core)
add_test(NAME sapphire_export_scripts_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/export_scripts_test.py $<TARGET_FILE:sapphire_backend_lidar_test>)
set_tests_properties(sapphire_export_scripts_test PROPERTIES TIMEOUT 180
  ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1;PYTHONDONTWRITEBYTECODE=1")

target_include_directories(sapphire_odometry_rejection_test PRIVATE "${SAPPHIRE_SOURCE}/src")
target_include_directories(sapphire_pipeline_backend_test PRIVATE "${SAPPHIRE_SOURCE}/src")

add_executable(sapphire_backend_replay backend_replay.cpp)
target_link_libraries(sapphire_backend_replay PRIVATE sapphire::core SQLite::SQLite3)

set_tests_properties(sapphire_recovery_test PROPERTIES TIMEOUT 30)
add_test(NAME sapphire_recovery_persistence_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/recovery_persistence_test.py $<TARGET_FILE:sapphire_recovery_test>
  ${SAPPHIRE_SOURCE}/config/params.toml)
set_tests_properties(sapphire_recovery_persistence_test PROPERTIES TIMEOUT 90)

add_executable(sapphire_segments_test segments_test.cpp)
target_link_libraries(sapphire_segments_test PRIVATE sapphire::core)
target_compile_options(sapphire_segments_test PRIVATE -UNDEBUG)
add_test(NAME sapphire_segments_persistence_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/segments_persistence_test.py $<TARGET_FILE:sapphire_segments_test>
  $<TARGET_FILE:sapphire_segments> ${SAPPHIRE_SOURCE}/config/params.toml)
set_tests_properties(sapphire_segments_persistence_test PROPERTIES TIMEOUT 90)

add_executable(sapphire_automatic_relocalization_test automatic_relocalization_test.cpp)
target_link_libraries(sapphire_automatic_relocalization_test PRIVATE sapphire::core)
target_compile_options(sapphire_automatic_relocalization_test PRIVATE -UNDEBUG)
add_test(NAME sapphire_automatic_relocalization_persistence_test COMMAND ${Python3_EXECUTABLE}
  ${CMAKE_CURRENT_SOURCE_DIR}/automatic_relocalization_persistence_test.py $<TARGET_FILE:sapphire_automatic_relocalization_test>
  $<TARGET_FILE:sapphire_segments> ${SAPPHIRE_SOURCE}/config/params.toml)
set_tests_properties(sapphire_automatic_relocalization_persistence_test PROPERTIES TIMEOUT 180 ENVIRONMENT "OMP_NUM_THREADS=1;OPENBLAS_NUM_THREADS=1")
