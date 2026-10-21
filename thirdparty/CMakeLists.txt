include(gtsam-4.3a2/minimal.cmake)
add_library(sapphire::gtsam ALIAS gtsam)
install(TARGETS gtsam ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${SAPPHIRE_SOURCE}/thirdparty/gtsam-4.3a2/gtsam DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.inl")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gtsam/config.h ${CMAKE_CURRENT_BINARY_DIR}/gtsam/dllexport.h DESTINATION include/gtsam)
