target_sources(sapphire_core PRIVATE
  src/runtime.cpp
  src/ricp/ricp.cpp
  src/common/synchronizer.cpp
  src/tools/parallel_executor.cpp
)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/include")
file(CREATE_LINK "${CMAKE_CURRENT_SOURCE_DIR}/include" "${CMAKE_CURRENT_BINARY_DIR}/include/frontend" SYMBOLIC)
target_include_directories(sapphire_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)
install(CODE [[file(REMOVE_RECURSE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/include/frontend")]])
install(DIRECTORY include/ DESTINATION include/frontend FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h")
