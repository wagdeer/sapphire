cmake_minimum_required(VERSION 3.18)
project(g3reg LANGUAGES C CXX)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common search kdtree filters features)
# Use the same GTSAM target as Sapphire; standalone research builds reuse its
# existing minimal package, never a second vendor copy or a system GTSAM version.
if(NOT TARGET sapphire::gtsam)
  if(TARGET gtsam)
    add_library(sapphire::gtsam ALIAS gtsam)
  else()
    add_subdirectory(.. ${CMAKE_CURRENT_BINARY_DIR}/sapphire_gtsam)
  endif()
endif()
add_library(g3reg STATIC
  pmc/pmc_clique_utils.cpp
  pmc/pmc_cores.cpp
  pmc/pmc_graph.cpp
  pmc/pmc_heu.cpp
  pmc/pmc_utils.cpp
  pmc/pmcx_maxclique.cpp
  src/config.cpp
  src/fpfh_matcher.cpp
  src/fpfh_utils.cpp
  src/geo_verify.cpp
  src/graph_vertex.cpp
  src/opt_utils.cpp
  src/pmc_solver.cpp
  src/registration.cpp
  src/reglib.cpp)
add_library(G3REG::g3reg ALIAS g3reg)
target_compile_features(g3reg PUBLIC cxx_std_17)
target_compile_definitions(g3reg PRIVATE EIGEN_DONT_PARALLELIZE)
target_include_directories(g3reg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/pmc)
target_include_directories(g3reg SYSTEM PUBLIC ${PCL_COMMON_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
# PCL 1.12's exported aggregate/targets also attach optional VTK/Qt visualization
# libraries to these nonvisual components. Link only the required component files.
target_link_libraries(g3reg PUBLIC Eigen3::Eigen
  ${PCL_FEATURES_LIBRARY} ${PCL_FILTERS_LIBRARY} ${PCL_SEARCH_LIBRARY} ${PCL_KDTREE_LIBRARY} ${PCL_COMMON_LIBRARY}
  PRIVATE FLANN::FLANN sapphire::gtsam)
# No directory-wide flags, output paths, OpenMP pools, demos or downloads.
