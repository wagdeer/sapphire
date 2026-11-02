# Existing serial G3Reg PAGOR/GNC subset. No upstream point-normal estimation.
find_package(PCL REQUIRED COMPONENTS common features search kdtree)
set(PAGOR_ROOT "${CMAKE_CURRENT_LIST_DIR}")
add_library(g3reg_pagor STATIC
  ${PAGOR_ROOT}/pmc/pmc_clique_utils.cpp ${PAGOR_ROOT}/pmc/pmc_cores.cpp
  ${PAGOR_ROOT}/pmc/pmc_graph.cpp ${PAGOR_ROOT}/pmc/pmc_heu.cpp
  ${PAGOR_ROOT}/pmc/pmc_utils.cpp ${PAGOR_ROOT}/pmc/pmcx_maxclique.cpp
  ${PAGOR_ROOT}/src/config.cpp ${PAGOR_ROOT}/src/graph_vertex.cpp
  ${PAGOR_ROOT}/src/pmc_solver.cpp ${PAGOR_ROOT}/src/registration.cpp ${PAGOR_ROOT}/src/opt_utils.cpp)
target_include_directories(g3reg_pagor PUBLIC ${PAGOR_ROOT}/include PRIVATE ${PAGOR_ROOT}/pmc)
target_include_directories(g3reg_pagor SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(g3reg_pagor PRIVATE EIGEN_DONT_PARALLELIZE)
target_link_libraries(g3reg_pagor PUBLIC Eigen3::Eigen PRIVATE sapphire::gtsam)
target_link_libraries(sapphire_core PRIVATE g3reg_pagor
  ${PCL_FEATURES_LIBRARY} ${PCL_SEARCH_LIBRARY} ${PCL_KDTREE_LIBRARY} ${PCL_COMMON_LIBRARY})
