# Keep this list explicit: adding a new GTSAM API requires extending its source
# and header dependency closure deliberately.

set(_gtsam_root "${CMAKE_CURRENT_LIST_DIR}")

if(NOT TARGET Eigen3::Eigen)
  find_package(Eigen3 REQUIRED)
endif()

set(GTSAM_VERSION_MAJOR 4)
set(GTSAM_VERSION_MINOR 3)
set(GTSAM_VERSION_PATCH 0)
set(GTSAM_VERSION_NUMERIC 40300)
set(GTSAM_VERSION_STRING "4.3a2")
set(GTSAM_SOURCE_DIR "${_gtsam_root}")
set(GTSAM_TOOLBOX_INSTALL_PATH "")

# These values must stay aligned with the source list below.
set(GTSAM_USE_SYSTEM_EIGEN ON)
set(GTSAM_POSE3_EXPMAP ON)
set(GTSAM_ROT3_EXPMAP ON)
set(GTSAM_DT_MERGING ON)
set(GTSAM_HYBRID_TIMING OFF)
set(GTSAM_ALLOCATOR_STL ON)
set(GTSAM_THROW_CHEIRALITY_EXCEPTION ON)
set(GTSAM_ALLOW_DEPRECATED_SINCE_V43 ON)
set(GTSAM_TANGENT_PREINTEGRATION ON)
set(GTSAM_ENABLE_BOOST_SERIALIZATION OFF)
set(GTSAM_USE_BOOST_FEATURES OFF)
set(GTSAM_SHARED_LIB OFF)

file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/gtsam")
configure_file(
  "${_gtsam_root}/gtsam/config.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/gtsam/config.h"
)
configure_file(
  "${_gtsam_root}/gtsam/dllexport.h.in"
  "${CMAKE_CURRENT_BINARY_DIR}/gtsam/dllexport.h"
)

set(_gtsam_sources
  # SuiteSparse ordering.
  gtsam/3rdparty/CCOLAMD/Source/ccolamd.c
  gtsam/3rdparty/SuiteSparse_config/SuiteSparse_config.c

  # Cephes is retained as a small compatibility dependency for GTSAM headers
  # and future robust/noise-model paths.
  gtsam/3rdparty/cephes/cephes/const.c
  gtsam/3rdparty/cephes/cephes/gamma.c
  gtsam/3rdparty/cephes/cephes/igam.c
  gtsam/3rdparty/cephes/cephes/igami.c
  gtsam/3rdparty/cephes/cephes/lanczos.c
  gtsam/3rdparty/cephes/cephes/sf_error.c
  gtsam/3rdparty/cephes/cephes/unity.c
  gtsam/3rdparty/cephes/cephes/zeta.c

  gtsam/base/Matrix.cpp
  gtsam/base/SymmetricBlockMatrix.cpp
  gtsam/base/Vector.cpp
  gtsam/base/VerticalBlockMatrix.cpp
  gtsam/base/cholesky.cpp
  gtsam/base/debug.cpp
  gtsam/base/timing.cpp
  gtsam/base/types.cpp
  gtsam/base/utilities.cpp

  gtsam/discrete/DecisionTreeFactor.cpp
  gtsam/discrete/DiscreteBayesNet.cpp
  gtsam/discrete/DiscreteBayesTree.cpp
  gtsam/discrete/DiscreteConditional.cpp
  gtsam/discrete/DiscreteEliminationTree.cpp
  gtsam/discrete/DiscreteFactor.cpp
  gtsam/discrete/DiscreteFactorGraph.cpp
  gtsam/discrete/DiscreteJunctionTree.cpp
  gtsam/discrete/DiscreteKey.cpp
  gtsam/discrete/DiscreteLookupDAG.cpp
  gtsam/discrete/DiscreteMarginals.cpp
  gtsam/discrete/DiscreteValues.cpp
  gtsam/discrete/Signature.cpp
  gtsam/discrete/SignatureParser.cpp
  gtsam/discrete/TableDistribution.cpp
  gtsam/discrete/TableFactor.cpp

  gtsam/geometry/Kernel.cpp
  gtsam/geometry/Point3.cpp
  gtsam/geometry/Pose3.cpp
  gtsam/geometry/Rot3.cpp
  gtsam/geometry/Rot3M.cpp
  gtsam/geometry/SO3.cpp
  gtsam/geometry/Unit3.cpp

  gtsam/hybrid/HybridBayesNet.cpp
  gtsam/hybrid/HybridBayesTree.cpp
  gtsam/hybrid/HybridConditional.cpp
  gtsam/hybrid/HybridEliminationTree.cpp
  gtsam/hybrid/HybridFactor.cpp
  gtsam/hybrid/HybridFactorGraph.cpp
  gtsam/hybrid/HybridGaussianConditional.cpp
  gtsam/hybrid/HybridGaussianFactor.cpp
  gtsam/hybrid/HybridGaussianFactorGraph.cpp
  gtsam/hybrid/HybridGaussianProductFactor.cpp
  gtsam/hybrid/HybridJunctionTree.cpp
  gtsam/hybrid/HybridValues.cpp

  gtsam/inference/BayesTree.cpp
  gtsam/inference/DotWriter.cpp
  gtsam/inference/Factor.cpp
  gtsam/inference/Key.cpp
  gtsam/inference/LabeledSymbol.cpp
  gtsam/inference/Ordering.cpp
  gtsam/inference/Symbol.cpp
  gtsam/inference/VariableIndex.cpp
  gtsam/inference/VariableSlots.cpp
  gtsam/inference/inferenceExceptions.cpp

  gtsam/linear/Errors.cpp
  gtsam/linear/GaussianBayesNet.cpp
  gtsam/linear/GaussianBayesTree.cpp
  gtsam/linear/GaussianConditional.cpp
  gtsam/linear/GaussianEliminationTree.cpp
  gtsam/linear/GaussianFactor.cpp
  gtsam/linear/GaussianFactorGraph.cpp
  gtsam/linear/GaussianJunctionTree.cpp
  gtsam/linear/HessianFactor.cpp
  gtsam/linear/JacobianFactor.cpp
  gtsam/linear/JointMarginal.cpp
  gtsam/linear/LossFunctions.cpp
  gtsam/linear/NoiseModel.cpp
  gtsam/linear/Sampler.cpp
  gtsam/linear/Scatter.cpp
  gtsam/linear/VectorValues.cpp
  gtsam/linear/linearExceptions.cpp

  gtsam/nonlinear/DoglegOptimizerImpl.cpp
  gtsam/nonlinear/GraphvizFormatting.cpp
  gtsam/nonlinear/ISAM2-impl.cpp
  gtsam/nonlinear/ISAM2.cpp
  gtsam/nonlinear/ISAM2Clique.cpp
  gtsam/nonlinear/ISAM2Params.cpp
  gtsam/nonlinear/LinearContainerFactor.cpp
  gtsam/nonlinear/NonlinearFactor.cpp
  gtsam/nonlinear/NonlinearFactorGraph.cpp
  gtsam/nonlinear/Values.cpp

  gtsam/symbolic/SymbolicBayesNet.cpp
  gtsam/symbolic/SymbolicBayesTree.cpp
  gtsam/symbolic/SymbolicConditional.cpp
  gtsam/symbolic/SymbolicEliminationTree.cpp
  gtsam/symbolic/SymbolicFactor.cpp
  gtsam/symbolic/SymbolicFactorGraph.cpp
  gtsam/symbolic/SymbolicJunctionTree.cpp
)

list(TRANSFORM _gtsam_sources PREPEND "${_gtsam_root}/")

add_library(gtsam STATIC ${_gtsam_sources})
target_compile_features(gtsam PUBLIC cxx_std_17)
set_target_properties(gtsam PROPERTIES
  OUTPUT_NAME gtsam
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(gtsam
  PUBLIC
    "${_gtsam_root}"
    "${CMAKE_CURRENT_BINARY_DIR}"
  PRIVATE
    "${_gtsam_root}/gtsam/3rdparty/cephes"
)

target_include_directories(gtsam SYSTEM
  PUBLIC
    "${_gtsam_root}/gtsam/3rdparty/CCOLAMD/Include"
    "${_gtsam_root}/gtsam/3rdparty/SuiteSparse_config"
)

target_link_libraries(gtsam PUBLIC Eigen3::Eigen)
target_link_libraries(gtsam PUBLIC m)

unset(_gtsam_sources)
unset(_gtsam_root)
