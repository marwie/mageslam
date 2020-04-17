add_subdirectory(BoostHeaders)
add_subdirectory(cereal EXCLUDE_FROM_ALL)
add_subdirectory(GSL)
add_subdirectory(Arcana)
add_subdirectory(Tracing)
add_subdirectory(Platform)

add_library(Eigen INTERFACE)
target_include_directories(Eigen INTERFACE "eigen-git-mirror/")

# Prevents the inclusion of any non-permissively licensed code from Eigen.
target_compile_definitions(Eigen INTERFACE EIGEN_MPL2_ONLY)

option(G2O_NO_IMPLICIT_OWNERSHIP_OF_OBJECTS "Disables memory management in the graph types, this requires the callers to manager the memory of edges and nodes" ON)
option(BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the g2o plugin system)" OFF)
set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen-git-mirror" CACHE STRING "The location of the Eigen library.")
add_subdirectory(g2o EXCLUDE_FROM_ALL)
add_library(g2o INTERFACE)
target_link_libraries(g2o INTERFACE
    core
    solver_dense
    types_sba)
target_include_directories(g2o INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/g2o" "${CMAKE_CURRENT_BINARY_DIR}/g2o")

add_subdirectory(BundlerLib)
add_subdirectory(FuserLib)
