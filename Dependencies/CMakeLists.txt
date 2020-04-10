add_subdirectory(BoostHeaders)
add_subdirectory(cereal EXCLUDE_FROM_ALL)
add_subdirectory(GSL)
add_subdirectory(Arcana)

add_library(Eigen INTERFACE)
target_include_directories(Eigen INTERFACE "eigen-git-mirror/")

# Prevents the inclusion of any non-permissively licensed code from Eigen.
target_compile_definitions(Eigen INTERFACE EIGEN_MPL2_ONLY)
