cmake_minimum_required (VERSION 3.5)
project(AtlasMPNet)
set(CMAKE_CXX_STANDARD 14)
add_compile_options(-Wno-deprecated-declarations)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(PythonInterp REQUIRED)
find_package(OpenRAVE 0.9 REQUIRED)
find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem system program_options chrono)
find_package(ompl 1.4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${TORCH_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

add_subdirectory(src bin)