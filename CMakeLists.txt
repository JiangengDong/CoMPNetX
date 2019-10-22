cmake_minimum_required (VERSION 2.6.0)
project(AtlasMPNet)
set (CMAKE_CXX_STANDARD 11)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(OpenRAVE 0.9 REQUIRED)
#find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)
find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem system program_options chrono)
find_package(ompl REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS} include)
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

add_subdirectory(src bin)

MESSAGE(STATUS "${Boost_LIBRARIES}")