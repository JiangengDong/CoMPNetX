cmake_minimum_required (VERSION 3.5)
project(CoMPNetX)
set(CMAKE_CXX_STANDARD 14)

# optional in case boost is used
find_package(PythonInterp REQUIRED)
find_package(OpenRAVE 0.9 REQUIRED)
find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem system program_options chrono)
find_package(ompl 1.4.2 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)

add_subdirectory(src bin)
