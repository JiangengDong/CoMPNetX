add_library(CoMPNetX SHARED
    compnetx/src/lib.cpp
    compnetx/src/Problem.cpp
    compnetx/src/Parameters.cpp
    compnetx/src/Constraint.cpp
    compnetx/src/StateValidityChecker.cpp
    compnetx/src/TaskSpaceRegionChain.cpp
    compnetx/planner/src/RRTConnect.cpp
    compnetx/planner/src/MPNetWithTSRSampler.cpp
    compnetx/planner/src/MPNetWithoutTSRSampler.cpp
    compnetx/planner/src/MPNetXPlanner.cpp
    compnetx/planner/src/RobotHelper.cpp
)
set_target_properties(CoMPNetX PROPERTIES
    COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}"
    LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
    LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/plugins")
target_compile_options(CoMPNetX PUBLIC
    -Wall
    -Wno-deprecated-declarations)
target_include_directories(CoMPNetX PUBLIC
    ./compnetx
    ${OpenRAVE_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${TORCH_INCLUDE_DIRS}
)
target_link_libraries(CoMPNetX
    ${OpenRAVE_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    ${TORCH_LIBRARIES}
    HighFive)
