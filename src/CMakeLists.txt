include_directories(./include ./planner/include)

add_library(AtlasMPNet SHARED
        main.cpp
        Problem.cpp
        Parameters.cpp
        Constraint.cpp
        StateValidityChecker.cpp
        TaskSpaceRegionChain.cpp
        planner/RRTConnect.cpp
        planner/MPNetSampler.cpp
        planner/MPNetPlanner.cpp
        planner/RobotHelper.cpp
        planner/PRM.cpp
        planner/BiEST.cpp
        planner/EST.cpp
        planner/PRMMPNet.cpp
        )
set_target_properties(AtlasMPNet PROPERTIES
        COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}"
        LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
        LIBRARY_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}/plugins")
target_link_libraries(AtlasMPNet
        ${OpenRAVE_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OMPL_LIBRARIES}
        ${TORCH_LIBRARIES})
