include_directories(./include)

add_library(AtlasMPNet SHARED
        main.cpp
        Problem.cpp
        Parameters.cpp
        Constraint.cpp
        StateValidityChecker.cpp
        TaskSpaceRegionChain.cpp
        RRTConnect.cpp
        MPNetSamplerX.cpp
        MPNetPlanner.cpp
        RobotHelper.cpp
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
