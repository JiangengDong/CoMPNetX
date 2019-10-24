if(CMAKE_BUILD_TYPE MATCHES Debug)
    add_compile_definitions(ATLASMPNET_DEBUG)
endif()

include_directories(./include)

add_library(AtlasMPNet SHARED
        main.cpp
        Problem.cpp
        Parameters.cpp
        Constraint.cpp
        StateValidityChecker.cpp
        TSR.cpp
        TSRRobot.cpp
        TSRChain.cpp
        SemiToroidalStateSpace.cpp
        or_conversions.cpp)
set_target_properties(AtlasMPNet PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(AtlasMPNet
        ${OpenRAVE_LIBRARIES}
        ${Boost_LIBRARIES}
        ${Eigen_LIBRARIES}
        ${OMPL_LIBRARIES})
