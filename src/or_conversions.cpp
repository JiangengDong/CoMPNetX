//
// Created by jiangeng on 10/22/19.
//
#include "or_conversions.h"

#include <ompl/geometric/PathGeometric.h>

using namespace AtlasMPNet;

OpenRAVE::PlannerStatus ToORTrajectory(
        OpenRAVE::RobotBasePtr const &robot,
        ompl::geometric::PathGeometric &ompl_traj,
        OpenRAVE::TrajectoryBasePtr or_traj) {
    using ompl::geometric::PathGeometric;

    size_t const num_dof = robot->GetActiveDOF();
    or_traj->Init(robot->GetActiveConfigurationSpecification("linear"));

    ompl::base::StateSpacePtr space = ompl_traj.getSpaceInformation()->getStateSpace();

    for (size_t i = 0; i < ompl_traj.getStateCount(); ++i) {
        std::vector<double> values;
        space->copyToReals(values, ompl_traj.getState(i));
        or_traj->Insert(i, values, true);
    }
    return OpenRAVE::PS_HasSolution;
}
