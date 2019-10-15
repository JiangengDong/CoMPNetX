//
// Created by jiangeng on 10/12/19.
//
#include "Parameters.h"

#include <openrave/plugin.h>
#include <Eigen/Dense>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>

AtlasMPNet::Parameters::Parameters() : OpenRAVE::PlannerBase::PlannerParameters() {

}

AtlasMPNet::Parameters::~Parameters() = default;

bool AtlasMPNet::Parameters::getStartState(ompl::base::ScopedState<> &start) const {
    const int dof = GetDOF();// TODO: is this the right dof?
    for (int i = 0; i < dof; i++) {
        start[i] = vinitialconfig[i];
    }
    return true;
}

bool AtlasMPNet::Parameters::getGoalState(ompl::base::ScopedState<> &goal) const {
    const int dof = GetDOF();
    for (int i = 0; i < dof; i++) {
        goal[i] = vgoalconfig[i];
    }
    return true;
}
