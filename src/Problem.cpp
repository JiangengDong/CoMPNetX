//
// Created by jiangeng on 10/11/19.
//
#include <openrave/plugin.h>
#include <utility>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "Problem.h"
#include "Parameters.h"
#include "TSRChainConstraint.h"
#include "StateValidityChecker.h"

AtlasMPNet::Problem::Problem(OpenRAVE::EnvironmentBasePtr penv, std::istream &ss) : OpenRAVE::PlannerBase(
        std::move(penv)) {
}

AtlasMPNet::Problem::~Problem() = default;

bool AtlasMPNet::Problem::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params) {
    robot_ = std::move(robot);
    parameters_ = std::move(params);
    return setAmbientStateSpace()
        && setConstrainedStateSpace()
        && simpleSetup()
        && setStartAndGoalStates();
//        && setStateValidityChecker();
}

OpenRAVE::PlannerStatus AtlasMPNet::Problem::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {
    OpenRAVE::PlannerStatus plannerStatus = OpenRAVE::PS_Failed;
    planner_ = std::make_shared<ompl::geometric::RRTstar>(constrained_space_info_);
    // TODO: set range of planner
    simple_setup_->setup();
    // TODO: add solving time parameters
    ompl::base::PlannerStatus status = simple_setup_->solve(1);
    if(status) {
        plannerStatus = OpenRAVE::PS_HasSolution;
    }
    return plannerStatus;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr AtlasMPNet::Problem::GetParameters() const {
    return parameters_;
}

bool AtlasMPNet::Problem::setAmbientStateSpace() {
    // TODO: create ambient configuration space and add bounds w.r.t robot
    ambient_state_space_ = std::make_shared<ompl::base::RealVectorStateSpace>(3);
    return false;
}

bool AtlasMPNet::Problem::setConstrainedStateSpace() {
    // create the constraint according to TSR
    // TODO: implement TSRChainConstraint
    constraint_ = std::make_shared<AtlasMPNet::TSRChainConstraint>(1, 0);
    // create the constrained configuration space
    constrained_state_space_ = std::make_shared<ompl::base::AtlasStateSpace>(ambient_state_space_, constraint_);
    constrained_space_info_ = std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space_);
    constrained_state_space_->setup();
    // setup parameters
    constraint_->setTolerance(parameters_->tolerance);
    constraint_->setMaxIterations(parameters_->max_iter);
    constrained_state_space_->setDelta(parameters_->delta);
    constrained_state_space_->setLambda(parameters_->lambda);
    constrained_state_space_->setExploration(parameters_->exploration);
    constrained_state_space_->setEpsilon(parameters_->epsilon);
    constrained_state_space_->setRho(parameters_->rho);
    constrained_state_space_->setAlpha(parameters_->alpha);
    constrained_state_space_->setMaxChartsPerExtension(parameters_->max_charts);
    constrained_state_space_->setSeparated(parameters_->separate);
    // TODO: atlas->setBiasFunction?
    return true;
}

bool AtlasMPNet::Problem::simpleSetup() {
    simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(constrained_space_info_);

    return false;
}

bool AtlasMPNet::Problem::setStartAndGoalStates() {
    ompl::base::ScopedState<> start(constrained_state_space_);
    ompl::base::ScopedState<> goal(constrained_state_space_);
    parameters_->getStartState(start);
    parameters_->getGoalState(goal);
    constrained_state_space_->anchorChart(start.get());
    constrained_state_space_->anchorChart(goal.get());
    simple_setup_->setStartAndGoalStates(start, goal);
    return true;
}

// TODO: create StateValidityChecker Class
bool AtlasMPNet::Problem::setStateValidityChecker(const AtlasMPNet::StateValidityCheckerPtr &svc) {
    simple_setup_->setStateValidityChecker(svc);
    return true;
}
