//
// Created by jiangeng on 10/11/19.
//
#include <openrave/openrave.h>
#include <utility>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <boost/make_shared.hpp>

#include "Problem.h"
#include "Parameters.h"
#include "Constraint.h"
#include "StateValidityChecker.h"

AtlasMPNet::Problem::Problem(OpenRAVE::EnvironmentBasePtr penv, std::istream &ss) : OpenRAVE::PlannerBase(
        std::move(penv)) {
}

AtlasMPNet::Problem::~Problem() = default;

bool AtlasMPNet::Problem::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream &input) {
    AtlasMPNet::Parameters::Ptr params = boost::make_shared<AtlasMPNet::Parameters>();
    input >> *params;
    return InitPlan(robot, params);
}

bool AtlasMPNet::Problem::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params) {
    RAVELOG_INFO("Start to init planner.");
    initialized_ = false;
    if (robot == nullptr || params == nullptr) {
                RAVELOG_ERROR("Robot and params must not be NULL.\n"); // NOLINT(hicpp-signed-bitwise)
        return initialized_;
    }
    robot_ = std::move(robot);
    parameters_ = boost::make_shared<AtlasMPNet::Parameters>();
    std::cout << *params;
    parameters_->copy(params);
    std::cout << *parameters_;
    // TODO: check the serialize and deserialize process
    setAmbientStateSpace();
    setConstrainedStateSpace();
    simpleSetup();
    setStartAndGoalStates();
    setStateValidityChecker();
    setPlanner();
    initialized_ = true;
    return initialized_;
}

OpenRAVE::PlannerStatus AtlasMPNet::Problem::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {
    OpenRAVE::PlannerStatus plannerStatus = OpenRAVE::PS_Failed;
    if (!initialized_) {
                RAVELOG_ERROR("Unable to plan. Did you call InitPlan?\n"); // NOLINT(hicpp-signed-bitwise)
        return plannerStatus;
    }
    simple_setup_->setup();
    ompl::base::PlannerStatus status = simple_setup_->solve(parameters_->time);
    if (status) {
        plannerStatus = OpenRAVE::PS_HasSolution;
    }
    return plannerStatus;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr AtlasMPNet::Problem::GetParameters() const {
    return parameters_;
}

bool AtlasMPNet::Problem::setAmbientStateSpace() {
    const int dof = robot_->GetActiveDOF();
    // Set bounds
    std::vector<OpenRAVE::dReal> lower_limits, upper_limits;
    ompl::base::RealVectorBounds bounds(dof);
    robot_ -> GetActiveDOFLimits(lower_limits, upper_limits);
    for (size_t i = 0; i < dof; ++i) {
        bounds.setLow(i, lower_limits[i]);
        bounds.setHigh(i, upper_limits[i]);
    }
    ambient_state_space_ = std::make_shared<ompl::base::RealVectorStateSpace>(dof);
    ambient_state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    // Set resolution
    std::vector<OpenRAVE::dReal> dof_resolutions;
    robot_->GetActiveDOFResolutions(dof_resolutions);

    double conservative_resolution = std::numeric_limits<double>::max();
    for (size_t i = 0; i < dof; ++i) {
        conservative_resolution = std::min(conservative_resolution, dof_resolutions[i]);
    }

    double conservative_fraction = conservative_resolution / ambient_state_space_->getMaximumExtent();
    ambient_state_space_->setLongestValidSegmentFraction(conservative_fraction);
    RAVELOG_INFO("Set ambient configuration space.");
    return true;
}

bool AtlasMPNet::Problem::setConstrainedStateSpace() {
    constraint_ = std::make_shared<AtlasMPNet::SphereConstraint>();
    // create the constrained configuration space
    if (parameters_->using_tb_) {
        constrained_state_space_ = std::make_shared<ompl::base::TangentBundleStateSpace>(ambient_state_space_, constraint_);
        constrained_space_info_ = std::make_shared<ompl::base::TangentBundleSpaceInformation>(constrained_state_space_);
    } else {
        constrained_state_space_ = std::make_shared<ompl::base::AtlasStateSpace>(ambient_state_space_, constraint_);
        constrained_space_info_ = std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space_);
    }
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

    auto &&atlas = constrained_state_space_;
    if (parameters_->using_bias_) { // add different weight for sampling to different charts
        constrained_state_space_->setBiasFunction([atlas](ompl::base::AtlasChart *c) -> double {
            return 1.0 + atlas->getChartCount() - c->getNeighborCount();
        });
    }
    if (!parameters_->using_tb_)
        constrained_state_space_->setSeparated(parameters_->separate);
    constrained_state_space_->setup();
    RAVELOG_INFO("Set constrained configuration space.");
    return true;
}

bool AtlasMPNet::Problem::simpleSetup() {
    simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(constrained_space_info_);
    RAVELOG_INFO("Create simple setup.");
    return true;
}

bool AtlasMPNet::Problem::setStartAndGoalStates() {
    // TODO: fix the get state and goal function in parameter class
    ompl::base::ScopedState<> start(constrained_state_space_);
    ompl::base::ScopedState<> goal(constrained_state_space_);
    parameters_->getStartState(start);
    parameters_->getGoalState(goal);
    // TODO: Sphere constraint need to adjusted
//    constrained_state_space_->anchorChart(start.get());
//    constrained_state_space_->anchorChart(goal.get());
//    simple_setup_->setStartAndGoalStates(start, goal);
    RAVELOG_INFO("Set start and goal configurations.");
    return true;
}

bool AtlasMPNet::Problem::setStateValidityChecker() {
    std::vector<int> dof_indices = robot_->GetActiveDOFIndices();
    state_validity_checker_.reset(new AtlasMPNet::StateValidityChecker(constrained_space_info_, robot_, dof_indices));
    simple_setup_->setStateValidityChecker(state_validity_checker_);
    RAVELOG_INFO("Set validity checker.");
    return true;
}

bool AtlasMPNet::Problem::setPlanner() {
    planner_ = std::make_shared<ompl::geometric::RRTstar>(constrained_space_info_);
    if (parameters_->range == 0)
        planner_->as<ompl::geometric::RRTstar>()->setRange(constrained_state_space_->getRho_s());
    else
        planner_->as<ompl::geometric::RRTstar>()->setRange(parameters_->range);
    simple_setup_->setPlanner(planner_);
    RAVELOG_INFO("Set planner.");
    return true;
}
