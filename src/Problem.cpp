/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>
         Matthew Klingensmith <mklingen@cs.cmu.edu>
         Christopher Dellin <cdellin@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/
#include <openrave/openrave.h>
#include <utility>

#include <ompl/base/spaces/RealVectorStateSpace.h>
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
#include "SemiToroidalStateSpace.h"

AtlasMPNet::Problem::Problem(OpenRAVE::EnvironmentBasePtr penv, std::istream &ss) :
        OpenRAVE::PlannerBase(std::move(penv)),
        parameters_(boost::make_shared<AtlasMPNet::Parameters>()) {
    RegisterCommand("GetParameters",
                    boost::bind(&AtlasMPNet::Problem::GetParametersCommand, this, _1, _2),
                    "returns the values of all the parameters");
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
    parameters_->copy(params);
    initialized_ = setAmbientStateSpace() &&
                   setConstrainedStateSpace() &&
                   simpleSetup() &&
                   setStartAndGoalStates() &&
                   setStateValidityChecker() &&
                   setPlanner();
    return initialized_;
}

OpenRAVE::PlannerStatus AtlasMPNet::Problem::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {
            RAVELOG_DEBUG("Enter PlanPath.");
    OpenRAVE::PlannerStatus plannerStatus = OpenRAVE::PS_Failed;
    if (!initialized_) {
                RAVELOG_ERROR("Unable to plan. Did you call InitPlan?\n"); // NOLINT(hicpp-signed-bitwise)
        return plannerStatus;
    }
            RAVELOG_DEBUG("Start to set SimpleSetup.");
    simple_setup_->setup();
            RAVELOG_DEBUG("Start to plan path.");
    ompl::base::PlannerStatus status = simple_setup_->solve(parameters_->planner_parameters_.time_);
            RAVELOG_DEBUG("Finish path planning.");
    if (status) {
        plannerStatus = OpenRAVE::PS_HasSolution;
        // TODO: copy the result path to ptraj
    }
    return plannerStatus;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr AtlasMPNet::Problem::GetParameters() const {
    return parameters_;
}

bool AtlasMPNet::Problem::GetParametersCommand(std::ostream &sout, std::istream &sin) const {
    sout << parameters_->planner_parameters_ << std::endl
         << parameters_->constraint_parameters_ << std::endl
         << parameters_->atlas_parameters_ << std::endl;
    return true;
}

bool AtlasMPNet::Problem::setAmbientStateSpace() {
    const int dof = robot_->GetActiveDOF();
    // Set bounds
    std::vector<OpenRAVE::dReal> lower_limits, upper_limits;
    ompl::base::RealVectorBounds bounds(dof);
    robot_->GetActiveDOFLimits(lower_limits, upper_limits);
    for (size_t i = 0; i < dof; ++i) {
        bounds.setLow(i, lower_limits[i]);
        bounds.setHigh(i, upper_limits[i]);
    }
    ambient_state_space_ = std::make_shared<SemiToroidalStateSpace>(dof);
    ambient_state_space_->setBounds(bounds);
    // Set resolution
    std::vector<OpenRAVE::dReal> dof_resolutions;
    robot_->GetActiveDOFResolutions(dof_resolutions);

    double conservative_resolution = std::numeric_limits<double>::max();
    for (const auto &dof_resolution: dof_resolutions) {
        conservative_resolution = std::min(conservative_resolution, dof_resolution);
    }

    double conservative_fraction = conservative_resolution / ambient_state_space_->getMaximumExtent();
    ambient_state_space_->setLongestValidSegmentFraction(conservative_fraction);
            RAVELOG_INFO("Set ambient configuration space.");
    return true;
}

bool AtlasMPNet::Problem::setConstrainedStateSpace() {
    constraint_ = std::make_shared<AtlasMPNet::SphereConstraint>(robot_->GetActiveDOF());
    // create the constrained configuration space
    if (parameters_->atlas_parameters_.using_tb_) {
        constrained_state_space_ = std::make_shared<ompl::base::TangentBundleStateSpace>(ambient_state_space_, constraint_);
        constrained_space_info_ = std::make_shared<ompl::base::TangentBundleSpaceInformation>(constrained_state_space_);
    } else {
        constrained_state_space_ = std::make_shared<ompl::base::AtlasStateSpace>(ambient_state_space_, constraint_);
        constrained_space_info_ = std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space_);
    }
    // setup parameters
    constraint_->setTolerance(parameters_->constraint_parameters_.tolerance_);
    constraint_->setMaxIterations(parameters_->constraint_parameters_.max_iter_);
    constrained_state_space_->setDelta(parameters_->constraint_parameters_.delta_);
    constrained_state_space_->setLambda(parameters_->constraint_parameters_.lambda_);
    constrained_state_space_->setExploration(parameters_->atlas_parameters_.exploration_);
    constrained_state_space_->setEpsilon(parameters_->atlas_parameters_.epsilon_);
    constrained_state_space_->setRho(parameters_->atlas_parameters_.rho_);
    constrained_state_space_->setAlpha(parameters_->atlas_parameters_.alpha_);
    constrained_state_space_->setMaxChartsPerExtension(parameters_->atlas_parameters_.max_charts_);

    auto &&atlas = constrained_state_space_;
    if (parameters_->atlas_parameters_.using_bias_) { // add different weight for sampling to different charts
        constrained_state_space_->setBiasFunction([atlas](ompl::base::AtlasChart *c) -> double {
            return 1.0 + atlas->getChartCount() - c->getNeighborCount();
        });
    }
    if (!parameters_->atlas_parameters_.using_tb_)
        constrained_state_space_->setSeparated(parameters_->atlas_parameters_.separate_);
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
    ompl::base::ScopedState<> start(constrained_state_space_);
    ompl::base::ScopedState<> goal(constrained_state_space_);
    parameters_->getStartState(start);
    parameters_->getGoalState(goal);
    constrained_state_space_->anchorChart(start.get());
    constrained_state_space_->anchorChart(goal.get());
    simple_setup_->setStartAndGoalStates(start, goal);
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
    if (parameters_->planner_parameters_.range_ == 0)
        planner_->as<ompl::geometric::RRTstar>()->setRange(constrained_state_space_->getRho_s());
    else
        planner_->as<ompl::geometric::RRTstar>()->setRange(parameters_->planner_parameters_.range_);
    simple_setup_->setPlanner(planner_);
            RAVELOG_INFO("Set planner.");
    return true;
}
