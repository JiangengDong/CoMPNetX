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
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <boost/make_shared.hpp>

#include "Problem.h"
#include "Parameters.h"
#include "Constraint.h"
#include "StateValidityChecker.h"
#include "SemiToroidalStateSpace.h"
#include "or_conversions.h"
#include "AtlasStateSpace.h"
#include "TangentBundleStateSpace.h"

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
    initialized_ = setTSRChainRobot() &&
                   setAmbientStateSpace() &&
                   setConstrainedStateSpace() &&
                   simpleSetup() &&
                   setStartAndGoalStates() &&
                   setStateValidityChecker() &&
                   setPlanner();
    // if success, we should print the problem infos out
    if (initialized_) {
        std::stringstream ss;
        ss << "The problem to be solved is defined as follows:" << std::endl;
        ss << "Ambient configuration space dim: " << constrained_state_space_->getAmbientDimension() << std::endl
           << "Constrained manifold dim: " << constrained_state_space_->getManifoldDimension() << std::endl
           << "Planner: " << planner_->getName() << std::endl;

        unsigned int am_dim = constrained_state_space_->getAmbientDimension();
        auto config = new double[am_dim];
        ss << "Start:";
        parameters_->getStartState(config);
        for (unsigned int i = 0; i < am_dim; ++i) {
            ss << " " << config[i];
        }
        ss << std::endl;
        ss << "Goal: ";
        parameters_->getGoalState(config);
        for (unsigned int i = 0; i < am_dim; ++i) {
            ss << " " << config[i];
        }
        ss << std::endl;
                RAVELOG_INFO(ss.str());
        delete[] config;
    }
    return initialized_;
}

OpenRAVE::PlannerStatus AtlasMPNet::Problem::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {
    OpenRAVE::PlannerStatus plannerStatus = OpenRAVE::PS_Failed;
    if (!initialized_) {
                RAVELOG_ERROR("Unable to plan. Did you call InitPlan?\n"); // NOLINT(hicpp-signed-bitwise)
        return plannerStatus;
    }
    simple_setup_->setup();
    boost::chrono::steady_clock::time_point const tic = boost::chrono::steady_clock::now();
    ompl::base::PlannerStatus status = simple_setup_->solve(/*parameters_->planner_parameters_.time_*/);
    boost::chrono::steady_clock::time_point const toc = boost::chrono::steady_clock::now();
            RAVELOG_INFO("Find a solution after %f s.", boost::chrono::duration_cast<boost::chrono::duration<double> >(toc - tic).count());
            RAVELOG_INFO("Atlas charts: %d", constrained_state_space_->getChartCount());
//            planner_->printSettings(std::cout);
    if (status) {
        auto ompl_traj = simple_setup_->getSolutionPath();
        size_t const num_dof = robot_->GetActiveDOF();
        ptraj->Init(robot_->GetActiveConfigurationSpecification("linear"));
        ompl::base::StateSpacePtr space = ompl_traj.getSpaceInformation()->getStateSpace();
        std::stringstream ss;
        ss << "states in path: " << ompl_traj.getStateCount() << std::endl;
        for (size_t i = 0; i < ompl_traj.getStateCount(); ++i) {
            std::vector<double> values;
            space->copyToReals(values, ompl_traj.getState(i));
            ptraj->Insert(i, values, true);
            ss << "state " << i << ": ";
            for (auto value:values) {
                ss << value << " ";
            }
            ss << " \tdistance to manifold: " << constraint_->distance(ompl_traj.getState(i)) << std::endl;
        }
                RAVELOG_INFO(ss.str());
        plannerStatus = OpenRAVE::PS_HasSolution;
        // TODO: detailed behavior w.r.t planner status
    }
    return plannerStatus;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr AtlasMPNet::Problem::GetParameters() const {
    return parameters_;
}

bool AtlasMPNet::Problem::GetParametersCommand(std::ostream &sout, std::istream &sin) const {
    sout << parameters_->planner_parameters_ << std::endl
         << parameters_->constraint_parameters_ << std::endl
         << parameters_->atlas_parameters_ << std::endl
         << parameters_->tsrchain_parameters_ << std::endl;
    return true;
}

bool AtlasMPNet::Problem::setTSRChainRobot() {
    env_ = robot_->GetEnv();
    tsr_chain_ = std::make_shared<TaskSpaceRegionChain>();
    *tsr_chain_ = parameters_->tsrchain_parameters_;
    tsr_chain_->Initialize(env_);
    tsr_chain_->RobotizeTSRChain(env_, tsr_robot_);
    return tsr_robot_ != nullptr;
}

/** \brief Setup the ambient state space
 *
 * The config is composed of two parts.
 * The first one is the joint values of real robot,
 * and the second one is the joint values of virtual TSRChain robot.
 *
 * @return true if setup successfully
 */
bool AtlasMPNet::Problem::setAmbientStateSpace() {
    const int dof = robot_->GetActiveDOF();
    const int dof_tsr = tsr_robot_->GetActiveDOF();
    // Set bounds
    ompl::base::RealVectorBounds bounds(dof + dof_tsr);
    std::vector<OpenRAVE::dReal> lower_limits, upper_limits;
    // get bounds for real robot
    robot_->GetActiveDOFLimits(lower_limits, upper_limits);
    for (size_t i = 0; i < dof; ++i) {
        bounds.setLow(i, lower_limits[i]);
        bounds.setHigh(i, upper_limits[i]);
    }
    // get bounds for virtual robot
    tsr_robot_->GetActiveDOFLimits(lower_limits, upper_limits);
    for (size_t i = 0; i < dof_tsr; ++i) {
        bounds.setLow(i + dof, lower_limits[i]);
        bounds.setHigh(i + dof, upper_limits[i]);
    }
    ambient_state_space_ = std::make_shared<SemiToroidalStateSpace>(dof + dof_tsr);
    ambient_state_space_->setBounds(bounds);

    // Set resolution
    std::vector<OpenRAVE::dReal> dof_resolutions;
    double conservative_resolution = std::numeric_limits<double>::max();
    robot_->GetActiveDOFResolutions(dof_resolutions);
    for (const auto &dof_resolution: dof_resolutions) {
        conservative_resolution = std::min(conservative_resolution, dof_resolution);
    }
    tsr_robot_->GetActiveDOFResolutions(dof_resolutions);
    for (const auto &dof_resolution: dof_resolutions) {
        conservative_resolution = std::min(conservative_resolution, dof_resolution);
    }
    double conservative_fraction = conservative_resolution / ambient_state_space_->getMaximumExtent();
    ambient_state_space_->setLongestValidSegmentFraction(conservative_fraction);
            RAVELOG_DEBUG("Set ambient configuration space.");

    // TODO: set wrappings
    return true;
}

bool AtlasMPNet::Problem::setConstrainedStateSpace() {
    constraint_ = std::make_shared<TSRChainConstraint>(robot_, tsr_robot_);
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
            RAVELOG_DEBUG("Set constrained configuration space.");
    return true;
}

bool AtlasMPNet::Problem::simpleSetup() {
    simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(constrained_space_info_);
            RAVELOG_DEBUG("Create simple setup.");
    return true;
}

bool AtlasMPNet::Problem::setStartAndGoalStates() {
    int dof_robot = robot_->GetActiveDOF();
    int dof_tsr = tsr_robot_->GetActiveDOF();
    auto start_config = new double[dof_robot + dof_tsr];
    auto goal_config = new double[dof_robot + dof_tsr];
    // get the joint values of real robot
    parameters_->getStartState(start_config);
    parameters_->getGoalState(goal_config);
    // get the joint values of virtual robot
    OpenRAVE::Transform Ttemp;
    robot_->SetActiveDOFValues(std::vector<double>(start_config, start_config + dof_robot)); // TODO: is this the right grammar?
    tsr_chain_->GetClosestTransform(robot_->GetActiveManipulator()->GetEndEffectorTransform(), start_config + dof_robot, Ttemp);
    robot_->SetActiveDOFValues(std::vector<double>(goal_config, goal_config + dof_robot)); // TODO: is this the right grammar?
    tsr_chain_->GetClosestTransform(robot_->GetActiveManipulator()->GetEndEffectorTransform(), goal_config + dof_robot, Ttemp);
    // convert to ScopedState and set start and goal with simple_setup_
    ompl::base::ScopedState<> start(constrained_state_space_);
    ompl::base::ScopedState<> goal(constrained_state_space_);
    for (int i = 0; i < dof_robot + dof_tsr; i++) {
        start[i] = start_config[i];
        goal[i] = goal_config[i];
    }
    constrained_state_space_->anchorChart(start.get());
    constrained_state_space_->anchorChart(goal.get());
    simple_setup_->setStartAndGoalStates(start, goal);
            RAVELOG_DEBUG("Set start and goal configurations.");
    delete[] start_config;
    delete[] goal_config;
    return true;
}

bool AtlasMPNet::Problem::setStateValidityChecker() {   // TODO: need to check mimic body and links
    std::vector<int> dof_indices = robot_->GetActiveDOFIndices();
    state_validity_checker_.reset(new AtlasMPNet::StateValidityChecker(constrained_space_info_, robot_, tsr_robot_));
    simple_setup_->setStateValidityChecker(state_validity_checker_);
            RAVELOG_DEBUG("Set validity checker.");
    return true;
}

bool AtlasMPNet::Problem::setPlanner() {
    planner_ = std::make_shared<ompl::geometric::RRT>(constrained_space_info_);
    if (parameters_->planner_parameters_.range_ == 0)
        planner_->as<ompl::geometric::RRT>()->setRange(constrained_state_space_->getRho_s());
    else
        planner_->as<ompl::geometric::RRT>()->setRange(parameters_->planner_parameters_.range_);
    simple_setup_->setPlanner(planner_);
            RAVELOG_DEBUG("Set planner.");
    return true;
}
