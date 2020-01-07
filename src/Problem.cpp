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
#include <ompl/base/StateValidityChecker.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <boost/make_shared.hpp>

#include "Problem.h"
#include "Parameters.h"
#include "Constraint.h"
#include "StateValidityChecker.h"
#include "ProjectedStateSpace.h"
#include "AtlasStateSpace.h"
#include "TangentBundleStateSpace.h"
#include "RRTConnect.h"
#include "RRT.h"

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
    return initialized_;
}

OpenRAVE::PlannerStatus AtlasMPNet::Problem::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {
    OpenRAVE::PlannerStatus plannerStatus = OpenRAVE::PS_Failed;
    if (!initialized_) {
                RAVELOG_ERROR("Unable to plan. Did you call InitPlan?\n"); // NOLINT(hicpp-signed-bitwise)
        return plannerStatus;
    }
    simple_setup_->setup();
    ompl::base::PlannerStatus status = simple_setup_->solve(parameters_->solver_parameters_.time_);
    if(parameters_->constraint_parameters_.type_ != ConstraintParameters::PROJECTION)
            RAVELOG_INFO("Atlas charts created: %d", constrained_state_space_->as<ompl::base::AtlasStateSpace>()->getChartCount());
    switch (ompl::base::PlannerStatus::StatusType(status)) {
        case ompl::base::PlannerStatus::UNKNOWN:
                    RAVELOG_WARN("Unknown status!");
            break;
        case ompl::base::PlannerStatus::INVALID_START:
                    RAVELOG_WARN("Invalid start!");
            break;
        case ompl::base::PlannerStatus::INVALID_GOAL:
                    RAVELOG_WARN("Invalid goal!");
            break;
        case ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
                    RAVELOG_WARN("Unrecognized goal type!");
            break;
        case ompl::base::PlannerStatus::TIMEOUT:
                    RAVELOG_WARN("Failed to find a solution!");
            break;
        case ompl::base::PlannerStatus::CRASH:
                    RAVELOG_WARN("The planner crashed!");
            break;
        case ompl::base::PlannerStatus::ABORT:
                    RAVELOG_WARN("The planner did not find a solution for some other reason!");
            break;
        case ompl::base::PlannerStatus::APPROXIMATE_SOLUTION:
                    RAVELOG_WARN("Found an approximate solution. ");
        case ompl::base::PlannerStatus::EXACT_SOLUTION: {
            auto ompl_traj = simple_setup_->getSolutionPath();
            size_t const dof_robot = robot_->GetActiveDOF();
            ptraj->Init(robot_->GetActiveConfigurationSpecification("linear"));
            ompl::base::StateSpacePtr space = ompl_traj.getSpaceInformation()->getStateSpace();
            std::vector<double> values, robot_values;
            for (size_t i = 0; i < ompl_traj.getStateCount(); i++) {
                space->copyToReals(values, ompl_traj.getState(i));
                robot_values.assign(values.begin(), values.begin() + dof_robot);
                ptraj->Insert(i, robot_values, true);
            }

            // print the result
            std::stringstream ss;
            ss << "states in path: " << ompl_traj.getStateCount() << std::endl;
            auto state = ompl_traj.getState(0);
            for (size_t i = 0; i < ompl_traj.getStateCount(); ++i) {
                state = ompl_traj.getState(i);
                space->copyToReals(values, state);
                ss << "\tstate " << i << ":\t";
                for (auto value:values) {
                    ss << " " << value;
                }
                ss << " \tdistance to manifold: " << constraint_->distance(state) << std::endl;
            }
                    RAVELOG_INFO(ss.str());
            plannerStatus = OpenRAVE::PS_HasSolution;
            break;
        }
        default:
            break;
    }
    return plannerStatus;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr AtlasMPNet::Problem::GetParameters() const {
    return parameters_;
}

bool AtlasMPNet::Problem::GetParametersCommand(std::ostream &sout, std::istream &sin) const {
    sout << parameters_->solver_parameters_ << std::endl
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

    // print the result
    if (tsr_robot_ != nullptr) {
        std::stringstream ss;
        ss << "Constructed virtual TSR robot successfully." << std::endl;
        ss << "\tDOF: " << tsr_robot_->GetDOF() << std::endl;
        ss << "\tActive DOF: " << tsr_robot_->GetActiveDOF() << std::endl;
        ss << "\tNum of manipulators: " << tsr_robot_->GetManipulators().size() << std::endl;
        ss << "\tNum of active manipulators: " << (tsr_robot_->GetActiveManipulator() != nullptr);
                RAVELOG_INFO(ss.str());
        return true;
    } else {
                RAVELOG_ERROR("Failed to construct virtual TSR robot!");
        return false;
    }
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
    ambient_state_space_ = std::make_shared<ompl::base::RealVectorStateSpace>(dof + dof_tsr);
    auto ambient_state_space_temp = ambient_state_space_->as<ompl::base::RealVectorStateSpace>();

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
    ambient_state_space_temp->setBounds(bounds);
    bounds = ambient_state_space_temp->getBounds();

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
    double conservative_fraction = conservative_resolution / ambient_state_space_temp->getMaximumExtent();
    ambient_state_space_temp->setLongestValidSegmentFraction(conservative_fraction);
    ambient_state_space_temp->setup();

    // print result
    if (ambient_state_space_ != nullptr) {
        std::stringstream ss;
        ss << "Constructed ambient state space successfully." << std::endl;
        ss << "\tDimension: " << ambient_state_space_->getDimension() << std::endl;
        ss << "\tUpper bound:";
        for (auto hb:bounds.high) {
            ss << " " << hb;
        }
        ss << std::endl;
        ss << "\tLower bound:";
        for (auto lb:bounds.low) {
            ss << " " << lb;
        }
        ss << std::endl;
        ss << "\tResolution: " << conservative_resolution;
                RAVELOG_INFO(ss.str());
        return true;
    } else {
                RAVELOG_ERROR("Failed to construct ambient state space!");
        return false;
    }
}

bool AtlasMPNet::Problem::setConstrainedStateSpace() {
    constraint_ = std::make_shared<TSRChainConstraint>(robot_, tsr_robot_);
    // create the constrained configuration space
    switch (parameters_->constraint_parameters_.type_) {
        case ConstraintParameters::PROJECTION:
            constrained_state_space_ = std::make_shared<ompl::base::ProjectedStateSpace>(ambient_state_space_, constraint_);
            constrained_space_info_ = std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space_);
            break;
        case ConstraintParameters::ATLAS:
            constrained_state_space_ = std::make_shared<ompl::base::AtlasStateSpace>(ambient_state_space_, constraint_);
            constrained_space_info_ = std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space_);
            break;
        case ConstraintParameters::TANGENT_BUNDLE:
            constrained_state_space_ = std::make_shared<ompl::base::TangentBundleStateSpace>(ambient_state_space_, constraint_);
            constrained_space_info_ = std::make_shared<ompl::base::TangentBundleSpaceInformation>(constrained_state_space_);
            break;
    }
    // setup parameters
    constraint_->setTolerance(parameters_->constraint_parameters_.tolerance_);
    constraint_->setMaxIterations(parameters_->constraint_parameters_.max_iter_);
    constrained_state_space_->setDelta(parameters_->constraint_parameters_.delta_);
    constrained_state_space_->setLambda(parameters_->constraint_parameters_.lambda_);
    if (parameters_->constraint_parameters_.type_ == ConstraintParameters::ATLAS ||
        parameters_->constraint_parameters_.type_ == ConstraintParameters::TANGENT_BUNDLE) {
        auto constrained_state_space_temp = constrained_state_space_->as<ompl::base::AtlasStateSpace>();
        constrained_state_space_temp->setExploration(parameters_->atlas_parameters_.exploration_);
        constrained_state_space_temp->setEpsilon(parameters_->atlas_parameters_.epsilon_);
        constrained_state_space_temp->setRho(parameters_->atlas_parameters_.rho_);
        constrained_state_space_temp->setAlpha(parameters_->atlas_parameters_.alpha_);
        constrained_state_space_temp->setMaxChartsPerExtension(parameters_->atlas_parameters_.max_charts_);

        auto &&atlas = constrained_state_space_temp;
        if (parameters_->atlas_parameters_.using_bias_) { // add different weight for sampling to different charts
            constrained_state_space_temp->setBiasFunction([atlas](ompl::base::AtlasChart *c) -> double {
                return 1.0 + atlas->getChartCount() - c->getNeighborCount();
            });
        }
        if (parameters_->constraint_parameters_.type_ == ConstraintParameters::ATLAS) {
            constrained_state_space_temp->setSeparated(parameters_->atlas_parameters_.separate_);
        }
    }
    constrained_state_space_->setup();

    // print result
    if (constrained_state_space_ != nullptr) {
        std::stringstream ss;
        ss << "Constructed constrained state space successfully." << std::endl;
        switch (parameters_->constraint_parameters_.type_) {
            case ConstraintParameters::PROJECTION:
                ss << "\tType: Projection" << std::endl;
                break;
            case ConstraintParameters::ATLAS:
                ss << "\tType: Atlas" << std::endl;
                break;
            case ConstraintParameters::TANGENT_BUNDLE:
                ss << "\tType: Tangent bundle" << std::endl;
                break;
        }
        ss << "\tDimension of manifold: " << constrained_state_space_->getManifoldDimension() << std::endl;
        ss << "\tParameters of constrained state space: " << std::endl;
        ss << "\t\tTolerance: " << constraint_->getTolerance() << std::endl;
        ss << "\t\tMax projection iteration:" << constraint_->getMaxIterations() << std::endl;
        ss << "\t\tDelta (Step-size for discrete geodesic on manifold): " << constrained_state_space_->getDelta() << std::endl;
        ss << "\t\tLambda (Maximum `wandering` allowed during traversal): " << constrained_state_space_->getLambda() << std::endl;
        if (parameters_->constraint_parameters_.type_ == ConstraintParameters::PROJECTION) {
                    RAVELOG_INFO(ss.str());
            return true;
        }
        auto constrained_state_space_temp = constrained_state_space_->as<ompl::base::AtlasStateSpace>();
        ss << "\tParameters of atlas state space: " << std::endl;
        ss << "\t\tExploration (tunes balance of refinement and exploration in atlas sampling): " << constrained_state_space_temp->getExploration()
           << std::endl;
        ss << "\t\tEpsilon (max distance from manifold to chart): " << constrained_state_space_temp->getEpsilon() << std::endl;
        ss << "\t\tRho (max radius for an atlas chart): " << constrained_state_space_temp->getRho() << std::endl;
        ss << "\t\tAlpha (max angle between chart and manifold): " << constrained_state_space_temp->getAlpha() << std::endl;
        ss << "\t\tMax chart generated during a traversal: " << constrained_state_space_temp->getMaxChartsPerExtension();
                RAVELOG_INFO(ss.str());
        return true;
    } else {
                RAVELOG_ERROR("Failed to construct constrained state space!");
        return false;
    }
}

bool AtlasMPNet::Problem::simpleSetup() {
    simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(constrained_space_info_);

    // print result
    if (simple_setup_ != nullptr) {
                RAVELOG_INFO("Constructed simple setup successfully.");
        return true;
    } else {
                RAVELOG_ERROR("Failed to construct simple setup!");
        return false;
    }
}

bool AtlasMPNet::Problem::setStartAndGoalStates() {
    int dof_robot = robot_->GetActiveDOF();
    int dof_tsr = tsr_robot_->GetActiveDOF();

    std::vector<double> robot_start(dof_robot), tsr_start(dof_tsr), robot_goal(dof_robot), tsr_goal(dof_tsr);
    OpenRAVE::Transform Ttemp;

    // start config
    parameters_->getStartState(robot_start);   // get the joint values of real robot
    robot_->SetActiveDOFValues(robot_start);
    tsr_chain_->GetClosestTransform(robot_->GetActiveManipulator()->GetEndEffectorTransform(), tsr_start, Ttemp);    // get the joint values of virtual robot
    // goal config
    parameters_->getGoalState(robot_goal);
    robot_->SetActiveDOFValues(robot_goal);
    tsr_chain_->GetClosestTransform(robot_->GetActiveManipulator()->GetEndEffectorTransform(), tsr_goal, Ttemp);

    // convert to ScopedState and set start and goal with simple_setup_
    ompl::base::ScopedState<> start(constrained_state_space_);
    ompl::base::ScopedState<> goal(constrained_state_space_);
    for (int i = 0; i < dof_robot; i++) {
        start[i] = robot_start[i];
        goal[i] = robot_goal[i];
    }
    for (int i = 0; i < dof_tsr; i++) {
        start[i + dof_robot] = tsr_start[i];
        goal[i + dof_robot] = tsr_goal[i];
    }
    if (parameters_->constraint_parameters_.type_ != ConstraintParameters::PROJECTION){
        auto constrained_state_space_temp = constrained_state_space_->as<ompl::base::AtlasStateSpace>();
        constrained_state_space_temp->anchorChart(start.get());
        constrained_state_space_temp->anchorChart(goal.get());
    }
    simple_setup_->setStartAndGoalStates(start, goal);

    // print result
    std::stringstream ss;
    ss << "Set start and goal successfully." << std::endl;
    ss << "\tStart:" << std::endl;
    ss << "\t\tRobot:";
    for (int i = 0; i < dof_robot; i++) {
        ss << "\t" << start[i];
    }
    ss << std::endl;
    ss << "\t\tTSR:";
    for (int i = dof_robot; i < dof_robot + dof_tsr; i++) {
        ss << "\t" << start[i];
    }
    ss << std::endl;
    ss << "\t\tDistance: " << constraint_->distance(start.get()) << std::endl;
    ss << "\tGoal:" << std::endl;
    ss << "\t\tRobot:";
    for (int i = 0; i < dof_robot; i++) {
        ss << "\t" << goal[i];
    }
    ss << std::endl;
    ss << "\t\tTSR:";
    for (int i = dof_robot; i < dof_robot + dof_tsr; i++) {
        ss << "\t" << goal[i];
    }
    ss << std::endl;
    ss << "\t\tDistance: " << constraint_->distance(goal.get());
            RAVELOG_INFO(ss.str());
    return true;
}

bool AtlasMPNet::Problem::setStateValidityChecker() {
    state_validity_checker_= std::make_shared<AtlasMPNet::StateValidityChecker>(constrained_space_info_, robot_, tsr_robot_, tsr_chain_);
//    state_validity_checker_= std::make_shared<ompl::base::AllValidStateValidityChecker>(constrained_space_info_);
    simple_setup_->setStateValidityChecker(state_validity_checker_);

    // print result
    if (state_validity_checker_ != nullptr) {
                RAVELOG_INFO("Constructed state validity checker successfully.");
        return true;
    } else {
                RAVELOG_ERROR("Failed to construct state validity checker!");
        return false;
    }
}

bool AtlasMPNet::Problem::setPlanner() {
    switch (parameters_->solver_parameters_.type_) {
        case SolverParameters::RRT:
            planner_ = std::make_shared<ompl::geometric::RRT>(constrained_space_info_);
            planner_->as<ompl::geometric::RRT>()->setRange(parameters_->solver_parameters_.range_);
            break;
        case SolverParameters::RRTStar:
            planner_ = std::make_shared<ompl::geometric::RRTstar>(constrained_space_info_);
            planner_->as<ompl::geometric::RRTstar>()->setRange(parameters_->solver_parameters_.range_);
            break;
        case SolverParameters::RRTConnect:
            planner_ = std::make_shared<ompl::geometric::RRTConnect>(constrained_space_info_);
            planner_->as<ompl::geometric::RRTConnect>()->setRange(parameters_->solver_parameters_.range_);
            break;
        case SolverParameters::MPNet:
            break;
    }
    simple_setup_->setPlanner(planner_);

    // print result
    if (planner_ != nullptr) {
        std::stringstream ss;
        ss << "Constructed planner successfully." << std::endl;
        ss << "\tPlanner: " << planner_->getName() << std::endl;
        switch (parameters_->solver_parameters_.type_) {
            case SolverParameters::RRT:
                ss << "\t\tRange: " << planner_->as<ompl::geometric::RRT>()->getRange() << std::endl;
                break;
            case SolverParameters::RRTStar:
                ss << "\t\tRange: " << planner_->as<ompl::geometric::RRTstar>()->getRange() << std::endl;
                break;
            case SolverParameters::RRTConnect:
                ss << "\t\tRange: " << planner_->as<ompl::geometric::RRTConnect>()->getRange() << std::endl;
                break;
            case SolverParameters::MPNet:
                break;
        }
                RAVELOG_INFO(ss.str());
        return true;
    } else {
                RAVELOG_ERROR("Failed to construct planner!");
        return false;
    }
}
