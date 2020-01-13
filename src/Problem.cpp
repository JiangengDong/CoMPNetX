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
        OpenRAVE::PlannerBase(std::move(penv)) {
    RegisterCommand("GetParameters",
                    boost::bind(&AtlasMPNet::Problem::GetParametersCommand, this, _1, _2),
                    "returns the values of all the parameters");
    RegisterCommand("GetPlanningTime",
                    boost::bind(&AtlasMPNet::Problem::GetPlanningTimeCommand, this, _1, _2),
                    "returns the amount of time (in seconds) spent during the last planning step");
    RegisterCommand("SetLogLevel",
                    boost::bind(&AtlasMPNet::Problem::SetLogLevelCommand, this, _1, _2),
                    "set the log level");
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
        OMPL_ERROR("Robot and params must not be NULL.\n"); // NOLINT(hicpp-signed-bitwise)
        return initialized_;
    }
    // delete all the previously stored members
    parameters_.reset();
    env_.reset();
    robot_.reset();
    tsr_robot_.reset();
    tsr_chain_.reset();
    ambient_state_space_.reset();
    constraint_.reset();
    constrained_state_space_.reset();
    constrained_space_info_.reset();
    planner_.reset();
    state_validity_checker_.reset();
    start_.clear();
    goal_.clear();
    simple_setup_.reset();

    robot_ = std::move(robot);
    parameters_ = boost::make_shared<AtlasMPNet::Parameters>();
    parameters_->copy(params);
    initialized_ = setTSRChainRobot() &&
                   setAmbientStateSpace() &&
                   setConstrainedStateSpace() &&
                   setStartAndGoalStates() &&
                   setStateValidityChecker() &&
                   setPlanner() &&
                   simpleSetup();
    return initialized_;
}

OpenRAVE::PlannerStatus AtlasMPNet::Problem::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {
    OpenRAVE::PlannerStatus plannerStatus = OpenRAVE::PS_Failed;
    if (!initialized_) {
        OMPL_ERROR("Unable to plan. Did you call InitPlan?\n"); // NOLINT(hicpp-signed-bitwise)
        return plannerStatus;
    }
    ompl::base::PlannerStatus status = simple_setup_->solve(parameters_->solver_parameters_.time_);
    if (parameters_->constraint_parameters_.type_ != ConstraintParameters::PROJECTION)
        OMPL_INFORM ("Atlas charts created: %d", constrained_state_space_->as<ompl::base::AtlasStateSpace>()->getChartCount());
    switch (ompl::base::PlannerStatus::StatusType(status)) {
        case ompl::base::PlannerStatus::UNKNOWN:
            OMPL_WARN("Unknown status!");
            break;
        case ompl::base::PlannerStatus::INVALID_START:
            OMPL_WARN("Invalid start!");
            break;
        case ompl::base::PlannerStatus::INVALID_GOAL:
            OMPL_WARN("Invalid goal!");
            break;
        case ompl::base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE:
            OMPL_WARN("Unrecognized goal type!");
            break;
        case ompl::base::PlannerStatus::TIMEOUT:
            OMPL_WARN("Failed to find a solution!");
            break;
        case ompl::base::PlannerStatus::CRASH:
            OMPL_WARN("The planner crashed!");
            break;
        case ompl::base::PlannerStatus::ABORT:
            OMPL_WARN("The planner did not find a solution for some other reason!");
            break;
        case ompl::base::PlannerStatus::APPROXIMATE_SOLUTION:
            OMPL_WARN("Found an approximate solution. ");
//            break;
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
            OMPL_INFORM("States in path: %d", ompl_traj.getStateCount());
            std::stringstream ss;
            ss << std::endl;
            for(int i=0; i<3;i++) {
                ss << "\tState " << i << ":";
                space->copyToReals(values, ompl_traj.getState(i));
                for (auto v:values){
                    ss << " " << v;
                }
                ss << "\tDistance: " << constraint_->distance(ompl_traj.getState(i)) << std::endl;
            }
            for(int i=ompl_traj.getStateCount()-3; i<ompl_traj.getStateCount();i++) {
                ss << "\tState " << i << ":";
                space->copyToReals(values, ompl_traj.getState(i));
                for (auto v:values){
                    ss << " " << v;
                }
                ss << "\tDistance: " << constraint_->distance(ompl_traj.getState(i)) << std::endl;
            }
            OMPL_DEBUG(ss.str().c_str());
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

bool AtlasMPNet::Problem::GetPlanningTimeCommand(std::ostream &sout, std::istream &sin) const {
    sout << simple_setup_->getLastPlanComputationTime();
    return true;
}

bool AtlasMPNet::Problem::SetLogLevelCommand(std::ostream &sout, std::istream &sin) const {
    int leveli;
    sin >> leveli;
    ompl::msg::LogLevel level;
    level = static_cast<ompl::msg::LogLevel>(leveli);
    ompl::msg::setLogLevel(level);
    sout << "1";
}

bool AtlasMPNet::Problem::setTSRChainRobot() {
    env_ = robot_->GetEnv();
    tsr_chain_ = std::make_shared<TaskSpaceRegionChain>();
    *tsr_chain_ = parameters_->tsrchain_parameters_;
    tsr_chain_->Initialize(env_);
    tsr_chain_->RobotizeTSRChain(env_, tsr_robot_);

    // print the result
    if (tsr_robot_ != nullptr) {
        OMPL_INFORM("Constructed virtual TSR robot successfully.");
        std::stringstream ss;
        ss << std::endl;
        ss << "\tDOF: " << tsr_robot_->GetDOF() << std::endl;
        ss << "\tActive DOF: " << tsr_robot_->GetActiveDOF() << std::endl;
        ss << "\tNum of manipulators: " << tsr_robot_->GetManipulators().size() << std::endl;
        ss << "\tNum of active manipulators: " << (tsr_robot_->GetActiveManipulator() != nullptr);
        OMPL_DEBUG(ss.str().c_str());
        return true;
    } else {
        OMPL_ERROR("Failed to construct virtual TSR robot!");
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
    if (ambient_state_space_ == nullptr) {
        OMPL_ERROR("Failed to construct ambient state space!");
        return false;
    }

    // set bound and resolution if constructed successfully
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

    // print result
    OMPL_INFORM("Constructed ambient state space successfully.");
    std::stringstream ss;
    ss << std::endl;
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
    OMPL_DEBUG(ss.str().c_str());
    return true;
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
    if (constraint_ == nullptr || constrained_state_space_ == nullptr || constrained_space_info_ == nullptr) {
        OMPL_ERROR("Failed to construct constrained state space!");
        return false;
    }

    // setup parameters if constructed successfully
    constraint_->setTolerance(parameters_->constraint_parameters_.tolerance_);
    constraint_->setMaxIterations(parameters_->constraint_parameters_.max_iter_);
    constrained_state_space_->setDelta(parameters_->constraint_parameters_.delta_);
    constrained_state_space_->setLambda(parameters_->constraint_parameters_.lambda_);
    if (parameters_->constraint_parameters_.type_ != ConstraintParameters::PROJECTION) {
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

    // print result
    OMPL_INFORM("Constructed constrained state space successfully.");
    std::stringstream ss;
    ss << std::endl;
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
    if (parameters_->constraint_parameters_.type_ != ConstraintParameters::PROJECTION) {
        auto constrained_state_space_temp = constrained_state_space_->as<ompl::base::AtlasStateSpace>();
        ss << "\tParameters of atlas state space: " << std::endl;
        ss << "\t\tExploration (tunes balance of refinement and exploration in atlas sampling): " << constrained_state_space_temp->getExploration()
           << std::endl;
        ss << "\t\tEpsilon (max distance from manifold to chart): " << constrained_state_space_temp->getEpsilon() << std::endl;
        ss << "\t\tRho (max radius for an atlas chart): " << constrained_state_space_temp->getRho() << std::endl;
        ss << "\t\tAlpha (max angle between chart and manifold): " << constrained_state_space_temp->getAlpha() << std::endl;
        ss << "\t\tMax chart generated during a traversal: " << constrained_state_space_temp->getMaxChartsPerExtension();
    }
    OMPL_DEBUG(ss.str().c_str());
    return true;
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
    std::cout << "Visualize start. Press enter to continue." << std::endl;
    std::cin.get();
    // goal config
    parameters_->getGoalState(robot_goal);
    robot_->SetActiveDOFValues(robot_goal);
    tsr_chain_->GetClosestTransform(robot_->GetActiveManipulator()->GetEndEffectorTransform(), tsr_goal, Ttemp);
    std::cout << "Visualize goal. Press enter to continue." << std::endl;
    std::cin.get();

    // concatenate config of robot and tsr
    start_.insert(start_.end(), robot_start.begin(), robot_start.end());
    start_.insert(start_.end(), tsr_start.begin(), tsr_start.end());
    goal_.insert(goal_.end(), robot_goal.begin(), robot_goal.end());
    goal_.insert(goal_.end(), tsr_goal.begin(), tsr_goal.end());

    // print result
    OMPL_INFORM("Set start and goal successfully.");
    std::stringstream ss;
    ss << std::endl;
    ss << "\tStart:" << std::endl;
    ss << "\t\tRobot:";
    for (int i = 0; i < dof_robot; i++) {
        ss << "\t" << start_[i];
    }
    ss << std::endl;
    ss << "\t\tTSR:";
    for (int i = dof_robot; i < dof_robot + dof_tsr; i++) {
        ss << "\t" << start_[i];
    }
    ss << std::endl;
    Eigen::VectorXd start_temp = Eigen::VectorXd::Map(start_.data(), start_.size());
    ss << "\t\tDistance: " << constraint_->distance(start_temp) << std::endl;
    ss << "\tGoal:" << std::endl;
    ss << "\t\tRobot:";
    for (int i = 0; i < dof_robot; i++) {
        ss << "\t" << goal_[i];
    }
    ss << std::endl;
    ss << "\t\tTSR:";
    for (int i = dof_robot; i < dof_robot + dof_tsr; i++) {
        ss << "\t" << goal_[i];
    }
    ss << std::endl;
    Eigen::VectorXd goal_temp = Eigen::VectorXd::Map(goal_.data(), goal_.size());
    ss << "\t\tDistance: " << constraint_->distance(goal_temp);
    OMPL_DEBUG(ss.str().c_str());
    return true;
}

bool AtlasMPNet::Problem::setStateValidityChecker() {
    state_validity_checker_ = std::make_shared<AtlasMPNet::StateValidityChecker>(constrained_space_info_, robot_, tsr_robot_, tsr_chain_);
//    state_validity_checker_ = std::make_shared<ompl::base::AllValidStateValidityChecker>(constrained_space_info_);
    // print result
    if (state_validity_checker_ != nullptr) {
        OMPL_INFORM("Constructed state validity checker successfully.");
        return true;
    } else {
        OMPL_ERROR("Failed to construct state validity checker!");
        return false;
    }
}

bool AtlasMPNet::Problem::setPlanner() {
    // create a planner
    switch (parameters_->solver_parameters_.type_) {
        case SolverParameters::RRT:
            planner_ = std::make_shared<ompl::geometric::RRT>(constrained_space_info_);
            break;
        case SolverParameters::RRTStar:
            planner_ = std::make_shared<ompl::geometric::RRTstar>(constrained_space_info_);
            break;
        case SolverParameters::RRTConnect:
            planner_ = std::make_shared<ompl::geometric::RRTConnect>(constrained_space_info_);
            break;
        case SolverParameters::MPNet:
            break;
    }
    if (planner_ == nullptr) {
        OMPL_ERROR("Failed to construct planner!");
        return false;
    }

    // set the range if constructed successfully
    switch (parameters_->solver_parameters_.type_) {
        case SolverParameters::RRT:
            planner_->as<ompl::geometric::RRT>()->setRange(parameters_->solver_parameters_.range_);
            break;
        case SolverParameters::RRTStar:
            planner_->as<ompl::geometric::RRTstar>()->setRange(parameters_->solver_parameters_.range_);
            break;
        case SolverParameters::RRTConnect:
            planner_->as<ompl::geometric::RRTConnect>()->setRange(parameters_->solver_parameters_.range_);
            break;
        case SolverParameters::MPNet:
            break;
    }

    // print result
    OMPL_INFORM("Constructed planner successfully.");
    std::stringstream ss;
    ss << std::endl;
    ss << "\tPlanner: " << planner_->getName() << std::endl;
    switch (parameters_->solver_parameters_.type_) {
        case SolverParameters::RRT:
            ss << "\t\tRange: " << planner_->as<ompl::geometric::RRT>()->getRange();
            break;
        case SolverParameters::RRTStar:
            ss << "\t\tRange: " << planner_->as<ompl::geometric::RRTstar>()->getRange();
            break;
        case SolverParameters::RRTConnect:
            ss << "\t\tRange: " << planner_->as<ompl::geometric::RRTConnect>()->getRange();
            break;
        case SolverParameters::MPNet:
            break;
    }
    OMPL_DEBUG(ss.str().c_str());
    return true;
}

bool AtlasMPNet::Problem::simpleSetup() {
    // create a SimpleSetup
    simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(constrained_space_info_);
    if (simple_setup_ == nullptr) {
        OMPL_ERROR("Failed to construct simple setup!");
        return false;
    }
    // do some setup if constructed successfully
    ompl::base::ScopedState<> start(constrained_state_space_), goal(constrained_state_space_);
    for (int i = 0; i < constrained_state_space_->getAmbientDimension(); i++) {
        start[i] = start_[i];
        goal[i] = goal_[i];
    }
    if (parameters_->constraint_parameters_.type_ != ConstraintParameters::PROJECTION) {
        auto constrained_state_space_temp = constrained_state_space_->as<ompl::base::AtlasStateSpace>();
        constrained_state_space_temp->anchorChart(start.get());
        constrained_state_space_temp->anchorChart(goal.get());
    }
    simple_setup_->setStartAndGoalStates(start, goal);
    simple_setup_->setStateValidityChecker(state_validity_checker_);
    simple_setup_->setPlanner(planner_);
    simple_setup_->setup();
    if(!state_validity_checker_->isValid(start.get())){
        OMPL_WARN("Start is not valid!");
        return false;
    }
    if(!state_validity_checker_->isValid(goal.get())) {
        OMPL_WARN("Goal is not valid!");
        return false;
    }
    OMPL_INFORM("Constructed simple setup successfully.");
    return true;
}
