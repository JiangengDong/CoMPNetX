/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
Copyright (c) 2020, University of California, San Diego
All rights reserved.

Authors: Michael Koval <mkoval@cs.cmu.edu>
         Matthew Klingensmith <mklingen@cs.cmu.edu>
         Christopher Dellin <cdellin@cs.cmu.edu>
Modifier: Jiangeng Dong <jid103@ucsd.edu>

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
#include "Problem.h"
#include <cstddef>
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <openrave/openrave.h>
#include <utility>

#include <boost/make_shared.hpp>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include "planner/MPNetXPlanner.h"

CoMPNetX::Problem::Problem(OpenRAVE::EnvironmentBasePtr penv, std::istream &ss) : OpenRAVE::PlannerBase(std::move(penv)) {
    RegisterCommand("GetParameters", boost::bind(&CoMPNetX::Problem::GetParametersCommand, this, _1, _2), "returns the values of all the parameters");
    RegisterCommand("GetPlanningTime", boost::bind(&CoMPNetX::Problem::GetPlanningTimeCommand, this, _1, _2),
                    "returns the amount of time (in seconds) spent during the last planning step");
    RegisterCommand("SetLogLevel", boost::bind(&CoMPNetX::Problem::SetLogLevelCommand, this, _1, _2), "set the log level");
    RegisterCommand("SetShortcutIteration", boost::bind(&CoMPNetX::Problem::SetShortcutIteration, this, _1, _2), "set the log level");
    RegisterCommand("GetDistanceToManifold", boost::bind(&CoMPNetX::Problem::GetDistanceToManifoldCommand, this, _1, _2), "calculate the distance from a config to the manifold");
}

CoMPNetX::Problem::~Problem() = default;

bool CoMPNetX::Problem::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream &input) {
    CoMPNetX::Parameters::Ptr params = boost::make_shared<CoMPNetX::Parameters>();
    input >> *params;
    return InitPlan(robot, params);
}

bool CoMPNetX::Problem::InitPlan(OpenRAVE::RobotBasePtr robot, OpenRAVE::PlannerBase::PlannerParametersConstPtr params) {
    initialized_ = false;
    if (robot == nullptr || params == nullptr) {
        OMPL_ERROR("Robot and params must not be NULL.\n"); // NOLINT(hicpp-signed-bitwise)
        return initialized_;
    }
    // delete all the previously stored members
    parameters_.reset();
    env_.reset();
    robot_.reset();
    tsrchains_.clear();
    ambient_state_space_.reset();
    constraint_.reset();
    constrained_state_space_.reset();
    constrained_space_info_.reset();
    planner_.reset();
    state_validity_checker_.reset();
    start_.clear();
    goal_.clear();
    simple_setup_.reset();

    parameters_ = boost::make_shared<CoMPNetX::Parameters>();
    parameters_->copy(params);
    env_ = robot->GetEnv();
    robot_ = std::move(robot);

    initialized_ = setTSRChainRobot()            //
                   && setAmbientStateSpace()     //
                   && setConstrainedStateSpace() //
                   && setStartAndGoalStates()    //
                   && setStateValidityChecker()  //
                   && setPlanner()               //
                   && simpleSetup();
    return initialized_;
}

OpenRAVE::PlannerStatus CoMPNetX::Problem::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {
    // Initialize ptraj
    size_t dof = robot_->GetActiveDOF();
    OpenRAVE::ConfigurationSpecification planningspec = robot_->GetActiveConfigurationSpecification("linear");
    for (const auto &tsrchain : tsrchains_) {
        auto mimic_body = tsrchain->GetMimicBody();
        if (mimic_body != nullptr) {
            dof = dof + tsrchain->GetNumMimicDOF();
            mimic_body->SetActiveDOFs(tsrchain->GetMimicDOFInds());
            planningspec = planningspec + mimic_body->GetActiveConfigurationSpecification("linear");
        }
    }
    ptraj->Init(planningspec);

    OpenRAVE::PlannerStatus plannerStatus = OpenRAVE::PS_Failed;
    if (!initialized_) {
        OMPL_ERROR("Unable to plan. Did you call InitPlan?\n"); // NOLINT(hicpp-signed-bitwise)
        return plannerStatus;
    }
    ompl::base::PlannerStatus status = simple_setup_->solve(parameters_->solver_parameter_.time_);
    if (parameters_->constraint_parameter_.type_ != ConstraintParameter::PROJECTION)
        OMPL_INFORM("Atlas charts created: %d", constrained_state_space_->as<ompl::base::AtlasStateSpace>()->getChartCount());
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
            break;
        case ompl::base::PlannerStatus::EXACT_SOLUTION: {
            auto ompl_traj = simple_setup_->getSolutionPath();
            simplfyOnManifold(ompl_traj, ptraj);

            ompl::base::StateSpacePtr space = ompl_traj.getSpaceInformation()->getStateSpace();
            std::vector<double> values, robot_values;

            // print the result
            OMPL_INFORM("States in path: %d", ompl_traj.getStateCount());
            std::stringstream ss;
            ss << std::endl;
            for (int i = 0; i < 3; i++) {
                ss << "\tState " << i << ":";
                space->copyToReals(values, ompl_traj.getState(i));
                for (auto v : values) {
                    ss << " " << v;
                }
                ss << "\tDistance: " << constraint_->distance(ompl_traj.getState(i)) << std::endl;
            }
            for (unsigned long i = ompl_traj.getStateCount() - 3; i < ompl_traj.getStateCount(); i++) {
                ss << "\tState " << i << ":";
                space->copyToReals(values, ompl_traj.getState(i));
                for (auto v : values) {
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

OpenRAVE::PlannerBase::PlannerParametersConstPtr CoMPNetX::Problem::GetParameters() const {
    return parameters_;
}

bool CoMPNetX::Problem::GetParametersCommand(std::ostream &sout, std::istream &sin) const {
    sout << parameters_->solver_parameter_ << std::endl
         << parameters_->constraint_parameter_ << std::endl
         << parameters_->atlas_parameter_ << std::endl;
    for (const auto &tsrchain : parameters_->tsrchains_) {
        sout << tsrchain << std::endl;
    }
    return true;
}

bool CoMPNetX::Problem::GetPlanningTimeCommand(std::ostream &sout, std::istream &sin) const {
    sout << simple_setup_->getLastPlanComputationTime();
    return true;
}

bool CoMPNetX::Problem::SetLogLevelCommand(std::ostream &sout, std::istream &sin) const {
    int leveli;
    sin >> leveli;
    ompl::msg::LogLevel level;
    level = static_cast<ompl::msg::LogLevel>(leveli);
    ompl::msg::setLogLevel(level);
    sout << "1";
    return true;
}

bool CoMPNetX::Problem::SetShortcutIteration(std::ostream &sout, std::istream &sin) {
    sin >> shortcut_iteration_;
    sout << "1";
    return true;
}

bool CoMPNetX::Problem::GetDistanceToManifoldCommand(std::ostream &sout, std::istream &sin) const {
    auto dof = robot_->GetActiveDOF();
    std::vector<double> joint_vals(dof);
    double temp;
    for (unsigned int i = 0; i < static_cast<unsigned int>(dof); i++) {
        sin >> temp;
        joint_vals[i] = temp;
    }
    robot_->SetActiveDOFValues(joint_vals);

    auto manips = robot_->GetManipulators();
    double dist_square = 0;
    for (const auto &tsrchain : tsrchains_) {
        auto Trobot = manips[tsrchain->GetManipInd()]->GetEndEffectorTransform();
        OpenRAVE::Transform Ttsr;
        std::vector<double> tsr_joint_vals;
        tsrchain->GetRobot()->GetActiveDOFValues(tsr_joint_vals);
        double dist = tsrchain->GetClosestTransform(Trobot, tsr_joint_vals, Ttsr);
        for (const auto &val : tsr_joint_vals) {
            sout << val << " ";
        }
        dist_square += dist * dist;
    }

    sout << std::sqrt(dist_square);

    return true;
}

float CoMPNetX::Problem::getPathLength(const std::vector<ompl::base::State *> &path, std::size_t l, std::size_t r) {
    if (r - l < 2) return 0.0;

    float total_length = 0.0;
    for (std::size_t m = l; m < r - 1; m++) {
        total_length += constrained_state_space_->distance(path[m], path[m + 1]);
    }

    return total_length;
}

bool CoMPNetX::Problem::simplfyOnManifold(const ompl::geometric::PathGeometric &input_traj, OpenRAVE::TrajectoryBasePtr output_traj) {
    std::vector<ompl::base::State *> input_path;
    std::vector<ompl::base::State *> output_path;
    // copy states out for easier iteration
    input_path.resize(input_traj.getStateCount(), nullptr);
    constrained_space_info_->allocStates(input_path);
    for (std::size_t i = 0; i < input_traj.getStateCount(); i++) {
        constrained_state_space_->copyState(input_path[i], input_traj.getState(i));
    }

    for (std::size_t i = 0; i < shortcut_iteration_; i++) {
        std::size_t input_start = 0;
        while (input_start < input_traj.getStateCount() - 2) {
            std::size_t input_end;
            for (input_end = input_traj.getStateCount() - 1; input_end > input_start + 1; input_end--) {
                // try to shortcut
                float original_length = getPathLength(input_path, input_start, input_end + 1);
                std::vector<ompl::base::State *> shortcut_path;
                bool succeed = constrained_state_space_->discreteGeodesic(input_path[input_start], input_path[input_end], false, &shortcut_path);
                if (succeed &&
                    shortcut_path.size() > 2 &&
                    getPathLength(shortcut_path, 0, shortcut_path.size()) < original_length) {
                    // copy shortcut_path to the output_path
                    for (std::size_t i = 0; i < shortcut_path.size() - 1; i++) {
                        const auto *state = shortcut_path[i];
                        ompl::base::State *copy_state = constrained_space_info_->allocState();
                        constrained_space_info_->copyState(copy_state, state);
                        output_path.emplace_back(copy_state);
                    }
                    // only copy the last point of the shortcut if it is the same as the goal
                    if (input_end == input_traj.getStateCount() - 1) {
                        const auto *state = shortcut_path.back();
                        ompl::base::State *copy_state = constrained_space_info_->allocState();
                        constrained_space_info_->copyState(copy_state, state);
                        output_path.emplace_back(copy_state);
                    }
                    input_start = input_end;
                }
                constrained_space_info_->freeStates(shortcut_path);
            }
            // if not shortcut is available from this start, move to the next one
            if (input_start != input_end) {
                input_start++;
            }
        }
        input_path.swap(output_path);
        constrained_space_info_->freeStates(output_path);
    }

    // copy shortcut traj to output
    std::vector<double> values, robot_values;
    std::size_t dof = output_traj->GetConfigurationSpecification().GetDOF();
    for (std::size_t i = 0; i<input_path.size(); i++) {
        constrained_state_space_->copyToReals(values, input_path[i]);
        robot_values.assign(values.begin(), values.begin() + dof);
        output_traj->Insert(i, robot_values, true);
    }

    // cleanup
    constrained_space_info_->freeStates(input_path);
    return true;
}

bool CoMPNetX::Problem::setTSRChainRobot() {
    env_ = robot_->GetEnv();
    TaskSpaceRegionChain::Ptr tsrchain_temp;
    for (const auto &tsrchain_param : parameters_->tsrchains_) {
        tsrchain_temp = std::make_shared<TaskSpaceRegionChain>(env_, tsrchain_param);
        if (tsrchain_temp == nullptr) {
            OMPL_ERROR("Failed to construct virtual TSR robot!");
            return false;
        }
        tsrchains_.emplace_back(tsrchain_temp);
    }

    // print the result
    OMPL_INFORM("Constructed virtual TSR robot successfully.");
    std::stringstream ss;
    ss << std::endl;
    for (unsigned int i = 0; i < tsrchains_.size(); i++) {
        ss << "\tTSR Chain " << i << "    DOF: " << tsrchains_[i]->GetNumDOF() << std::endl;
    }
    OMPL_DEBUG(ss.str().c_str());
    return true;
}

/** \brief Setup the ambient state space
 *
 * The config is composed of two parts.
 * The first one is the joint values of real robot,
 * and the second one is the joint values of virtual TSRChain robot.
 *
 * @return true if setup successfully
 */
bool CoMPNetX::Problem::setAmbientStateSpace() {
    const unsigned int dof = robot_->GetActiveDOF();
    unsigned int dof_tsrs = 0;
    for (const auto &tsrchain : tsrchains_) {
        dof_tsrs += tsrchain->GetNumDOF();
    }
    ambient_state_space_ = std::make_shared<ompl::base::RealVectorStateSpace>(dof + dof_tsrs);
    if (ambient_state_space_ == nullptr) {
        OMPL_ERROR("Failed to construct ambient state space!");
        return false;
    }

    // set bound and resolution if constructed successfully
    auto ambient_state_space_temp = ambient_state_space_->as<ompl::base::RealVectorStateSpace>();
    // Set bounds
    ompl::base::RealVectorBounds bounds(dof + dof_tsrs);
    std::vector<OpenRAVE::dReal> lower_limits, upper_limits;
    unsigned int offset = 0;
    // get bounds for real robot
    robot_->GetActiveDOFLimits(lower_limits, upper_limits);
    for (unsigned int i = 0; i < dof; ++i) {
        bounds.setLow(i, lower_limits[i]);
        bounds.setHigh(i, upper_limits[i]);
    }
    offset += dof;
    // get bounds for virtual robot
    for (const auto &tsrchain : tsrchains_) {
        unsigned int dof_tsr = tsrchain->GetNumDOF();
        tsrchain->GetChainJointLimits(lower_limits, upper_limits);
        for (unsigned int i = 0; i < dof_tsr; ++i) {
            bounds.setLow(i + offset, lower_limits[i]);
            bounds.setHigh(i + offset, upper_limits[i]);
        }
        offset += dof_tsr;
    }

    ambient_state_space_temp->setBounds(bounds);
    bounds = ambient_state_space_temp->getBounds();

    // Set resolution
    std::vector<OpenRAVE::dReal> dof_resolutions;
    double conservative_resolution = std::numeric_limits<double>::max();
    robot_->GetActiveDOFResolutions(dof_resolutions);
    for (const auto &dof_resolution : dof_resolutions) {
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
    for (auto hb : bounds.high) {
        ss << " " << hb;
    }
    ss << std::endl;
    ss << "\tLower bound:";
    for (auto lb : bounds.low) {
        ss << " " << lb;
    }
    ss << std::endl;
    ss << "\tResolution: " << conservative_resolution;
    OMPL_DEBUG(ss.str().c_str());
    return true;
}

bool CoMPNetX::Problem::setConstrainedStateSpace() {
    constraint_ = std::make_shared<TSRChainConstraint>(robot_, tsrchains_);
    // create the constrained configuration space
    switch (parameters_->constraint_parameter_.type_) {
        case ConstraintParameter::PROJECTION:
            constrained_state_space_ = std::make_shared<ompl::base::ProjectedStateSpace>(ambient_state_space_, constraint_);
            constrained_space_info_ = std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space_);
            break;
        case ConstraintParameter::ATLAS:
            constrained_state_space_ = std::make_shared<ompl::base::AtlasStateSpace>(ambient_state_space_, constraint_);
            constrained_space_info_ = std::make_shared<ompl::base::ConstrainedSpaceInformation>(constrained_state_space_);
            break;
        case ConstraintParameter::TANGENT_BUNDLE:
            constrained_state_space_ = std::make_shared<ompl::base::TangentBundleStateSpace>(ambient_state_space_, constraint_);
            constrained_space_info_ = std::make_shared<ompl::base::TangentBundleSpaceInformation>(constrained_state_space_);
            break;
    }
    if (constraint_ == nullptr || constrained_state_space_ == nullptr || constrained_space_info_ == nullptr) {
        OMPL_ERROR("Failed to construct constrained state space!");
        return false;
    }

    // setup parameters if constructed successfully
    constraint_->setTolerance(parameters_->constraint_parameter_.tolerance_);
    constraint_->setMaxIterations(parameters_->constraint_parameter_.max_iter_);
    constrained_state_space_->setDelta(parameters_->constraint_parameter_.delta_);
    constrained_state_space_->setLambda(parameters_->constraint_parameter_.lambda_);
    if (parameters_->constraint_parameter_.type_ != ConstraintParameter::PROJECTION) {
        auto constrained_state_space_temp = constrained_state_space_->as<ompl::base::AtlasStateSpace>();
        constrained_state_space_temp->setExploration(parameters_->atlas_parameter_.exploration_);
        constrained_state_space_temp->setEpsilon(parameters_->atlas_parameter_.epsilon_);
        constrained_state_space_temp->setRho(parameters_->atlas_parameter_.rho_);
        constrained_state_space_temp->setAlpha(parameters_->atlas_parameter_.alpha_);
        constrained_state_space_temp->setMaxChartsPerExtension(parameters_->atlas_parameter_.max_charts_);

        auto &&atlas = constrained_state_space_temp;
        if (parameters_->atlas_parameter_.using_bias_) { // add different weight for sampling to different charts
            constrained_state_space_temp->setBiasFunction([atlas](ompl::base::AtlasChart *c) -> double {
                return 1.0 + atlas->getChartCount() - c->getNeighborCount();
            });
        }
        if (parameters_->constraint_parameter_.type_ == ConstraintParameter::ATLAS) {
            constrained_state_space_temp->setSeparated(parameters_->atlas_parameter_.separate_);
        }
    }

    // print result
    OMPL_INFORM("Constructed constrained state space successfully.");
    std::stringstream ss;
    ss << std::endl;
    switch (parameters_->constraint_parameter_.type_) {
        case ConstraintParameter::PROJECTION:
            ss << "\tType: Projection" << std::endl;
            break;
        case ConstraintParameter::ATLAS:
            ss << "\tType: Atlas" << std::endl;
            break;
        case ConstraintParameter::TANGENT_BUNDLE:
            ss << "\tType: Tangent bundle" << std::endl;
            break;
    }
    ss << "\tDimension of manifold: " << constrained_state_space_->getManifoldDimension() << std::endl;
    ss << "\tParameters of constrained state space: " << std::endl;
    ss << "\t\tTolerance: " << constraint_->getTolerance() << std::endl;
    ss << "\t\tMax projection iteration:" << constraint_->getMaxIterations() << std::endl;
    ss << "\t\tDelta (Step-size for discrete geodesic on manifold): " << constrained_state_space_->getDelta() << std::endl;
    ss << "\t\tLambda (Maximum `wandering` allowed during traversal): " << constrained_state_space_->getLambda() << std::endl;
    if (parameters_->constraint_parameter_.type_ != ConstraintParameter::PROJECTION) {
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

bool CoMPNetX::Problem::setStartAndGoalStates() {
    std::vector<double> qtemp;
    OpenRAVE::Transform Ttemp;
    std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manips = robot_->GetManipulators();

    // start config
    parameters_->getStartState(qtemp); // get the joint values of real robot
    start_.insert(start_.end(), qtemp.begin(), qtemp.end());
    robot_->SetActiveDOFValues(qtemp);
    for (const auto &tsrchain : tsrchains_) {
        tsrchain->GetClosestTransform(manips[tsrchain->GetManipInd()]->GetEndEffectorTransform(), qtemp, Ttemp);
        start_.insert(start_.end(), qtemp.begin(), qtemp.end());
    }

    // goal config
    parameters_->getGoalState(qtemp);
    goal_.insert(goal_.end(), qtemp.begin(), qtemp.end());
    robot_->SetActiveDOFValues(qtemp);
    for (const auto &tsrchain : tsrchains_) {
        tsrchain->GetClosestTransform(manips[tsrchain->GetManipInd()]->GetEndEffectorTransform(), qtemp, Ttemp);
        goal_.insert(goal_.end(), qtemp.begin(), qtemp.end());
    }

    // print result
    OMPL_INFORM("Set start and goal successfully.");
    std::stringstream ss;
    int offset;
    int dof_robot = robot_->GetActiveDOF();
    ss << std::endl;
    ss << "\tStart:" << std::endl;
    offset = 0;
    ss << "\t\tRobot:";
    for (int i = 0; i < dof_robot; i++)
        ss << "\t" << start_[i + offset];
    ss << std::endl;
    offset += dof_robot;
    for (const auto &tsrchain : tsrchains_) {
        int dof_tsr = tsrchain->GetNumDOF();
        ss << "\t\tTSR:";
        for (int i = 0; i < dof_tsr; i++)
            ss << "\t" << start_[i + offset];
        ss << std::endl;
        offset += dof_tsr;
    }
    Eigen::VectorXd start_temp = Eigen::VectorXd::Map(start_.data(), start_.size());
    ss << "\t\tDistance: " << constraint_->distance(start_temp) << std::endl;

    ss << "\tGoal:" << std::endl;
    offset = 0;
    ss << "\t\tRobot:";
    for (int i = 0; i < dof_robot; i++)
        ss << "\t" << goal_[i + offset];
    ss << std::endl;
    offset += dof_robot;
    for (const auto &tsrchain : tsrchains_) {
        int dof_tsr = tsrchain->GetNumDOF();
        ss << "\t\tTSR:";
        for (int i = 0; i < dof_tsr; i++)
            ss << "\t" << goal_[i + offset];
        ss << std::endl;
        offset += dof_tsr;
    }

    Eigen::VectorXd goal_temp = Eigen::VectorXd::Map(goal_.data(), goal_.size());
    ss << "\t\tDistance: " << constraint_->distance(goal_temp);
    OMPL_DEBUG(ss.str().c_str());
    return true;
}

bool CoMPNetX::Problem::setStateValidityChecker() {
    state_validity_checker_ = std::make_shared<CoMPNetX::StateValidityChecker>(constrained_space_info_, robot_, tsrchains_);
    // print result
    if (state_validity_checker_ != nullptr) {
        OMPL_INFORM("Constructed state validity checker successfully.");
        return true;
    } else {
        OMPL_ERROR("Failed to construct state validity checker!");
        return false;
    }
}

bool CoMPNetX::Problem::setPlanner() {
    // create a planner

    switch (parameters_->solver_parameter_.type_) {
        case SolverParameter::RRTConnect:
            planner_ = std::make_shared<ompl::geometric::RRTConnect>(constrained_space_info_);
            break;
        case SolverParameter::CoMPNetX:
            planner_ = std::make_shared<ompl::geometric::MPNetXPlanner>(constrained_space_info_, robot_, tsrchains_, parameters_->mpnet_parameter_);
            break;
        default:
            planner_ = nullptr;
            break;
    }

    if (planner_ == nullptr) {
        OMPL_ERROR("Failed to construct planner!");
        return false;
    }

    // set the range if constructed successfully
    switch (parameters_->solver_parameter_.type_) {
        case SolverParameter::RRT:
            planner_->as<ompl::geometric::RRT>()->setRange(parameters_->solver_parameter_.range_);
            break;
        case SolverParameter::RRTstar:
            planner_->as<ompl::geometric::RRTstar>()->setRange(parameters_->solver_parameter_.range_);
            break;
        case SolverParameter::RRTConnect:
            planner_->as<ompl::geometric::RRTConnect>()->setRange(parameters_->solver_parameter_.range_);
            break;
        default:
            break;
    }

    // print result
    OMPL_INFORM("Constructed planner successfully.");
    std::stringstream ss;
    ss << std::endl;
    ss << "\tPlanner: " << planner_->getName() << std::endl;
    switch (parameters_->solver_parameter_.type_) {
        case SolverParameter::RRT:
            ss << "\t\tRange: " << planner_->as<ompl::geometric::RRT>()->getRange();
            break;
        case SolverParameter::RRTstar:
            ss << "\t\tRange: " << planner_->as<ompl::geometric::RRTstar>()->getRange();
            break;
        case SolverParameter::RRTConnect:
            ss << "\t\tRange: " << planner_->as<ompl::geometric::RRTConnect>()->getRange();
            break;
        default:
            break;
    }
    OMPL_DEBUG(ss.str().c_str());
    return true;
}

bool CoMPNetX::Problem::simpleSetup() {
    // create a SimpleSetup
    simple_setup_ = std::make_shared<ompl::geometric::SimpleSetup>(constrained_space_info_);
    if (simple_setup_ == nullptr) {
        OMPL_ERROR("Failed to construct simple setup!");
        return false;
    }
    // do some setup if constructed successfully
    ompl::base::ScopedState<> start(constrained_state_space_), goal(constrained_state_space_);
    for (unsigned int i = 0; i < constrained_state_space_->getAmbientDimension(); i++) {
        start[i] = start_[i];
        goal[i] = goal_[i];
    }
    if (parameters_->constraint_parameter_.type_ != ConstraintParameter::PROJECTION) {
        auto constrained_state_space_temp = constrained_state_space_->as<ompl::base::AtlasStateSpace>();
        constrained_state_space_temp->anchorChart(start.get());
        constrained_state_space_temp->anchorChart(goal.get());
    }
    simple_setup_->setStartAndGoalStates(start, goal);
    simple_setup_->setStateValidityChecker(state_validity_checker_);
    simple_setup_->setPlanner(planner_);
    simple_setup_->setup();
    if (!state_validity_checker_->isValid(start.get())) {
        OpenRAVE::CollisionReportPtr report = boost::make_shared<OpenRAVE::CollisionReport>();
        OMPL_WARN("Start is not valid!");
        if (env_->CheckCollision(robot_, report)) {
            std::cout << "Collision between " << report->plink1->GetParent()->GetName() << "/" << report->plink1->GetName() << "----"
                      << report->plink2->GetParent()->GetName() << "/" << report->plink2->GetName() << std::endl;
        }
        if (robot_->CheckSelfCollision(report)) {
            std::cout << "Collision between " << report->plink1->GetParent()->GetName() << "/" << report->plink1->GetName() << "----"
                      << report->plink2->GetParent()->GetName() << "/" << report->plink2->GetName() << std::endl;
        }
        return false;
    }
    if (!state_validity_checker_->isValid(goal.get())) {
        OpenRAVE::CollisionReportPtr report = boost::make_shared<OpenRAVE::CollisionReport>();
        OMPL_WARN("Goal is not valid!");
        if (env_->CheckCollision(robot_, report)) {
            std::cout << "Collision between " << report->plink1->GetParent()->GetName() << "/" << report->plink1->GetName() << "----"
                      << report->plink2->GetParent()->GetName() << "/" << report->plink2->GetName() << std::endl;
        }
        if (robot_->CheckSelfCollision(report)) {
            std::cout << "Collision between " << report->plink1->GetParent()->GetName() << "/" << report->plink1->GetName() << "----"
                      << report->plink2->GetParent()->GetName() << "/" << report->plink2->GetName() << std::endl;
        }
        return false;
    }
    OMPL_INFORM("Constructed simple setup successfully.");
    return true;
}
