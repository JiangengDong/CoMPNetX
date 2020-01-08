//
// Created by jiangeng on 10/11/19.
//

#ifndef ATLASMPNET_PROBLEM_H
#define ATLASMPNET_PROBLEM_H

#include <openrave/openrave.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/StateValidityChecker.h>

#include "Parameters.h"
#include "Constraint.h"
#include "StateValidityChecker.h"
#include "TaskSpaceRegionChain.h"

namespace AtlasMPNet {
    /*! \brief The main class in this plugin.
     *
     * It accepts a OpenRAVE Environtment and a Robot, and generates the corresponding OMPL StateSpace.
     * Then a planner can be called to planning a path in the StateSpace.
     */
    class Problem : public OpenRAVE::PlannerBase {
    public:
        Problem(OpenRAVE::EnvironmentBasePtr penv, std::istream &ss);

        ~Problem() override;

        bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream &input) override;

        bool InitPlan(OpenRAVE::RobotBasePtr robot,
                      OpenRAVE::PlannerBase::PlannerParametersConstPtr params) override;

        OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) override;

        OpenRAVE::PlannerBase::PlannerParametersConstPtr GetParameters() const override;

        bool GetParametersCommand(std::ostream &sout, std::istream &sin) const;
        bool GetPlanningTimeCommand(std::ostream &sout, std::istream &sin) const;
        bool SetLogLevelCommand(std::ostream &sout, std::istream &sin) const;

    private:
        bool setTSRChainRobot();

        bool setAmbientStateSpace();

        bool setConstrainedStateSpace();

        bool simpleSetup();

        bool setStartAndGoalStates();

        bool setStateValidityChecker();

        bool setPlanner();

        AtlasMPNet::Parameters::Ptr parameters_;

        OpenRAVE::EnvironmentBasePtr env_;
        OpenRAVE::RobotBasePtr robot_;
        OpenRAVE::RobotBasePtr tsr_robot_;
        TaskSpaceRegionChain::Ptr tsr_chain_;

        ompl::base::StateSpacePtr ambient_state_space_;
        ompl::base::ConstraintPtr constraint_;
        ompl::base::ConstrainedStateSpacePtr constrained_state_space_;
        ompl::base::ConstrainedSpaceInformationPtr constrained_space_info_;
        ompl::base::PlannerPtr planner_;
        ompl::base::StateValidityCheckerPtr state_validity_checker_;
        std::vector<double> start_;
        std::vector<double> goal_;
        ompl::geometric::SimpleSetupPtr simple_setup_;

        bool initialized_ = false;
    };
}

#endif //ATLASMPNET_PROBLEM_H
