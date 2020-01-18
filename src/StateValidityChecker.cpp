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

#include "StateValidityChecker.h"

#include <utility>
#include <openrave/openrave.h>
#include <ompl/base/StateValidityChecker.h>
#include <boost/make_shared.hpp>

#include "TaskSpaceRegionChain.h"

using namespace AtlasMPNet;

StateValidityChecker::StateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                           OpenRAVE::RobotBasePtr robot,
                                           AtlasMPNet::TaskSpaceRegionChain::Ptr tsr_chain) :
        ompl::base::StateValidityChecker(si),
        _state_space(si->getStateSpace()),
        _robot(std::move(robot)),
        _tsr_robot(tsr_chain->GetRobot()),
        _tsr_chain(std::move(tsr_chain)),
        _env(_robot->GetEnv()),
        _robot_dof(_robot->GetActiveDOF()),
        _tsr_dof(_tsr_robot->GetActiveDOF()),
        _robot_values(_robot_dof, 0),
        _tsr_values(_tsr_dof, 0) {
}

bool StateValidityChecker::computeFk(const ompl::base::State *state, uint32_t checklimits) const {
    std::vector<double> values;
    _state_space->copyToReals(values, state);

    std::vector<double> robot_values(values.begin(), values.begin() + _robot_dof);
    std::vector<double> tsr_values(values.begin() + _robot_dof, values.begin() + _robot_dof + _tsr_dof);

    for (double v: robot_values) {
        if (std::isnan(v)) {
                    RAVELOG_WARN("Invalid value in state.\n");
            return false;
        }
    }
    for (double v: tsr_values) {
        if (std::isnan(v)) {
                    RAVELOG_WARN("Invalid value in state.\n");
            return false;
        }
    }
    _robot->SetActiveDOFValues(robot_values, checklimits);
    _tsr_robot->SetActiveDOFValues(tsr_values, checklimits);
    _tsr_chain->ApplyMimicValuesToMimicBody(tsr_values.data());
    return true;
}

bool StateValidityChecker::isValid(const ompl::base::State *state) const {
    bool valid = _state_space->satisfiesBounds(state)
           && computeFk(state, OpenRAVE::KinBody::CLA_Nothing)
           && !_env->CheckCollision(_robot)
           && !_robot->CheckSelfCollision();
    return valid;
}
