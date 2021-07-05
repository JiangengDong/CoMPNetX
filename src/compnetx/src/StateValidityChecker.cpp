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

#include "StateValidityChecker.h"

#include <boost/make_shared.hpp>
#include <ompl/base/StateValidityChecker.h>
#include <openrave/openrave.h>
#include <utility>

#include "TaskSpaceRegionChain.h"

using namespace CoMPNetX;

StateValidityChecker::StateValidityChecker(const ompl::base::SpaceInformationPtr &si,
                                           const OpenRAVE::RobotBasePtr &robot,
                                           const std::vector<TaskSpaceRegionChain::Ptr> &tsr_chains) : ompl::base::StateValidityChecker(si),
                                                                                                       _state_space(si->getStateSpace()),
                                                                                                       _env(robot->GetEnv()),
                                                                                                       _robot(robot),
                                                                                                       _robot_dof(robot->GetActiveDOF()),
                                                                                                       _tsr_chains(tsr_chains),
                                                                                                       _num_tsr_chains(tsr_chains.size()) {
    for (const auto &tsr_chain : _tsr_chains) {
        _tsr_dofs.emplace_back(tsr_chain->GetNumDOF());
    }
}

bool StateValidityChecker::computeFk(const ompl::base::State *state, uint32_t checklimits) const {
    std::vector<double> values;
    _state_space->copyToReals(values, state);
    // check state value
    for (double v : values) {
        if (std::isnan(v)) {
            RAVELOG_WARN("Invalid value in state.\n");
            return false;
        }
    }
    // apply robot joint values
    int offset = 0;
    std::vector<double> robot_values(values.begin() + offset, values.begin() + offset + _robot_dof);
    _robot->SetActiveDOFValues(robot_values, checklimits);
    offset += _robot_dof;
    // apply tsr joint values
    for (int i = 0; i < _num_tsr_chains; i++) {
        std::vector<double> tsr_values(values.begin() + offset, values.begin() + offset + _tsr_dofs[i]);
        _tsr_chains[i]->SetActiveDOFValues(tsr_values);
        _tsr_chains[i]->ApplyMimicValuesToMimicBody(tsr_values);
        offset += _tsr_dofs[i];
    }
    return true;
}

bool StateValidityChecker::isValid(const ompl::base::State *state) const {
    bool valid = _state_space->satisfiesBounds(state)                //
                 && computeFk(state, OpenRAVE::KinBody::CLA_Nothing) //
                 && !_env->CheckCollision(_robot)                    //
                 && !_robot->CheckSelfCollision();
    return valid;
}
