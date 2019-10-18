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

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <utility>
#include <openrave/openrave.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>

using namespace AtlasMPNet;

StateValidityChecker::StateValidityChecker(
        const ompl::base::SpaceInformationPtr &si,
        const OpenRAVE::RobotBasePtr& robot,
        std::vector<int> indices) :
        ompl::base::StateValidityChecker(si),
        _num_dof(si->getStateDimension()),
        _stateSpace(si->getStateSpace().get()),
        _env(robot->GetEnv()),
        _robot(robot),
        _indices(std::move(indices)),
        _numCollisionChecks(0),
        _totalCollisionTime(0.0) {
}

bool StateValidityChecker::computeFk(const ompl::base::State *state, uint32_t checklimits) const {
    auto const *real_state = state->as<ompl::base::RealVectorStateSpace::StateType>();

    std::vector<double> values(real_state->values, real_state->values + _num_dof);

    BOOST_FOREACH(double v, values) {
        if (std::isnan(v)) {
            RAVELOG_ERROR("Invalid value in state.\n");
            return false;
        }
    }

    _robot->SetDOFValues(values, checklimits, _indices);
    return true;
}

bool StateValidityChecker::isValid(const ompl::base::State *state) const {
    boost::chrono::steady_clock::time_point const tic = boost::chrono::steady_clock::now();

    bool collided = !computeFk(state, OpenRAVE::KinBody::CLA_Nothing);

    if (!collided) {
        collided = collided || _env->CheckCollision(_robot) || _robot->CheckSelfCollision();

        boost::chrono::steady_clock::time_point const toc = boost::chrono::steady_clock::now();
        _totalCollisionTime += boost::chrono::duration_cast<boost::chrono::duration<double>>(toc - tic).count();
        _numCollisionChecks++;
    }

    return !collided;
}
