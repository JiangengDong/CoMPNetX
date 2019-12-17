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

#ifndef ATLASMPNET_STATEVALIDITYCHECKER_H
#define ATLASMPNET_STATEVALIDITYCHECKER_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>
#include <openrave/openrave.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>

namespace AtlasMPNet {
    /*! \brief The Wrapper of the OpenRAVE collision checker for use in OMPL
     *
     * Use the OpenRAVE collision checker (usually the fcl) to check it a state is valid.
     */
    class StateValidityChecker : public ompl::base::StateValidityChecker {
    public:
        typedef std::shared_ptr<StateValidityChecker> Ptr;
        StateValidityChecker(const ompl::base::SpaceInformationPtr &si, OpenRAVE::RobotBasePtr robot,
                             OpenRAVE::RobotBasePtr tsr_robot);

        bool computeFk(const ompl::base::State *state, uint32_t checklimits) const;

        bool isValid(const ompl::base::State *state) const override;

    private:
        OpenRAVE::RobotBasePtr _robot;
        OpenRAVE::RobotBasePtr _tsr_robot;
        OpenRAVE::EnvironmentBasePtr _env;
        const std::size_t _robot_dof;
        const std::size_t _tsr_dof;
        mutable int _numCollisionChecks;
        mutable double _totalCollisionTime;
        // temporary variables
        std::vector<double> _robot_values;
        std::vector<double> _tsr_values;
    };

} // namespace AtlasMPNet


#endif //ATLASMPNET_STATEVALIDITYCHECKER_H
