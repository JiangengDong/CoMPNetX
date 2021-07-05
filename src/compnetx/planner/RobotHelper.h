/***********************************************************************

Copyright (c) 2020, University of California, San Diego
All rights reserved.

Author: Jiangeng Dong <jid103@ucsd.edu>

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
#ifndef COMPNETX_ROBOTHELPER_H
#define COMPNETX_ROBOTHELPER_H

#include <Eigen/Dense>
#include <openrave/openrave.h>

namespace CoMPNetX {
class RobotHelper {
public:
    RobotHelper(OpenRAVE::RobotBasePtr robot, OpenRAVE::RobotBase::ManipulatorPtr manip) : robot_(robot),
                                                                                           manip_(manip),
                                                                                           numdof(robot->GetActiveDOF()) {
        robot_->GetActiveDOFLimits(_lower_limits, _upper_limits);
        eeIndex = manip_->GetEndEffector()->GetIndex();
    };

    ~RobotHelper(){};

    void EnforceBound(std::vector<double> &TSRJointVals) const {
        for (unsigned int i = 0; i < numdof; i++) {
            if (TSRJointVals[i] > _upper_limits[i])
                TSRJointVals[i] = _upper_limits[i];
            else if (TSRJointVals[i] < _lower_limits[i])
                TSRJointVals[i] = _lower_limits[i];
        }
    }

    OpenRAVE::Transform ForwardKinematics(std::vector<double> &TSRJointVals) const {
        robot_->SetActiveDOFValues(TSRJointVals, 0);
        return manip_->GetEndEffectorTransform();
    }

    double TransformDifference(const OpenRAVE::Transform &T0_s, const OpenRAVE::Transform &T0_g, OpenRAVE::Transform &Tdiff) const {
        Tdiff = T0_s.inverse() * T0_g;
        return Tdiff.trans.lengthsqr3() + Tdiff.rot.y * Tdiff.rot.y + Tdiff.rot.z * Tdiff.rot.z + Tdiff.rot.w * Tdiff.rot.w;
    }

    OpenRAVE::Transform GetEndEffectorTransform() const {
        return manip_->GetEndEffectorTransform();
    }

    void GetJacobian(const OpenRAVE::Transform &T0_s, const OpenRAVE::Transform &T0_closest, Eigen::Ref<Eigen::MatrixXd> J) const;

    double GetClosestTransform(const OpenRAVE::Transform &T0_s, std::vector<OpenRAVE::dReal> &TSRJointVals, OpenRAVE::Transform &T0_closest) const;

    OpenRAVE::RobotBasePtr robot_;
    OpenRAVE::RobotBase::ManipulatorPtr manip_;

private:
    unsigned int numdof;
    std::vector<double> _lower_limits, _upper_limits;
    int eeIndex;
};
} // namespace CoMPNetX

#endif // COMPNETX_ROBOTHELPER_H
