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
#include "Constraint.h"

#include <Eigen/Dense>
#include <ompl/base/Constraint.h>
#include <openrave/openrave.h>

#include "TaskSpaceRegionChain.h"

using namespace CoMPNetX;

TSRChainConstraint::TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const std::vector<TaskSpaceRegionChain::Ptr> &tsr_chains) : Constraint([&robot, &tsr_chains] {
                                                                                                                                            unsigned int dof = robot->GetActiveDOF();
                                                                                                                                            for (const auto &tsr_chain : tsr_chains)
                                                                                                                                                dof += tsr_chain->GetNumDOF();
                                                                                                                                            return dof;
                                                                                                                                        }(),
                                                                                                                                                   6 * tsr_chains.size()),
                                                                                                                                        _robot(robot), _dof_robot(robot->GetActiveDOF()), _tsr_chains(tsr_chains), _num_tsr_chains(tsr_chains.size()) {
    std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manipulators = robot->GetManipulators();
    for (const auto &tsr_chain : _tsr_chains) {
        _dof_tsrs.emplace_back(tsr_chain->GetNumDOF());
        _robot_manipulators.emplace_back(manipulators[tsr_chain->GetManipInd()]);
        _robot_eeindices.emplace_back(manipulators[tsr_chain->GetManipInd()]->GetEndEffector()->GetIndex());
    }
}

void TSRChainConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const {
    TransformPairVector Tpairs;
    robotFK(x, Tpairs);
    for (int i = 0; i < _num_tsr_chains; i++) {
        OpenRAVE::Transform &Trobot = Tpairs[i].first;
        OpenRAVE::Transform &Ttsr = Tpairs[i].second;
        OpenRAVE::Transform Tdiff = Trobot.inverse() * Ttsr;
        out[i * 6 + 0] = Tdiff.rot[1];
        out[i * 6 + 1] = Tdiff.rot[2];
        out[i * 6 + 2] = Tdiff.rot[3];
        out[i * 6 + 3] = Trobot.trans[0] - Ttsr.trans[0];
        out[i * 6 + 4] = Trobot.trans[1] - Ttsr.trans[1];
        out[i * 6 + 5] = Trobot.trans[2] - Ttsr.trans[2];
    }
}

void TSRChainConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const {
    TransformPairVector Tpairs;
    robotFK(x, Tpairs);
    out.setZero();
    int offset = _dof_robot;
    for (int i = 0; i < _num_tsr_chains; i++) {
        OpenRAVE::Transform &Trobot = Tpairs[i].first;
        OpenRAVE::Transform &Ttsr = Tpairs[i].second;
        std::vector<OpenRAVE::dReal> Jrobot, Jtsr;

        auto Jrot_robot = out.block(6 * i, 0, 3, _dof_robot);
        auto Jtrans_robot = out.block(6 * i + 3, 0, 3, _dof_robot);
        auto Jrot_tsr = out.block(6 * i, offset, 3, _dof_tsrs[i]);
        auto Jtrans_tsr = out.block(6 * i + 3, offset, 3, _dof_tsrs[i]);
        offset += _dof_tsrs[i];

        // rotation Jacobian
        _robot->CalculateActiveRotationJacobian(_robot_eeindices[i], Trobot.rot, Jrobot);
        _tsr_chains[i]->CalculateActiveRotationJacobian(Ttsr.rot, Jtsr);
        for (int j = 0; j < _dof_robot; j++) {
            Jrot_robot(0, j) = Jrobot[0 * _dof_robot + j] * Ttsr.rot[1] - Jrobot[1 * _dof_robot + j] * Ttsr.rot[0] - Jrobot[2 * _dof_robot + j] * Ttsr.rot[3] + Jrobot[3 * _dof_robot + j] * Ttsr.rot[2];
            Jrot_robot(1, j) = Jrobot[0 * _dof_robot + j] * Ttsr.rot[2] + Jrobot[1 * _dof_robot + j] * Ttsr.rot[3] - Jrobot[2 * _dof_robot + j] * Ttsr.rot[0] - Jrobot[3 * _dof_robot + j] * Ttsr.rot[1];
            Jrot_robot(2, j) = Jrobot[0 * _dof_robot + j] * Ttsr.rot[3] - Jrobot[1 * _dof_robot + j] * Ttsr.rot[2] + Jrobot[2 * _dof_robot + j] * Ttsr.rot[1] - Jrobot[3 * _dof_robot + j] * Ttsr.rot[0];
        }

        for (int j = 0; j < _dof_tsrs[i]; j++) {
            Jrot_tsr(0, j) = Trobot.rot[0] * Jtsr[1 * _dof_tsrs[i] + j] - Trobot.rot[1] * Jtsr[0 * _dof_tsrs[i] + j] - Trobot.rot[2] * Jtsr[3 * _dof_tsrs[i] + j] + Trobot.rot[3] * Jtsr[2 * _dof_tsrs[i] + j];
            Jrot_tsr(1, j) = Trobot.rot[0] * Jtsr[2 * _dof_tsrs[i] + j] + Trobot.rot[1] * Jtsr[3 * _dof_tsrs[i] + j] - Trobot.rot[2] * Jtsr[0 * _dof_tsrs[i] + j] - Trobot.rot[3] * Jtsr[1 * _dof_tsrs[i] + j];
            Jrot_tsr(2, j) = Trobot.rot[0] * Jtsr[3 * _dof_tsrs[i] + j] - Trobot.rot[1] * Jtsr[2 * _dof_tsrs[i] + j] + Trobot.rot[2] * Jtsr[1 * _dof_tsrs[i] + j] - Trobot.rot[3] * Jtsr[0 * _dof_tsrs[i] + j];
        }
        // Translation Jacobian
        _robot->CalculateActiveJacobian(_robot_eeindices[i], Trobot.trans, Jrobot);
        _tsr_chains[i]->CalculateActiveJacobian(Ttsr.trans, Jtsr);
        for (int j = 0; j < _dof_robot; j++) {
            Jtrans_robot(0, j) = Jrobot[0 * _dof_robot + j];
            Jtrans_robot(1, j) = Jrobot[1 * _dof_robot + j];
            Jtrans_robot(2, j) = Jrobot[2 * _dof_robot + j];
        }
        for (int j = 0; j < _dof_tsrs[i]; j++) {
            Jtrans_tsr(0, j) = -Jtsr[0 * _dof_tsrs[i] + j];
            Jtrans_tsr(1, j) = -Jtsr[1 * _dof_tsrs[i] + j];
            Jtrans_tsr(2, j) = -Jtsr[2 * _dof_tsrs[i] + j];
        }
    }
}

void TSRChainConstraint::robotFK(const Eigen::Ref<const Eigen::VectorXd> &x, TransformPairVector &Tpairs) const {
    std::vector<double> q;
    int offset = 0;
    // joint values of real robot
    q.resize(_dof_robot);
    for (int i = 0; i < _dof_robot; ++i) {
        q[i] = x[i + offset];
    }
    _robot->SetActiveDOFValues(q, 0);
    offset += _dof_robot;
    // joint values of virtual tsr robot
    for (int i = 0; i < _num_tsr_chains; i++) {
        q.resize(_dof_tsrs[i]);
        for (int j = 0; j < _dof_tsrs[i]; ++j) {
            q[j] = x[j + offset];
        }
        _tsr_chains[i]->SetActiveDOFValues(q);
        offset += _dof_tsrs[i];
    }
    Tpairs.clear();
    for (int i = 0; i < _num_tsr_chains; i++)
        Tpairs.emplace_back(_robot_manipulators[i]->GetEndEffectorTransform(), _tsr_chains[i]->GetEndEffectorTransform());
}
