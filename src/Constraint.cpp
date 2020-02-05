//
// Created by jiangeng on 10/12/19.
//
#include "Constraint.h"

#include <Eigen/Dense>
#include <openrave/openrave.h>

#include "TaskSpaceRegionChain.h"

using namespace AtlasMPNet;

TSRChainConstraint::TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const std::vector<TaskSpaceRegionChain::Ptr> &tsr_chains) :
        Constraint([&robot, &tsr_chains] {
            unsigned int dof_tsrs = robot->GetActiveDOF();
            for (const auto &tsr_chain:tsr_chains)
                dof_tsrs += tsr_chain->GetNumDOF();
            return dof_tsrs;
        }(), 6 * tsr_chains.size()),
        _robot(robot),
        _dof_robot(robot->GetActiveDOF()),
        _tsr_chains(tsr_chains),
        _num_tsr_chains(tsr_chains.size()) {
    std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manipulators = robot->GetManipulators();
    for (const auto &tsr_chain: _tsr_chains) {
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
    int offset = _dof_robot;
    for (int i = 0; i < _num_tsr_chains; i++) {
        OpenRAVE::Transform &Trobot = Tpairs[i].first;
        OpenRAVE::Transform &Ttsr = Tpairs[i].second;
        std::vector<OpenRAVE::dReal> Jrobot, Jtsr;

        auto Jrot_robot = out.block(6 * i, 0, 3, _dof_robot);
        auto Jrot_tsr = out.block(6 * i, offset, 3, _dof_tsrs[i]);
        auto Jtrans_robot = out.block(6 * i + 3, 0, 3, _dof_robot);
        auto Jtrans_tsr = out.block(6 * i + 3, offset, 3, _dof_tsrs[i]);
        offset += _dof_tsrs[i];

        // rotation Jacobian
        _robot->CalculateActiveRotationJacobian(_robot_eeindices[i], Trobot.rot, Jrobot);
        _tsr_chains[i]->CalculateActiveRotationJacobian(Ttsr.rot, Jtsr);
        for (int j = 0; j < _dof_robot; j++) {
            Jrot_robot(0, j) = Jrobot[0 * _dof_robot + j] * Ttsr.rot[1]
                               - Jrobot[1 * _dof_robot + j] * Ttsr.rot[0]
                               - Jrobot[2 * _dof_robot + j] * Ttsr.rot[3]
                               + Jrobot[3 * _dof_robot + j] * Ttsr.rot[2];
            Jrot_robot(1, j) = Jrobot[0 * _dof_robot + j] * Ttsr.rot[2]
                               + Jrobot[1 * _dof_robot + j] * Ttsr.rot[3]
                               - Jrobot[2 * _dof_robot + j] * Ttsr.rot[0]
                               - Jrobot[3 * _dof_robot + j] * Ttsr.rot[1];
            Jrot_robot(2, j) = Jrobot[0 * _dof_robot + j] * Ttsr.rot[3]
                               - Jrobot[1 * _dof_robot + j] * Ttsr.rot[2]
                               + Jrobot[2 * _dof_robot + j] * Ttsr.rot[1]
                               - Jrobot[3 * _dof_robot + j] * Ttsr.rot[0];
        }
        for (int j = 0; j < _dof_tsrs[i]; j++) {
            Jrot_tsr(0, j) = Trobot.rot[0] * Jtsr[1 * _dof_tsrs[i] + j]
                             - Trobot.rot[1] * Jtsr[0 * _dof_tsrs[i] + j]
                             - Trobot.rot[2] * Jtsr[3 * _dof_tsrs[i] + j]
                             + Trobot.rot[3] * Jtsr[2 * _dof_tsrs[i] + j];
            Jrot_tsr(1, j) = Trobot.rot[0] * Jtsr[2 * _dof_tsrs[i] + j]
                             + Trobot.rot[1] * Jtsr[3 * _dof_tsrs[i] + j]
                             - Trobot.rot[2] * Jtsr[0 * _dof_tsrs[i] + j]
                             - Trobot.rot[3] * Jtsr[1 * _dof_tsrs[i] + j];
            Jrot_tsr(2, j) = Trobot.rot[0] * Jtsr[3 * _dof_tsrs[i] + j]
                             - Trobot.rot[1] * Jtsr[2 * _dof_tsrs[i] + j]
                             + Trobot.rot[2] * Jtsr[1 * _dof_tsrs[i] + j]
                             - Trobot.rot[3] * Jtsr[0 * _dof_tsrs[i] + j];
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
    Tpairs.clear();
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
        if (_dof_tsrs[i] == 0)   // Point TSR
        {
            continue;
        }
        q.resize(_dof_tsrs[i]);
        for (int j = 0; j < _dof_tsrs[i]; ++j) {
            q[j] = x[j + offset];
        }
        _tsr_chains[i]->SetActiveDOFValues(q);
        Tpairs.emplace_back(_robot_manipulators[i]->GetEndEffectorTransform(), _tsr_chains[i]->GetEndEffectorTransform());
        offset += _dof_tsrs[i];
    }
}
