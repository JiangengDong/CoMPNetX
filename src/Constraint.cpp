//
// Created by jiangeng on 10/12/19.
//
#include "Constraint.h"
#include "or_conversions.h"
#include "TaskSpaceRegionChain.h"

using namespace AtlasMPNet;

TSRChainConstraint::TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const OpenRAVE::RobotBasePtr &tsr_robot) :
        Constraint(robot->GetActiveDOF() + tsr_robot->GetDOF(), 7), _robot(robot), _tsr_robot(tsr_robot) {
    // TODO: jacobian is forced to be surjective
    _dof_robot = _robot->GetActiveDOF();
    _dof_tsr = _tsr_robot->GetActiveDOF();

    _robot_eeindex = _robot->GetActiveManipulator()->GetEndEffector()->GetIndex();
    _tsr_eeindex = _tsr_robot->GetActiveManipulator()->GetEndEffector()->GetIndex();
    // TODO: manifold dim is just a protected member, so we can simply change it
}

void TSRChainConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const {
    robotFK(x);
    OpenRAVE::Transform Trobot = _robot->GetActiveManipulator()->GetEndEffectorTransform();
    OpenRAVE::Transform Ttsr = _tsr_robot->GetActiveManipulator()->GetEndEffectorTransform();
    OpenRAVE::Transform Tdiff = Trobot.inverse() * Ttsr;

    out[0] = Tdiff.rot[0] * Tdiff.rot[0] - 1;
    out[1] = Tdiff.rot[1] * Tdiff.rot[1];
    out[2] = Tdiff.rot[2] * Tdiff.rot[2];
    out[3] = Tdiff.rot[3] * Tdiff.rot[3];
    out[4] = Trobot.trans[0] - Ttsr.trans[0];
    out[5] = Trobot.trans[1] - Ttsr.trans[1];
    out[6] = Trobot.trans[2] - Ttsr.trans[2];
}

void TSRChainConstraint::robotFK(const Eigen::Ref<const Eigen::VectorXd> &x) const {
    std::vector<double> q_robot(_dof_robot), q_tsr(_dof_tsr);
    // joint values of real robot
    for (unsigned int i = 0; i < _dof_robot; ++i) {
        q_robot[i] = x[i];
    }
    _robot->SetActiveDOFValues(q_robot, 0);
    // joint values of virtual tsr robot
    for (unsigned int i = 0; i < _dof_tsr; ++i) {
        q_tsr[i] = x[i + _dof_robot];
    }
    _tsr_robot->SetActiveDOFValues(q_tsr);
}

//void TSRChainConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const {
//    robotFK(x);
//    OpenRAVE::Transform Trobot = _robot->GetActiveManipulator()->GetEndEffectorTransform();
//    OpenRAVE::Transform Ttsr = _tsr_robot->GetActiveManipulator()->GetEndEffectorTransform();
//    OpenRAVE::Transform Tdiff = Trobot.inverse() * Ttsr;
//
//    auto Jrot_robot = out.block(0, 0, 4, _dof_robot);
//    auto Jrot_tsr = out.block(0, _dof_robot, 4, _dof_tsr);
//    auto Jtrans_robot = out.block(4, 0, 3, _dof_robot);
//    auto Jtrans_tsr = out.block(4, _dof_robot, 3, _dof_tsr);
//
//    std::vector<OpenRAVE::dReal> Jrobot, Jtsr;
//
//    _robot->CalculateActiveRotationJacobian(_robot_eeindex, Trobot.rot, Jrobot);
//    _tsr_robot->CalculateActiveRotationJacobian(_tsr_eeindex, Ttsr.rot, Jtsr);
//    for (int j = 0; j < _dof_robot; j++) {
//        Jrot_robot(0, j) = 2 * Tdiff.rot[0] * (Jrobot[0 * _dof_robot + j] * Ttsr.rot[0]
//                                               + Jrobot[1 * _dof_robot + j] * Ttsr.rot[1]
//                                               + Jrobot[2 * _dof_robot + j] * Ttsr.rot[2]
//                                               + Jrobot[3 * _dof_robot + j] * Ttsr.rot[3]);
//        Jrot_robot(1, j) = 2 * Tdiff.rot[1] * (Jrobot[0 * _dof_robot + j] * Ttsr.rot[1]
//                                               - Jrobot[1 * _dof_robot + j] * Ttsr.rot[0]
//                                               - Jrobot[2 * _dof_robot + j] * Ttsr.rot[3]
//                                               + Jrobot[3 * _dof_robot + j] * Ttsr.rot[2]);
//        Jrot_robot(2, j) = 2 * Tdiff.rot[2] * (Jrobot[0 * _dof_robot + j] * Ttsr.rot[2]
//                                               + Jrobot[1 * _dof_robot + j] * Ttsr.rot[3]
//                                               - Jrobot[2 * _dof_robot + j] * Ttsr.rot[0]
//                                               - Jrobot[3 * _dof_robot + j] * Ttsr.rot[1]);
//        Jrot_robot(3, j) = 2 * Tdiff.rot[3] * (Jrobot[0 * _dof_robot + j] * Ttsr.rot[3]
//                                               - Jrobot[1 * _dof_robot + j] * Ttsr.rot[2]
//                                               + Jrobot[2 * _dof_robot + j] * Ttsr.rot[1]
//                                               - Jrobot[3 * _dof_robot + j] * Ttsr.rot[0]);
//    }
//    for (int j = 0; j < _dof_tsr; j++) {
//        Jrot_tsr(0, j) = 2 * Tdiff.rot[0] * (Trobot.rot[0] * Jtsr[0 * _dof_tsr + j]
//                                             + Trobot.rot[1] * Jtsr[1 * _dof_tsr + j]
//                                             + Trobot.rot[2] * Jtsr[2 * _dof_tsr + j]
//                                             + Trobot.rot[3] * Jtsr[3 * _dof_tsr + j]);
//        Jrot_tsr(1, j) = 2 * Tdiff.rot[1] * (Trobot.rot[0] * Jtsr[1 * _dof_tsr + j]
//                                             - Trobot.rot[1] * Jtsr[0 * _dof_tsr + j]
//                                             - Trobot.rot[2] * Jtsr[3 * _dof_tsr + j]
//                                             + Trobot.rot[3] * Jtsr[2 * _dof_tsr + j]);
//        Jrot_tsr(2, j) = 2 * Tdiff.rot[2] * (Trobot.rot[0] * Jtsr[2 * _dof_tsr + j]
//                                             + Trobot.rot[1] * Jtsr[3 * _dof_tsr + j]
//                                             - Trobot.rot[2] * Jtsr[0 * _dof_tsr + j]
//                                             - Trobot.rot[3] * Jtsr[1 * _dof_tsr + j]);
//        Jrot_tsr(3, j) = 2 * Tdiff.rot[3] * (Trobot.rot[0] * Jtsr[3 * _dof_tsr + j]
//                                             - Trobot.rot[1] * Jtsr[2 * _dof_tsr + j]
//                                             + Trobot.rot[2] * Jtsr[1 * _dof_tsr + j]
//                                             - Trobot.rot[3] * Jtsr[0 * _dof_tsr + j]);
//    }
//
//    _robot->CalculateActiveJacobian(_robot_eeindex, Trobot.trans, Jrobot);
//    _tsr_robot->CalculateActiveJacobian(_tsr_eeindex, Ttsr.trans, Jtsr);
//    for (int i = 0; i < 3; i++) {
//        unsigned int j;
//        for (j = 0; j < _dof_robot; j++) {
//            Jtrans_robot(i, j) = Jrobot[i * _dof_robot + j];
//        }
//        for (j = 0; j < _dof_tsr; j++) {
//            Jtrans_tsr(i, j) = -Jtsr[i * _dof_tsr + j];
//        }
//    }
//    std::cout << "!!!Jacobian: " << std::endl;
//    std::cout << out << std::endl;
//    Eigen::MatrixXd out_old(7, 11);
//    Constraint::jacobian(x, out_old);
//    std::cout << "!!!Old Jacobian: " << std::endl;
//    std::cout << out_old << std::endl;
//    std::cout << "!!!Diff: " << std::endl;
//    std::cout << out_old - out << std::endl;
//}
