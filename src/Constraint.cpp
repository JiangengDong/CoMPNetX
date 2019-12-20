//
// Created by jiangeng on 10/12/19.
//
#include "Constraint.h"
#include "or_conversions.h"
#include "TaskSpaceRegionChain.h"

using namespace AtlasMPNet;

TSRChainConstraint::TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const OpenRAVE::RobotBasePtr &tsr_robot) :
        Constraint(robot->GetActiveDOF() + tsr_robot->GetDOF(), 7), _robot(robot), _tsr_robot(tsr_robot) {
    _dof_robot = _robot->GetActiveDOF();
    _dof_tsr = _tsr_robot->GetActiveDOF();
    _tsrjointval = new double[_dof_tsr];
}

void TSRChainConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const {
    robotFK(x);
    OpenRAVE::Transform Trobot = _robot->GetActiveManipulator()->GetEndEffectorTransform();
    OpenRAVE::Transform Ttsr = _tsr_robot->GetActiveManipulator()->GetEndEffectorTransform();
    OpenRAVE::Transform pos = Trobot.inverse() * Ttsr;
//    int sign = (pos.rot[0]>=0)*2-1; // keep the first element greater than 0
    out[0] = pos.rot[0]*pos.rot[0]-1;
    out[1] = pos.rot[1]*pos.rot[1];
    out[2] = pos.rot[2]*pos.rot[2];
    out[3] = pos.rot[3]*pos.rot[3];
    out[4] = pos.trans[0];
    out[5] = pos.trans[1];
    out[6] = pos.trans[2];
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
//    std::vector<double> q_robot(_dof_robot), q_tsr(_dof_tsr);
//    // joint values of real robot
//    for (unsigned int i = 0; i < _dof_robot; ++i) {
//        q_robot[i] = x[i];
//    }
//    _robot->SetActiveDOFValues(q_robot, 0);
//    auto Trobot = _robot->GetActiveManipulator()->GetEndEffectorTransform();
//    // joint values of virtual tsr robot
//    for (unsigned int i = 0; i < _dof_tsr; ++i) {
//        q_tsr[i] = x[i+_dof_robot];
//    }
//    _tsr_robot->SetActiveDOFValues(q_tsr);
//    auto Ttsr = _tsr_robot->GetActiveManipulator()->GetEndEffectorTransform();
//
//    std::vector<OpenRAVE::dReal> J;
//    _robot->CalculateActiveJacobian(_robot->GetActiveManipulator()->GetEndEffector()->GetIndex(), Trobot.trans, Jtemp);
//}
