//
// Created by jiangeng on 10/12/19.
//
#include "Constraint.h"
#include "or_conversions.h"
#include "TaskSpaceRegionChain.h"

using namespace AtlasMPNet;

TSRChainConstraint::TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const OpenRAVE::RobotBasePtr &tsr_robot) :
        Constraint(robot->GetActiveDOF() + tsr_robot->GetDOF(), 6), _robot(robot), _tsr_robot(tsr_robot) {
    _dof_robot = _robot->GetActiveDOF();
    _dof_tsr = _tsr_robot->GetActiveDOF();

    _robot_eeindex = _robot->GetActiveManipulator()->GetEndEffector()->GetIndex();
    _tsr_eeindex = _tsr_robot->GetActiveManipulator()->GetEndEffector()->GetIndex();
}

void TSRChainConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const {
    robotFK(x);
    OpenRAVE::Transform Trobot = _robot->GetActiveManipulator()->GetEndEffectorTransform();
    OpenRAVE::Transform Ttsr = _tsr_robot->GetActiveManipulator()->GetEndEffectorTransform();
    OpenRAVE::Transform Tdiff = Trobot.inverse() * Ttsr;

    out[0] = Tdiff.rot[1];
    out[1] = Tdiff.rot[2];
    out[2] = Tdiff.rot[3];
    out[3] = Trobot.trans[0] - Ttsr.trans[0];
    out[4] = Trobot.trans[1] - Ttsr.trans[1];
    out[5] = Trobot.trans[2] - Ttsr.trans[2];
}

void TSRChainConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const {
    robotFK(x);
    OpenRAVE::Transform Trobot = _robot->GetActiveManipulator()->GetEndEffectorTransform();
    OpenRAVE::Transform Ttsr = _tsr_robot->GetActiveManipulator()->GetEndEffectorTransform();

    auto Jrot_robot = out.block(0, 0, 3, _dof_robot);
    auto Jrot_tsr = out.block(0, _dof_robot, 3, _dof_tsr);
    auto Jtrans_robot = out.block(3, 0, 3, _dof_robot);
    auto Jtrans_tsr = out.block(3, _dof_robot, 3, _dof_tsr);

    std::vector<OpenRAVE::dReal> Jrobot, Jtsr;

    _robot->CalculateActiveRotationJacobian(_robot_eeindex, Trobot.rot, Jrobot);
    _tsr_robot->CalculateActiveRotationJacobian(_tsr_eeindex, Ttsr.rot, Jtsr);
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
    for (int j = 0; j < _dof_tsr; j++) {
        Jrot_tsr(0, j) = Trobot.rot[0] * Jtsr[1 * _dof_tsr + j]
                         - Trobot.rot[1] * Jtsr[0 * _dof_tsr + j]
                         - Trobot.rot[2] * Jtsr[3 * _dof_tsr + j]
                         + Trobot.rot[3] * Jtsr[2 * _dof_tsr + j];
        Jrot_tsr(1, j) = Trobot.rot[0] * Jtsr[2 * _dof_tsr + j]
                         + Trobot.rot[1] * Jtsr[3 * _dof_tsr + j]
                         - Trobot.rot[2] * Jtsr[0 * _dof_tsr + j]
                         - Trobot.rot[3] * Jtsr[1 * _dof_tsr + j];
        Jrot_tsr(2, j) = Trobot.rot[0] * Jtsr[3 * _dof_tsr + j]
                         - Trobot.rot[1] * Jtsr[2 * _dof_tsr + j]
                         + Trobot.rot[2] * Jtsr[1 * _dof_tsr + j]
                         - Trobot.rot[3] * Jtsr[0 * _dof_tsr + j];
    }

    _robot->CalculateActiveJacobian(_robot_eeindex, Trobot.trans, Jrobot);
    _tsr_robot->CalculateActiveJacobian(_tsr_eeindex, Ttsr.trans, Jtsr);
    for (int i = 0; i < 3; i++) {
        unsigned int j;
        for (j = 0; j < _dof_robot; j++) {
            Jtrans_robot(i, j) = Jrobot[i * _dof_robot + j];
        }
        for (j = 0; j < _dof_tsr; j++) {
            Jtrans_tsr(i, j) = -Jtsr[i * _dof_tsr + j];
        }
    }
}

bool AtlasMPNet::TSRChainConstraint::project(Eigen::Ref<Eigen::VectorXd> x) const {
    // Newton's method
    unsigned int iter = 0;
    double norm = 0;
    Eigen::VectorXd f(getCoDimension());
    Eigen::MatrixXd j(getCoDimension(), n_);

    double old_norm = 0;
    double stepsize;
    Eigen::VectorXd old_x(getAmbientDimension());
    Eigen::VectorXd direction(getAmbientDimension());

    const double squaredTolerance = tolerance_ * tolerance_;

    function(x, f);
    norm = f.squaredNorm();
    while (norm > squaredTolerance && iter++ < maxIterations_) {
        jacobian(x, j);
        direction = j.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f);
        stepsize = 2.0;
        old_x = x;
        old_norm = norm;
        do {
            stepsize /= 2;
            x = old_x - stepsize * direction;
            function(x, f);
            norm = f.squaredNorm();
        } while (norm > old_norm && norm > squaredTolerance && stepsize > 1e-8);
    }

    return norm < squaredTolerance;
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
