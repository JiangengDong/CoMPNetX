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
    OpenRAVE::Transform pos = robotFK(x);
    out[0] = pos.trans.x;
    out[1] = pos.trans.y;
    out[2] = pos.trans.z;
    out[3] = pos.rot.x;
    out[4] = pos.rot.y;
    out[5] = pos.rot.z;
    out[6] = pos.rot.w;
}

OpenRAVE::Transform TSRChainConstraint::robotFK(const Eigen::Ref<const Eigen::VectorXd> &x) const {
    std::vector<double> q_robot(_dof_robot), q_tsr(_dof_tsr);
    // joint values of real robot
    for (unsigned int i = 0; i < _dof_robot; ++i) {
        q_robot[i] = x[i];
    }
    _robot->SetActiveDOFValues(q_robot, 0);
    auto Trobot = _robot->GetActiveManipulator()->GetEndEffectorTransform();
    // joint values of virtual tsr robot
    for (unsigned int i = 0; i < _dof_tsr; ++i) {
        q_tsr[i] = x[i+_dof_robot];
    }
    _tsr_robot->SetActiveDOFValues(q_tsr);
    auto Ttsr = _tsr_robot->GetActiveManipulator()->GetEndEffectorTransform();
    return Trobot*Ttsr.inverse(); // TODO: I think we can use the diff of (x, y, z, r, p, y) here so that we can calculate jacobian easily
}

void TSRChainConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const {
    Eigen::VectorXd y1 = x;
    Eigen::VectorXd y2 = x;
    Eigen::VectorXd t1(getCoDimension());
    Eigen::VectorXd t2(getCoDimension());

    // Use a 7-point central difference stencil on each column.
    for (std::size_t j = 0; j < n_; j++) {
        const double ax = std::fabs(x[j]);
        // Make step size as small as possible while still giving usable accuracy.
//        const double h = std::sqrt(std::numeric_limits<double>::epsilon()) * (ax >= 1 ? ax : 1);
        const double h = 1e-5;

        // Can't assume y1[j]-y2[j] == 2*h because of precision errors.
        y1[j] += h;
        y2[j] -= h;
        function(y1, t1);
        function(y2, t2);
        const Eigen::VectorXd m1 = (t1 - t2) / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        function(y1, t1);
        function(y2, t2);
        const Eigen::VectorXd m2 = (t1 - t2) / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        function(y1, t1);
        function(y2, t2);
        const Eigen::VectorXd m3 = (t1 - t2) / (y1[j] - y2[j]);

        out.col(j) = (1.5 * m1 - 0.6 * m2 + 0.1 * m3);

        // Reset for next iteration.
        y1[j] = y2[j] = x[j];
    }
}
