//
// Created by jiangeng on 10/12/19.
//
#include "Constraint.h"
#include "or_conversions.h"
#include "TaskSpaceRegionChain.h"

using namespace AtlasMPNet;

TSRChainConstraint::TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const TaskSpaceRegionChain &tsr_chain) :
        Constraint(robot->GetActiveDOF(), 1) {
    _tsr_chain = tsr_chain;
    _robot = robot;
    _tsr_chain.Initialize(_robot->GetEnv());
    OpenRAVE::RobotBasePtr tsr_robot;
    _tsr_chain.RobotizeTSRChain(_robot->GetEnv(), tsr_robot);
    _tsrjointval = new double[_tsr_chain.GetNumDOF()];
}

void TSRChainConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const {
    // TODO: the projection of atlas does not converge, but why?
    OpenRAVE::Transform pos = robotFK(x);
    OpenRAVE::Transform pos_proj;
    double dist = _tsr_chain.GetClosestTransform(pos, _tsrjointval, pos_proj);
    out[0] = dist;
}

OpenRAVE::Transform TSRChainConstraint::robotFK(const Eigen::Ref<const Eigen::VectorXd> &x) const {
    std::vector<double> qpos(getAmbientDimension());
    for (unsigned int i = 0; i < getAmbientDimension(); ++i) {
        qpos[i] = x[i];
    }
    _robot->SetActiveDOFValues(qpos, 0);
    return _robot->GetActiveManipulator()->GetEndEffectorTransform();
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

double TSRChainConstraint::distance(const Eigen::Ref<const Eigen::VectorXd> &x) const {
    Eigen::VectorXd t(getCoDimension());
    function(x, t);
    return t[0];
}

void TSRChainConstraint::printHessian(const Eigen::Ref<const Eigen::VectorXd> &x) const {
    Eigen::VectorXd y1 = x;
    Eigen::VectorXd y2 = x;
    Eigen::MatrixXd t1(1, getAmbientDimension());
    Eigen::MatrixXd t2(1, getAmbientDimension());
    Eigen::MatrixXd H(getAmbientDimension(), getAmbientDimension());

    // Use a 7-point central difference stencil on each column.
    for (std::size_t j = 0; j < n_; j++) {
        const double ax = std::fabs(x[j]);
        // Make step size as small as possible while still giving usable accuracy.
//        const double h = std::sqrt(std::numeric_limits<double>::epsilon()) * (ax >= 1 ? ax : 1);
        const double h = 1e-5;

        // Can't assume y1[j]-y2[j] == 2*h because of precision errors.
        y1[j] += h;
        y2[j] -= h;
        jacobian(y1, t1);
        jacobian(y2, t2);
        const Eigen::VectorXd m1 = (t1 - t2).transpose() / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        jacobian(y1, t1);
        jacobian(y2, t2);
        const Eigen::VectorXd m2 = (t1 - t2).transpose() / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        jacobian(y1, t1);
        jacobian(y2, t2);
        const Eigen::VectorXd m3 = (t1 - t2).transpose() / (y1[j] - y2[j]);

        H.col(j) = (1.5 * m1 - 0.6 * m2 + 0.1 * m3);

        // Reset for next iteration.
        y1[j] = y2[j] = x[j];
    }
    std::cout << "Hessian: " << std::endl << H << std::endl << std::endl;
}
