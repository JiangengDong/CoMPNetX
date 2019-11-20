//
// Created by jiangeng on 10/12/19.
//
#include "Constraint.h"
#include "or_conversions.h"
#include "TaskSpaceRegionChain.h"

using namespace AtlasMPNet;

void TSRChainConstraint::jacobian2(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const {
    Eigen::VectorXd y1 = x;
    Eigen::VectorXd y2 = x;
    Eigen::VectorXd t1(3);
    Eigen::VectorXd t2(3);

    // Use a 7-point central difference stencil on each column.
    for (std::size_t j = 0; j < n_; j++) {
        const double ax = std::fabs(x[j]);
        // Make step size as small as possible while still giving usable accuracy.
//        const double h = std::sqrt(std::numeric_limits<double>::epsilon()) * (ax >= 1 ? ax : 1);
        const double h = 1e-3;

        // Can't assume y1[j]-y2[j] == 2*h because of precision errors.
        y1[j] += h;
        y2[j] -= h;
        function2(y1, t1);
        function2(y2, t2);
        const Eigen::VectorXd m1 = (t1 - t2) / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        function2(y1, t1);
        function2(y2, t2);
        const Eigen::VectorXd m2 = (t1 - t2) / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        function2(y1, t1);
        function2(y2, t2);
        const Eigen::VectorXd m3 = (t1 - t2) / (y1[j] - y2[j]);

        out.col(j) = 1.5 * m1 - 0.6 * m2 + 0.1 * m3;

        // Reset for next iteration.
        y1[j] = y2[j] = x[j];
    }
}

void TSRChainConstraint::function2(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const {
    OpenRAVE::Transform pos = robotFK(x);
    OpenRAVE::Transform pos_proj;
    _tsr_chain.GetClosestTransform(pos, _tsrjointval, pos_proj);
    out[0] = pos_proj.trans.x;
    out[1] = pos_proj.trans.y;
    out[2] = pos_proj.trans.z;
}

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

    static int local_count = 0;
    static int pre_count_ = 0;
    static OpenRAVE::Transform pre_pos;
    static OpenRAVE::Transform pre_pos_proj;
    static double pre_dist;
    if (false && count_ < 100 && count_ != pre_count_) {
        pre_count_ = count_;
        std::cout << count_
                  << "\tpre_function: " << pre_dist << std::endl
                  << "\tfunction:     " << out << std::endl
                  << "\tpre_pos:      " << pre_pos * _tsr_chain.TSRChain.back().Tw_e.inverse() << std::endl
                  << "\tpos:          " << pos * _tsr_chain.TSRChain.back().Tw_e.inverse() << std::endl
                  << "\tpre_pos_proj: " << pre_pos_proj * _tsr_chain.TSRChain.back().Tw_e.inverse() << std::endl
                  << "\tpos_proj:     " << pos_proj * _tsr_chain.TSRChain.back().Tw_e.inverse() << std::endl
                  << std::endl;
        local_count = 0;
    }
    ++local_count;
    pre_pos = pos;
    pre_pos_proj = pos_proj;
    pre_dist = dist;
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
        const double h = 1e-3;

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

    if (count_ < 200) {
        static Eigen::MatrixXd pre_out(1, 7), pre_jac(3, 7), jac(3, 7);
        static Eigen::VectorXd pre_x(7);
        static double dist, pre_dist;
        static OpenRAVE::Transform tf, pre_tf;
        dist = distance(x);
        tf = robotFK(x) * _tsr_chain.TSRChain.back().Tw_e.inverse();
        jacobian2(x, jac);
        std::cout << count_
                  << "\033[1;32m" << "\tpre_function:  " << pre_dist << "\033[0m" << std::endl
                  << "\033[1;32m" << "\tpre_config:    " << pre_x.transpose() << "\033[0m" << std::endl
                  << "\tpre_transform: " << pre_tf << std::endl
                  << "\tpre_jacobian1: " << pre_out << std::endl
                  << "\tpre_jacobian2: " << pre_jac.block(0, 0, 1, 7) << std::endl
                  << "\t               " << pre_jac.block(1, 0, 1, 7) << std::endl
                  << "\t               " << pre_jac.block(2, 0, 1, 7) << std::endl
                  << "\033[1;32m" << "\tfunction:      " << dist << std::endl
                  << "\033[1;32m" << "\tconfig:        " << x.transpose() << "\033[0m" << std::endl
                  << "\ttransform:     " << tf << std::endl
                  << "\tjacobian1:     " << out << std::endl
                  << "\tjacobian2:     " << jac.block(0, 0, 1, 7) << std::endl
                  << "\t               " << jac.block(1, 0, 1, 7) << std::endl
                  << "\t               " << jac.block(2, 0, 1, 7) << std::endl
                  << std::endl;
        pre_x = x;
        pre_out = out;
        pre_dist = dist;
        pre_tf = tf;
        pre_jac = jac;
    }
    count_++;
}

double TSRChainConstraint::distance(const Eigen::Ref<const Eigen::VectorXd> &x) const {
    Eigen::VectorXd t(getCoDimension());
    function(x, t);
    return t[0];
}
