//
// Created by jiangeng on 10/12/19.
//
#include "Constraint.h"
#include "or_conversions.h"

using namespace AtlasMPNet;

TSRChainConstraint::TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const TSRChain::Ptr &tsr_chain) :
        Constraint(robot->GetActiveDOF(), 1) {
    _tsr_chain = tsr_chain;
    _robot = robot;
}

void TSRChainConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const {
    // TODO: the projection of atlas does not converge, but why?
    auto pos = robotFK(x);
    auto dist = _tsr_chain->distance(pos); // TODO: need to project R6 to Rk, check if this is permitted
    out[0] = dist.squaredNorm();

    static int pre_count = 0;
    if (count_ >= 0 && count_ < 403) {
        if(pre_count < count_) {
            std::cout << count_ << " " << out << std::endl;
        }
        pre_count = count_;
    }
}

Eigen::Affine3d TSRChainConstraint::robotFK(const Eigen::Ref<const Eigen::VectorXd> &x) const {
    std::vector<double> qpos(getAmbientDimension());
    for (unsigned int i = 0; i < getAmbientDimension(); ++i) {
        qpos[i] = x[i];
    }
    _robot->SetActiveDOFValues(qpos, 0);

    auto manip = _robot->GetActiveManipulator();
    auto tf = manip->GetEndEffectorTransform();
    auto tf_eigen = toEigen(tf);

//    static int local_count = 0;
//    if (local_count < 5) {
//        OpenRAVE::TransformMatrix or_matrix(tf);
//        std::cout << "tf " << std::endl << or_matrix << std::endl;
//        std::cout << "tf_eigen " << std::endl << tf_eigen.matrix() << std::endl;
//        local_count ++;
//    }

    return tf_eigen;
}

void TSRChainConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const {
    Constraint::jacobian(x, out);
    count_++;
//    if (count_ >= 0 && count_ < 203) {
//        Eigen::VectorXd t(getCoDimension());
//        function(x, t);
//        Eigen::VectorXd dist(6);
//        dist = _tsr_chain->distance(robotFK(x));
//        std::cout << out << std::endl << x.transpose() << std::endl << t.transpose() << std::endl << dist.transpose() << std::endl;
//    }
}

double TSRChainConstraint::distance(const Eigen::Ref<const Eigen::VectorXd> &x) const {
    Eigen::VectorXd t(getCoDimension());
    function(x, t);
    return t[0];
}

void TSRChainConstraint::testNewtonRaphson(const Eigen::Ref<Eigen::VectorXd> x0) {
    // initial guess
    Eigen::VectorXd out(7);

    // Newton-Raphson to solve Ax = b
    unsigned int iter = 0;
    double norm = 0;
    Eigen::MatrixXd A(6, 7);
    Eigen::VectorXd b(6);

    const double tolerance = getTolerance();
    const double squaredTolerance = tolerance * tolerance;

    // Initialize output to initial guess
    out = x0;
    // Initialize b with initial f(out) = b
    functiontest(out, b.head(6));
    std::cout << "out: \t" << out.transpose() << std::endl;
    std::cout << "b: \t" << b.transpose() << std::endl;

    while ((norm = b.squaredNorm()) > squaredTolerance && iter++ < getMaxIterations())
    {
        // Recompute the Jacobian at the new guess.
        jacobiantest(out, A.block(0, 0, 6, 7));

        // Move in the direction that decreases F(out) and is perpendicular to
        // the chart.
        out -= A.transpose() * (A*A.transpose()).inverse()*b;

        // Recompute b with new guess.
        functiontest(out, b.head(6));
        std::cout << "out: \t" << out.transpose() << std::endl;
        std::cout << "b: \t" << b.transpose() << std::endl;
    }
}

void TSRChainConstraint::functiontest(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const {
    auto pos = robotFK(x);
    auto dist = _tsr_chain->distance(pos);
    out = dist;
}

void TSRChainConstraint::jacobiantest(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const {
    Eigen::VectorXd y1 = x;
    Eigen::VectorXd y2 = x;
    Eigen::VectorXd t1(6);
    Eigen::VectorXd t2(6);

    // Use a 7-point central difference stencil on each column.
    for (std::size_t j = 0; j < n_; j++)
    {
        const double ax = std::fabs(x[j]);
        // Make step size as small as possible while still giving usable accuracy.
        const double h = std::sqrt(std::numeric_limits<double>::epsilon()) * (ax >= 1 ? ax : 1);

        // Can't assume y1[j]-y2[j] == 2*h because of precision errors.
        y1[j] += h;
        y2[j] -= h;
        functiontest(y1, t1);
        functiontest(y2, t2);
        const Eigen::VectorXd m1 = (t1 - t2) / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        functiontest(y1, t1);
        functiontest(y2, t2);
        const Eigen::VectorXd m2 = (t1 - t2) / (y1[j] - y2[j]);
        y1[j] += h;
        y2[j] -= h;
        functiontest(y1, t1);
        functiontest(y2, t2);
        const Eigen::VectorXd m3 = (t1 - t2) / (y1[j] - y2[j]);

        out.col(j) = 1.5 * m1 - 0.6 * m2 + 0.1 * m3;

        // Reset for next iteration.
        y1[j] = y2[j] = x[j];
    }
}
