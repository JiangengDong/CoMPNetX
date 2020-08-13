#include "RobotHelper.h"

using namespace AtlasMPNet;

void RobotHelper::GetJacobian(const OpenRAVE::Transform &T0_s, const OpenRAVE::Transform &T0_closest, Eigen::Ref<Eigen::MatrixXd> J) const
{
    std::vector<OpenRAVE::dReal> Jtrans, Jrot;

    robot_->CalculateActiveJacobian(eeIndex, T0_closest.trans, Jtrans);
    robot_->CalculateActiveRotationJacobian(eeIndex, T0_closest.rot, Jrot);
    // copy to Eigen
    for (int col = 0; col < numdof; col++)
    {
        for (int row = 0; row < 3; row++)
        {
            J(row, col) = Jtrans[row * numdof + col];
        }
        J(3, col) = T0_s.rot[0] * Jrot[1 * numdof + col] - T0_s.rot[1] * Jrot[0 * numdof + col] - T0_s.rot[2] * Jrot[3 * numdof + col] + T0_s.rot[3] * Jrot[2 * numdof + col];
        J(4, col) = T0_s.rot[0] * Jrot[2 * numdof + col] + T0_s.rot[1] * Jrot[3 * numdof + col] - T0_s.rot[2] * Jrot[0 * numdof + col] - T0_s.rot[3] * Jrot[1 * numdof + col];
        J(5, col) = T0_s.rot[0] * Jrot[3 * numdof + col] - T0_s.rot[1] * Jrot[2 * numdof + col] + T0_s.rot[2] * Jrot[1 * numdof + col] - T0_s.rot[3] * Jrot[0 * numdof + col];
    }
}

double RobotHelper::GetClosestTransform(const OpenRAVE::Transform &T0_s, std::vector<OpenRAVE::dReal> &TSRJointVals, OpenRAVE::Transform &T0_closest) const
{
    Eigen::MatrixXd J(6, numdof);
    Eigen::VectorXd p(6), q(numdof);
    std::vector<OpenRAVE::dReal> Jtrans, Jrot;
    OpenRAVE::Transform Tdiff;
    double squaredNorm = 1000;
    
    EnforceBound(TSRJointVals);
    T0_closest = ForwardKinematics(TSRJointVals);
    squaredNorm = TransformDifference(T0_s, T0_closest, Tdiff);
    for (int it = 0; it < 50; it++)
    {
        if (squaredNorm < 1e-16)
            break;

        // copy to Eigen
        p[0] = T0_closest.trans.x - T0_s.trans.x;
        p[1] = T0_closest.trans.y - T0_s.trans.y;
        p[2] = T0_closest.trans.z - T0_s.trans.z;
        p[3] = Tdiff.rot.y;
        p[4] = Tdiff.rot.z;
        p[5] = Tdiff.rot.w;
        GetJacobian(T0_s, T0_closest, J);

        q = J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(p);
        double probe_norm = 0;
        std::vector<OpenRAVE::dReal> probe(TSRJointVals);
        double stepsize;
        for (stepsize = 1; stepsize > 1e-4; stepsize /= 2)
        {
            for (int i = 0; i < numdof; i++)
                probe[i] = TSRJointVals[i] - stepsize * q[i];
            EnforceBound(probe);
            T0_closest = ForwardKinematics(probe);
            probe_norm = TransformDifference(T0_s, T0_closest, Tdiff);
            if (probe_norm < squaredNorm || probe_norm < 1e-16)
                break;
        }
        squaredNorm = probe_norm;
        TSRJointVals = probe;
        if (stepsize <= 1e-4)
            break;
    }
    return sqrt(squaredNorm);
}