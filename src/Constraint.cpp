//
// Created by jiangeng on 10/12/19.
//
#include "Constraint.h"
#include "or_conversions.h"

using namespace AtlasMPNet;

TSRChainConstraint::TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const TSRChain::Ptr &tsr_chain) :
        Constraint(robot->GetActiveDOF(), robot->GetActiveDOF() - tsr_chain->GetDOF()) {
    _tsr_chain = tsr_chain;
    _robot = robot;
}

void TSRChainConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const {
    auto dist = _tsr_chain->distance(robotFK(x)); // TODO: need to project R6 to Rk, check if this is permitted
    unsigned int co_dim = getCoDimension();
    for (unsigned int i = 0; i < co_dim; ++i)
        out[i] = 0;
    for (unsigned int i = 0; i < 6; ++i)
        out[i % co_dim] += dist[i] * dist[i];
}

Eigen::Affine3d TSRChainConstraint::robotFK(const Eigen::Ref<const Eigen::VectorXd> &x) const {
    std::vector<double> qpos;
    for (unsigned int i = 0; i < getAmbientDimension(); ++i) {
        qpos.emplace_back(x[i]);
    }
    _robot->SetActiveDOFValues(qpos, 0);
    return toEigen(_robot->GetActiveManipulator()->GetEndEffectorTransform());
}
