//
// Created by jiangeng on 10/12/19.
//
#include "Constraint.h"

using namespace AtlasMPNet;

TSRChainConstraint::TSRChainConstraint(const unsigned int ambientDim, const unsigned int coDim) : Constraint(ambientDim, coDim) {
}

void TSRChainConstraint::function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const {
}

void TSRChainConstraint::jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const {
}
