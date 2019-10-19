//
// Created by jiangeng on 10/11/19.
//

#ifndef ATLASMPNET_CONSTRAINT_H
#define ATLASMPNET_CONSTRAINT_H

#include <ompl/base/Constraint.h>

#include "Parameters.h"
#include "TSR.h"
#include "TSRRobot.h"

namespace AtlasMPNet {
    // TODO: implement this
    class TSRChainConstraint : public ompl::base::Constraint {
    public:
        typedef std::shared_ptr<TSRChainConstraint> Ptr;

        TSRChainConstraint(unsigned int ambientDim, unsigned int coDim);

        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;
    };

    class SphereConstraint : public ompl::base::Constraint {
    public:
        SphereConstraint() : ompl::base::Constraint(11, 1) {
        }

        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override {
            out[0] = x.norm() - 1;
        }

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override {
            out = x.transpose().normalized();
        }
    };
}
#endif //ATLASMPNET_CONSTRAINT_H
