//
// Created by jiangeng on 10/11/19.
//

#ifndef ATLASMPNET_CONSTRAINT_H
#define ATLASMPNET_CONSTRAINT_H

#include <ompl/base/Constraint.h>
#include <openrave/openrave.h>

#include "TaskSpaceRegionChain.h"

namespace AtlasMPNet {
    class TSRChainConstraint : public ompl::base::Constraint {
    public:
        typedef std::shared_ptr<TSRChainConstraint> Ptr;

        TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const TaskSpaceRegionChain &tsr_chain);

        ~TSRChainConstraint() override { delete[] _tsrjointval; }

        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;

        double distance(const Eigen::Ref<const Eigen::VectorXd> &x) const override;

        void printHessian(const Eigen::Ref<const Eigen::VectorXd> &x) const;

    private:
        TaskSpaceRegionChain _tsr_chain;
        OpenRAVE::RobotBasePtr _robot;
        mutable double* _tsrjointval = nullptr;

        OpenRAVE::Transform robotFK(const Eigen::Ref<const Eigen::VectorXd> &x) const;
    };

    class SphereConstraint : public ompl::base::Constraint {
    public:
        explicit SphereConstraint(const unsigned int dim) : ompl::base::Constraint(dim, 1) {
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
