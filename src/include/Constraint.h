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

        TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const OpenRAVE::RobotBasePtr &tsr_robot);

        ~TSRChainConstraint() override { delete[] _tsrjointval; }

        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;

    private:
        OpenRAVE::RobotBasePtr _robot;
        OpenRAVE::RobotBasePtr _tsr_robot;
        unsigned int _dof_robot;
        unsigned int _dof_tsr;
        mutable double* _tsrjointval = nullptr;

        // temporary variables


        OpenRAVE::Transform robotFK(const Eigen::Ref<const Eigen::VectorXd> &x) const;
    };
}
#endif //ATLASMPNET_CONSTRAINT_H
