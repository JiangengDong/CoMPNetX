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

        TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const TaskSpaceRegionChain::Ptr &tsr_chain);

        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;

    private:
        OpenRAVE::RobotBasePtr _robot;
        int _dof_robot;
        int _robot_eeindex;

        TaskSpaceRegionChain::Ptr _tsr_chain;
        OpenRAVE::RobotBasePtr _tsr_robot;
        int _dof_tsr;
        int _tsr_eeindex;

        void robotFK(const Eigen::Ref<const Eigen::VectorXd> &x) const;
    };
}
#endif //ATLASMPNET_CONSTRAINT_H
