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

        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;

        bool project(Eigen::Ref<Eigen::VectorXd> x) const override;

    private:
        OpenRAVE::RobotBasePtr _robot;
        OpenRAVE::RobotBasePtr _tsr_robot;
        unsigned int _dof_robot;
        unsigned int _dof_tsr;

        // temporary variables
        int _robot_eeindex;
        int _tsr_eeindex;

        void robotFK(const Eigen::Ref<const Eigen::VectorXd> &x) const;
    };
}
#endif //ATLASMPNET_CONSTRAINT_H
