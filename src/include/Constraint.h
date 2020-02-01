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

        TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const std::vector<TaskSpaceRegionChain::Ptr> &tsr_chains);

        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;

    private:
        OpenRAVE::RobotBasePtr _robot;
        int _dof_robot;
        int _robot_eeindex;
        std::vector<int> _robot_eeindices;
        std::vector<OpenRAVE::RobotBase::ManipulatorPtr> _robot_manipulators;

        TaskSpaceRegionChain::Ptr _tsr_chain;
        std::vector<TaskSpaceRegionChain::Ptr> _tsr_chains;
        OpenRAVE::RobotBasePtr _tsr_robot;
        std::vector<OpenRAVE::RobotBasePtr > _tsr_robots;
        int _num_tsr_chains;
        int _dof_tsr{};
        std::vector<int> _dof_tsrs;
        int _tsr_eeindex{};
        std::vector<int> _tsr_eeindices;

        typedef std::vector<std::pair<OpenRAVE::Transform, OpenRAVE::Transform>> TransformPairVector;

        void robotFK(const Eigen::Ref<const Eigen::VectorXd> &x, TransformPairVector& Tpairs) const;
    };
}
#endif //ATLASMPNET_CONSTRAINT_H
