//
// Created by jiangeng on 10/11/19.
//

#ifndef COMPNETX_CONSTRAINT_H
#define COMPNETX_CONSTRAINT_H

#include <ompl/base/Constraint.h>
#include <openrave/openrave.h>

#include "TaskSpaceRegionChain.h"

namespace CoMPNetX {
class TSRChainConstraint : public ompl::base::Constraint {
public:
    typedef std::shared_ptr<TSRChainConstraint> Ptr;

    TSRChainConstraint(const OpenRAVE::RobotBasePtr &robot, const std::vector<TaskSpaceRegionChain::Ptr> &tsr_chains);

    void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override;

    void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override;

private:
    OpenRAVE::RobotBasePtr _robot;
    int _dof_robot;
    std::vector<int> _robot_eeindices;
    std::vector<OpenRAVE::RobotBase::ManipulatorPtr> _robot_manipulators;

    std::vector<TaskSpaceRegionChain::Ptr> _tsr_chains;
    int _num_tsr_chains;
    std::vector<int> _dof_tsrs;

    typedef std::vector<std::pair<OpenRAVE::Transform, OpenRAVE::Transform>> TransformPairVector;

    void robotFK(const Eigen::Ref<const Eigen::VectorXd> &x, TransformPairVector &Tpairs) const;
};
} // namespace CoMPNetX
#endif //COMPNETX_CONSTRAINT_H
