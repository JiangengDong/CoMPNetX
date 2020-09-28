#ifndef ATLASMPNET_ROBOTHELPER_H
#define ATLASMPNET_ROBOTHELPER_H

#include <Eigen/Dense>
#include <openrave/openrave.h>

namespace AtlasMPNet {
    class RobotHelper 
    {
    public:
        RobotHelper(OpenRAVE::RobotBasePtr robot, OpenRAVE::RobotBase::ManipulatorPtr manip) : robot_(robot),
                                                                                               manip_(manip),
                                                                                               numdof(robot->GetActiveDOF()) {
            robot_->GetActiveDOFLimits(_lower_limits, _upper_limits);
            eeIndex = manip_->GetEndEffector()->GetIndex();
        };

        ~RobotHelper(){};

        void EnforceBound(std::vector<double> &TSRJointVals) const {
            for (int i = 0; i < numdof; i++) {
                if (TSRJointVals[i] > _upper_limits[i])
                    TSRJointVals[i] = _upper_limits[i];
                else if (TSRJointVals[i] < _lower_limits[i])
                    TSRJointVals[i] = _lower_limits[i];
            }
        }

        OpenRAVE::Transform ForwardKinematics(std::vector<double> &TSRJointVals) const {
            robot_->SetActiveDOFValues(TSRJointVals, 0);
            return manip_->GetEndEffectorTransform();
        }

        double TransformDifference(const OpenRAVE::Transform &T0_s, const OpenRAVE::Transform &T0_g, OpenRAVE::Transform &Tdiff) const {
            Tdiff = T0_s.inverse() * T0_g;
            return Tdiff.trans.lengthsqr3() + Tdiff.rot.y * Tdiff.rot.y + Tdiff.rot.z * Tdiff.rot.z + Tdiff.rot.w * Tdiff.rot.w;
        }

        OpenRAVE::Transform GetEndEffectorTransform() const {
            return manip_->GetEndEffectorTransform();
        }

        void GetJacobian(const OpenRAVE::Transform &T0_s, const OpenRAVE::Transform &T0_closest, Eigen::Ref<Eigen::MatrixXd> J) const;

        double GetClosestTransform(const OpenRAVE::Transform &T0_s, std::vector<OpenRAVE::dReal> &TSRJointVals, OpenRAVE::Transform &T0_closest) const;

        OpenRAVE::RobotBasePtr robot_;
        OpenRAVE::RobotBase::ManipulatorPtr manip_;

    private:
        unsigned int numdof;
        std::vector<double> _lower_limits, _upper_limits;
        int eeIndex;
    };
} // namespace AtlasMPNet

#endif // ATLASMPNET_ROBOTHELPER_H