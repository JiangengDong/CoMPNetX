/* Copyright (c) 2010 Carnegie Mellon University and Intel Corporation
   Author: Dmitry Berenson <dberenso@cs.cmu.edu>

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of Intel Corporation nor Carnegie Mellon University,
       nor the names of their contributors, may be used to endorse or
       promote products derived from this software without specific prior
       written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL INTEL CORPORATION OR CARNEGIE MELLON
   UNIVERSITY BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
   OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
   ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ATLASMPNET_TASKSPACEREGIONCHAIN_H
#define ATLASMPNET_TASKSPACEREGIONCHAIN_H

#include <openrave/openrave.h>

#include "Parameters.h"
#include <utility>

namespace AtlasMPNet {
    /// Class defining a TSR Chain: a more complex representation of pose constraints
    class TaskSpaceRegionChain {
    public:
        typedef std::shared_ptr<TaskSpaceRegionChain> Ptr;
        // this is an ordered list of TSRs, where each one relies on the previous one to determine T0_w,
        // note that the T0_w values of the TSRs in the chain will change (except the first one)
        TSRChainParameter param;

        TaskSpaceRegionChain(const OpenRAVE::EnvironmentBasePtr &penv_in, const TSRChainParameter &param);

        ~TaskSpaceRegionChain() {
            DestoryRobotizedTSRChain();
        }

        // create a virtual manipulator (a robot) corresponding to the TSR chain to use for ik solver calls
        bool RobotizeTSRChain(const OpenRAVE::EnvironmentBasePtr &penv_in);

        double TransformDifference(const OpenRAVE::Transform &T0_s, const OpenRAVE::Transform &T0_g, OpenRAVE::Transform &Tdiff) const {
            Tdiff = T0_s.inverse() * T0_g;
            return Tdiff.trans.lengthsqr3() + Tdiff.rot.y * Tdiff.rot.y + Tdiff.rot.z * Tdiff.rot.z + Tdiff.rot.w * Tdiff.rot.w;
        }

        // get the closest transform in the TSR Chain to a query transform
        OpenRAVE::dReal GetClosestTransform(const OpenRAVE::Transform &T0_s, std::vector<OpenRAVE::dReal> &TSRJointVals, OpenRAVE::Transform &T0_closest) const;

        // get the joint limits of the virtual manipulator
        bool GetChainJointLimits(OpenRAVE::dReal *lowerlimits, OpenRAVE::dReal *upperlimits) const;

        bool GetChainJointLimits(std::vector<OpenRAVE::dReal> &lowerlimits, std::vector<OpenRAVE::dReal> &upperlimits) const;

        // get a pointer to the mimiced body
        OpenRAVE::RobotBasePtr GetMimicBody() const {
            return _mimicbody;
        }

        // get the number of mimiced DOFs
        int GetNumMimicDOF() const {
            return _mimic_inds.size();
        }

        // get the mimiced DOFs
        std::vector<int> GetMimicDOFInds() const {
            return _mimic_inds;
        }

        // get the values of the mimiced DOFs
        bool ExtractMimicDOFValues(const OpenRAVE::dReal *TSRValues, OpenRAVE::dReal *MimicDOFVals) const;

        // turn the list of mimic joint values into a list of full joint values
        bool MimicValuesToFullMimicBodyValues(const OpenRAVE::dReal *TSRJointVals, std::vector<OpenRAVE::dReal> &mimicbodyvals);

        bool MimicValuesToFullMimicBodyValues(const std::vector<OpenRAVE::dReal> &TSRJointVals, std::vector<OpenRAVE::dReal> &mimicbodyvals);

        // apply mimiced joint values to a certain set of joints
        bool ApplyMimicValuesToMimicBody(const OpenRAVE::dReal *TSRJointVals);

        bool ApplyMimicValuesToMimicBody(const std::vector<OpenRAVE::dReal> &TSRJointVals);

        // return the manipulator index of the first TSR
        int GetManipInd() const {
            return param.manipind;
        }

        // get the number of DOFs of the virtual manipulator
        int GetNumDOF() const {
            if (numdof == -1)
                RAVELOG_INFO("ERROR : this chain has not been robotized yet\n");
            return numdof;
        }

        // is this TSR chain used for sampling goals?
        bool IsForGoalSampling() const {
            return param.purpose == TSRChainParameter::SAMPLE_GOAL;
        }

        // is this TSR chain used for sampling starts?
        bool IsForStartSampling() const {
            return param.purpose == TSRChainParameter::SAMPLE_START;
        }

        // is this TSR chain used for constraining the whole path?
        bool IsForConstraint() const {
            return param.purpose == TSRChainParameter::CONSTRAINT;
        }

        OpenRAVE::RobotBasePtr GetRobot() const {
            return robot;
        }

        void SetActiveDOFValues(const std::vector<double> &q) const {
            if (!_bPointTSR) {
                if (q.size() == numdof)
                    robot->SetActiveDOFValues(q, 0);
                else
                    RAVELOG_WARN("Incompatible dof values. ");
            }
        }

        OpenRAVE::Transform GetEndEffectorTransform() const {
            if (_bPointTSR)
                return prelativetolink->GetTransform() * param.TSRs[0].T0_w * param.TSRs[0].Tw_e;
            else
                return manipulator->GetEndEffectorTransform();
        }

        void CalculateActiveRotationJacobian(const OpenRAVE::Vector &qInitialRot, std::vector<OpenRAVE::dReal> &jacobian) const {
            if (_bPointTSR) {
                jacobian.clear();
                jacobian.resize(numdof);
            } else
                robot->CalculateActiveRotationJacobian(eeIndex, qInitialRot, jacobian);
        }

        void CalculateActiveJacobian(const OpenRAVE::Vector &offset, std::vector<OpenRAVE::dReal> &jacobian) {
            if (_bPointTSR) {
                jacobian.clear();
                jacobian.resize(numdof);
            } else
                robot->CalculateActiveJacobian(eeIndex, offset, jacobian);
        }

    private:
        void DestoryRobotizedTSRChain(); ///< delete the virtual manipulator from the environment

        void GetJacobian(const OpenRAVE::Transform &T0_s, const OpenRAVE::Transform &T0_closest, Eigen::Ref<Eigen::MatrixXd> J) const;

        OpenRAVE::Transform ForwardKinematics(std::vector<OpenRAVE::dReal> &TSRJointVals) const;

        OpenRAVE::EnvironmentBasePtr penv;
        OpenRAVE::RobotBasePtr robot;
        OpenRAVE::RobotBase::ManipulatorPtr manipulator;
        int eeIndex;
        int numdof;
        std::vector<OpenRAVE::dReal> _lowerlimits;
        std::vector<OpenRAVE::dReal> _upperlimits;

        OpenRAVE::RobotBasePtr _mimicbody;
        std::vector<int> _mimic_inds;
        std::vector<OpenRAVE::dReal> _mimicjointvals_temp;
        std::vector<OpenRAVE::dReal> _mimicjointoffsets;

        OpenRAVE::KinBody::LinkPtr prelativetolink;

        bool _bPointTSR;
    };
} // namespace AtlasMPNet
#endif //ATLASMPNET_TASKSPACEREGIONCHAIN_H
