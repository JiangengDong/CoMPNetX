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
#include "TaskSpaceRegion.h"

namespace AtlasMPNet {
    /// Class defining a TSR Chain: a more complex representation of pose constraints
    class TaskSpaceRegionChain : public OpenRAVE::BaseXMLReader {
    public:
        typedef std::shared_ptr<TaskSpaceRegionChain> Ptr;

        std::vector<TaskSpaceRegion> TSRChain; ///< this is an ordered list of TSRs, where each one relies on the previous one to determine T0_w, note that the T0_w values of the TSRs in the chain will change (except the first one)

        TaskSpaceRegionChain() {
            numdof = -1;
            _dx.resize(6);
            _sumbounds = -1;
        }

        ~TaskSpaceRegionChain() {
            DestoryRobotizedTSRChain();
        }
        // initialize the TSR chain
        bool Initialize(const OpenRAVE::EnvironmentBasePtr &penv_in);

        // create a virtual manipulator (a robot) corresponding to the TSR chain to use for ik solver calls
        bool RobotizeTSRChain(const OpenRAVE::EnvironmentBasePtr &penv_in, OpenRAVE::RobotBasePtr &probot_out);

        bool RobotizeTSRChain(const OpenRAVE::EnvironmentBasePtr &penv_in, OpenRAVE::RobotBasePtr &probot_out, int);

        // compute the distance between two transforms
        OpenRAVE::dReal TransformDifference(const OpenRAVE::Transform &tm_ref, const OpenRAVE::Transform &tm_targ) const;

        // get the closest transform in the TSR Chain to a query transform
        OpenRAVE::dReal GetClosestTransform(const OpenRAVE::Transform &T0_s, OpenRAVE::dReal *TSRJointVals, OpenRAVE::Transform &T0_closeset) const;

        // write the TSR Chain to a string
        bool serialize(std::ostream &O, int type = 0) const;

        friend std::ostream &operator<<(std::ostream &O, const TaskSpaceRegionChain &v) {
            v.serialize(O, 1);
            return O;
        }

        // parse a string to set the values of the TSR Chain
        bool deserialize(std::stringstream &_ss);

        // parse a string const from matlab to set the& values of the TSR Chain
        bool deserialize_from_matlab(const OpenRAVE::RobotBasePtr &robot_in, const OpenRAVE::EnvironmentBasePtr &penv_in, std::istream &_ss);

        ProcessElement startElement(const std::string &name, const OpenRAVE::AttributesList &atts) override;

        bool endElement(const std::string &name) override;

        void characters(const std::string &ch) override {}

        // generate a sample from this TSR Chain
        OpenRAVE::Transform GenerateSample();

        // add a TSR to the chain
        void AddTSR(TaskSpaceRegion &TSR) {
            TSRChain.push_back(TSR);
        }

        // get the joint limits of the virtual manipulator
        bool GetChainJointLimits(OpenRAVE::dReal *lowerlimits, OpenRAVE::dReal *upperlimits) const;

        // get a pointer to the mimiced body
        OpenRAVE::RobotBasePtr GetMimicBody() const {
            return _pmimicbody;
        }

        // get the number of mimiced DOFs
        int GetNumMimicDOF() const {
            return _mimicinds.size();
        }

        // get the mimiced DOFs
        std::vector<int> GetMimicDOFInds() const {
            return _mimicinds;
        }

        // get the values of the mimiced DOFs
        bool ExtractMimicDOFValues(const OpenRAVE::dReal *TSRValues, OpenRAVE::dReal *MimicDOFVals) const;

        // turn the list of mimic joint values into a list of full joint values
        bool MimicValuesToFullMimicBodyValues(const OpenRAVE::dReal *TSRJointVals, std::vector<OpenRAVE::dReal> &mimicbodyvals);

        // apply mimiced joint values to a certain set of joints
        bool ApplyMimicValuesToMimicBody(const OpenRAVE::dReal *TSRJointVals);

        // return the sum of the length of the bounds, summed over all TSRs in the chain
        OpenRAVE::dReal GetSumOfBounds() const {
            if (_sumbounds < 0)
                        RAVELOG_INFO ("ERROR TSR not initialized\n");
            else
                return _sumbounds;
        }

        // return the manipulator index of the first TSR
        int GetManipInd() const {
            if (!TSRChain.empty())
                return TSRChain[0].manipind;
            return -1;
        }

        // get the number of DOFs of the virtual manipulator
        int GetNumDOF() const {
            if (numdof == -1)
                        RAVELOG_INFO ("ERROR : this chain has not been robotized yet\n");
            return numdof;
        }

        // is this TSR chain used for sampling goals?
        bool IsForGoalSampling() const {
            return bSampleGoalFromChain;
        }

        // is this TSR chain used for sampling starts?
        bool IsForStartSampling() const {
            return bSampleStartFromChain;
        }

        // is this TSR chain used for constraining the whole path?
        bool IsForConstraint() const {
            return bConstrainToChain;
        }

    private:

        void DestoryRobotizedTSRChain(); ///< delete the virtual manipulator from the environment

        bool bSampleGoalFromChain;
        bool bSampleStartFromChain;
        bool bConstrainToChain;

        OpenRAVE::Transform Tw0_e;
        int numdof;

        OpenRAVE::RobotBasePtr robot;
        OpenRAVE::IkSolverBasePtr _pIkSolver;
        OpenRAVE::EnvironmentBasePtr penv;

        std::string mimicbodyname;
        OpenRAVE::RobotBasePtr _pmimicbody;
        std::vector<int> _mimicinds;
        std::vector<OpenRAVE::dReal> _mimicjointvals_temp;
        std::vector<OpenRAVE::dReal> _mimicjointoffsets;
        std::vector<OpenRAVE::dReal> _lowerlimits;
        std::vector<OpenRAVE::dReal> _upperlimits;
        bool _bPointTSR;

        mutable OpenRAVE::Transform _tmtemp;
        mutable std::vector<OpenRAVE::dReal> _dx;
        mutable OpenRAVE::dReal _sumsqr;

        OpenRAVE::dReal _sumbounds;

        mutable std::vector<OpenRAVE::dReal> ikparams;

        // inner variable for xml input
        bool _tag_open = false;
        std::string _tag_name = "tsr_chain";
        TaskSpaceRegion _temp_tsr;
    };
}
#endif //ATLASMPNET_TASKSPACEREGIONCHAIN_H
