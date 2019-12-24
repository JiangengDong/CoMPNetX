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
/** \file TaskSpaceRegion.h
    \brief Defines the Task Space Region and Task Space Region Chain class.es
 */
#ifndef  ATLASMPNET_TASKSPACEREGION_H
#define  ATLASMPNET_TASKSPACEREGION_H

#include <openrave/openrave.h>


namespace AtlasMPNet {
    /// Class defining a TSR: a simple representation of pose constraints
    class TaskSpaceRegion : public OpenRAVE::BaseXMLReader {
    public:
        int manipind; ///< this specifies the index of the manipulator of the robot that is associated with this TSR
        std::string relativebodyname; ///< name of the body T0_w is attached to (NULL = world frame)
        std::string relativelinkname; ///< name of the link T0_w is attached to (NULL = world frame)
        OpenRAVE::Transform T0_w; ///< the center of the TSR relative to the link it is attached to (or relative to world frame)
        OpenRAVE::Transform Tw_e; ///< the end-effector offset of this TSR
        OpenRAVE::dReal Bw[6][2]; ///< matrix defining maximum and minimum allowable deviation from T0_w in x,y,z,roll,pitch,and yaw

        OpenRAVE::KinBody::LinkPtr prelativetolink; ///< pointer to the link T0_w is attached to (NULL = world frame), this can be the link of a robot or of something else
        static OpenRAVE::Vector RPYIdentityOffsets[8]; ///< list of RPY identities

        TaskSpaceRegion();

        // initialize the TSR
        bool Initialize(const OpenRAVE::EnvironmentBasePtr &penv_in);

        // get the closest transform in the TSR to a query transform
        OpenRAVE::Transform GetClosestTransform(const OpenRAVE::Transform &T0_s) const;

        // get the distance to the TSR from a query transform
        OpenRAVE::dReal DistanceToTSR(const OpenRAVE::Transform &T0_s, std::vector<OpenRAVE::dReal> &dx) const;

        // generate a sample from this TSR
        OpenRAVE::Transform GenerateSample() const;

        // covert roll-pitch-yaw to a quaternion
        void RPYToQuat(const OpenRAVE::dReal *rpy, OpenRAVE::dReal *quat) const;

        // convert a quaternion to roll-pitch-yaw
        void QuatToRPY(const OpenRAVE::dReal *quat, OpenRAVE::dReal &psi, OpenRAVE::dReal &theta, OpenRAVE::dReal &phi) const;

        // print the TSR
        void Print() const;

        // write the TSR to a string
        bool serialize(std::ostream &O, int type=0) const;

        friend std::ostream &operator<<(std::ostream &O, const TaskSpaceRegion &v) {
            v.serialize(O, 1);
            return O;
        }

        // parse a string to set the values of the TSR
        bool deserialize(std::stringstream &_ss);

        // parse a string from matlab to set the values of the TSR
        bool deserialize_from_matlab(const OpenRAVE::RobotBasePtr &robot, const OpenRAVE::EnvironmentBasePtr &penv_in, std::istream &_ss);

        // xml input routine
        ProcessElement startElement(const std::string &name, const OpenRAVE::AttributesList &atts) override;

        bool endElement(const std::string &name) override;

        void characters(const std::string &ch) override {}

        // return the manipulator id of this TSR
        int GetManipInd() const {
            return manipind;
        }

        // return the 6D volume of this TSR
        OpenRAVE::dReal GetVolume() const {
            if (_volume < 0)
                        RAVELOG_INFO ("ERROR TSR not initialized\n");
            else
                return _volume;
        }

        // return the sum of the length of the bounds
        OpenRAVE::dReal GetSumOfBounds() const {
            if (_sumbounds < 0)
                        RAVELOG_INFO ("ERROR TSR not initialized\n");
            else
                return _sumbounds;
        }

    private:
        OpenRAVE::dReal _volume;
        OpenRAVE::dReal _sumbounds;
        OpenRAVE::dReal _dimensionality;
        //these are temporary variables used in TSR computations
        mutable OpenRAVE::Transform Tw_s1, T0_link, Tw_rand;
        mutable OpenRAVE::dReal dw_sample[6];
        mutable OpenRAVE::dReal frand;
        mutable OpenRAVE::dReal sumsqr;
        mutable OpenRAVE::dReal a, b, c, d;
        mutable OpenRAVE::dReal _cphi, _sphi, _ctheta, _stheta, _cpsi, _spsi;
        mutable OpenRAVE::dReal _min_dist;
        mutable OpenRAVE::Vector _temp_vec;
        mutable OpenRAVE::dReal _temp_dist;
        // inner variable for xml input
        bool _tag_open = false;
        std::string _tag_name = "tsr";
    };

    inline float RANDOM_FLOAT() {
        return rand() / ((float) RAND_MAX);
    }
}

#endif // ATLASMPNET_TASKSPACEREGION_H
