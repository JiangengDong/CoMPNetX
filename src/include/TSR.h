/***********************************************************************

Copyright (c) 2014, Carnegie Mellon University
All rights reserved.

Authors: Jennifer King <jeking04@gmail.com>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

#ifndef ATLASMPNET_TSR_H_
#define ATLASMPNET_TSR_H_

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <openrave/openrave.h>

namespace AtlasMPNet {

    class TSR : public OpenRAVE::BaseXMLReader {

    public:
        typedef std::shared_ptr<TSR> Ptr;

        /**
         * Constructor
         */
        TSR();

        /**
         * Constructor
         *
         * @param T0_w Transform from world frame to the TSR frame w
         * @param Tw_e End-effector offset transform in the coordinates of w
         * @param Bw 6x2 matrix of bounds in the coordinates of w
         *   bounds are (x, y, z, roll, pitch, yaw - assume RPY Euler angle convention)
         */
        TSR(const Eigen::Affine3d &T0_w,
            const Eigen::Affine3d &Tw_e,
            Eigen::Matrix<double, 6, 2> Bw);

        int manipulator_index() const;

        std::string relative_body_name() const;

        std::string relative_link_name() const;

        /**
         * Deserialize a serialized TSR.
         *
         * @param ss The stream to read the serialized TSR from
         */
        ProcessElement startElement(const std::string &name, const OpenRAVE::AttributesList &atts) override;

        bool endElement(const std::string &name) override;

        void characters(const std::string &ch) override {}

        /**
         * Serialize a TSR Chain.
         *
         * @param ss The stream to read the serialized TSR from
         */
        bool serialize(std::ostream &O) const;

        /**
         * Compute the distance to the TSR
         *
         * @param ee_pose The pose of the end-effector in world frame
         */
        Eigen::Matrix<double, 6, 1> distance(const Eigen::Affine3d &ee_pose) const;

        /**
         * Compute the displacement to the TSR
         *
         * @param ee_pose The pose of the end-effector in world frame
         */
        Eigen::Matrix<double, 6, 1> displacement(const Eigen::Affine3d &ee_pose) const;

        /**
         * Sample a pose from the TSR
         *
         * @return The sampled pose
         */
        Eigen::Affine3d sample() const;

        /**
         * Sample a displacement transform from the TSR
         * @return The sampled transform
         */
        Eigen::Affine3d sampleDisplacementTransform() const;

        /**
         * @return The transform for the frame of the TSR (T0_w)
         */
        Eigen::Affine3d getOriginTransform() const { return _T0_w; }

        /**
         * @return The end-effector offset transform (Tw_e)
         */
        Eigen::Affine3d getEndEffectorOffsetTransform() const { return _Tw_e; }

        /**
         * @return The bounds specified for the TSR (Bw)
         */
        Eigen::Matrix<double, 6, 2> getBounds() const { return _Bw; }

        /**
         * Output operator
         */
        friend std::ostream &operator<<(std::ostream &O, const TSR &v) {
            v.serialize(O);
            return O;
        }

    protected:
        Eigen::Affine3d _T0_w;
        Eigen::Affine3d _T0_w_inv;
        Eigen::Affine3d _Tw_e;
        Eigen::Affine3d _Tw_e_inv;
        Eigen::Matrix<double, 6, 2> _Bw;
        int _manipulator_index = 0;
        std::string _relative_body_name = "NULL";
        std::string _relative_link_name = "NULL";
        bool _initialized;
        bool _tag_open = false;
        const std::string _tag_name = "tsr";
    };

} // namespace AtlasMPNet

#endif // ATLASMPNET_TSR_H_
