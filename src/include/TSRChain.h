//
// Created by jiangeng on 10/24/19.
//

#ifndef ATLASMPNET_TSRCHAIN_H
#define ATLASMPNET_TSRCHAIN_H

#include <openrave/openrave.h>
#include <Eigen/Dense>
#include <ompl/base/Constraint.h>

#include "TSR.h"
#include "TSRRobot.h"

namespace AtlasMPNet {
    class TSRChain : public OpenRAVE::BaseXMLReader {
    public:
        typedef std::shared_ptr<TSRChain> Ptr;

        TSRChain() : BaseXMLReader() {}

        /**
        * Deserialize a serialized TSR Chain.
        *
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
         * Output operator
         */
        friend std::ostream &operator<<(std::ostream &O, const TSRChain &v) {
            v.serialize(O);
            return O;
        }

        /**
         * @return A sample from the TSR chain
         */
        Eigen::Affine3d sample() const;

        /**
         * Compute the distance to the TSR
         *
         * @param ee_pose The pose of the end-effector in world frame
         */
        Eigen::Matrix<double, 6, 1> distance(const Eigen::Affine3d &ee_pose) const;

        /**
         * Set the planning environment. This is required to enable computing
         * distance to a TSR.
         * @param penv The OpenRAVE environment this TSRChain will be used in
         */
        void setEnv(const OpenRAVE::EnvironmentBasePtr &penv);

        unsigned int GetDOF() const { return _tsr_robot->GetDOF(); }

//    private: //TODO: recover this
        AtlasMPNet::TSR::Ptr _active_tsr;
        bool _tag_open = false;
        const std::string _tag_name = "tsr_chain";

        std::vector<AtlasMPNet::TSR::Ptr> _tsrs;
        TSRRobot::Ptr _tsr_robot;
    };
}


#endif //ATLASMPNET_TSRCHAIN_H
