//
// Created by jiangeng on 10/24/19.
//

#include <boost/make_shared.hpp>
#include "TSRChain.h"

using namespace AtlasMPNet;

/*
 * implementation of TSRChain
 */
OpenRAVE::BaseXMLReader::ProcessElement TSRChain::startElement(const std::string &name, const OpenRAVE::AttributesList &atts) {
    if (name == _tag_name) {
        if (_tag_open) {
            return PE_Ignore;
        } else {
            _tag_open = true;
            return PE_Support;
        }
    } else if (name == "tsr") {
        if (_tag_open) {
            _active_tsr = std::make_shared<TSR>();
            _active_tsr->startElement(name, atts);
            return PE_Support;
        } else {
                    RAVELOG_WARN("TSR cannot be placed outside TSRChain tags.");
            return PE_Ignore;
        }
    } else {
        return PE_Pass;
    }
}

bool TSRChain::endElement(const std::string &name) {
    if (name == _tag_name) {
        _tag_open = false;
        return true;
    } else {
        if (_tag_open) {
            _active_tsr->endElement(name);
            _tsrs.emplace_back(_active_tsr);
        }
        return false;
    }
}

bool TSRChain::serialize(std::ostream &O) const {
    O << "<" << _tag_name << ">" << std::endl;
    for (const auto &tsr:_tsrs) {
        O << *tsr << std::endl;
    }
    O << "</" << _tag_name << ">" << std::endl;
    return true;
}

Eigen::Affine3d TSRChain::sample() const {
    Eigen::Affine3d T0_w;
    if (_tsrs.empty())
        throw OpenRAVE::openrave_exception("There are no TSRs in this TSR Chain.", OpenRAVE::ORE_InvalidState);

    T0_w = _tsrs.front()->getOriginTransform();
    for (const auto &tsr: _tsrs)
        T0_w = T0_w * tsr->sampleDisplacementTransform() * tsr->getEndEffectorOffsetTransform();

    return T0_w;
}

Eigen::Matrix<double, 6, 1> TSRChain::distance(const Eigen::Affine3d &ee_pose) const {

    if (_tsrs.size() == 1) {
        TSR::Ptr tsr = _tsrs.front();
        return tsr->displacement(ee_pose);
    }

    if (!_tsr_robot)
        throw OpenRAVE::openrave_exception("Failed to compute distance to TSRChain. Did you set the environment by calling the setEnv function?",
                                           OpenRAVE::ORE_InvalidState);

//    if (!_tsr_robot->construct())
//        throw OpenRAVE::openrave_exception("Failed to robotize TSR.", OpenRAVE::ORE_Failed);    // TODO: delete this


            RAVELOG_DEBUG("[TSRChain] Solving IK to compute distance");
    // Compute the ideal pose of the end-effector
//    Eigen::Affine3d Ttarget = ee_pose * _tsrs.back()->getEndEffectorOffsetTransform().inverse();
    Eigen::Affine3d Ttarget = ee_pose;


    // Ask the robot to solve ik to find the closest possible end-effector transform
    Eigen::Affine3d Tnear = _tsr_robot->findNearestFeasibleTransform(Ttarget);

    // Compute the distance between the two
    Eigen::Affine3d offset = Tnear.inverse() * Ttarget;
    Eigen::Matrix<double, 6, 1> dist = Eigen::Matrix<double, 6, 1>::Zero();
    dist[0] = offset.translation()[0];
    dist[1] = offset.translation()[1];
    dist[2] = offset.translation()[2];
    dist[3] = atan2(offset.rotation()(2, 1), offset.rotation()(2, 2));
    dist[4] = -asin(offset.rotation()(2, 0));
    dist[5] = atan2(offset.rotation()(1, 0), offset.rotation()(0, 0));

    return dist;
}

void TSRChain::setEnv(const OpenRAVE::EnvironmentBasePtr &penv) {
    _tsr_robot = boost::make_shared<TSRRobot>(_tsrs, penv);
    _tsr_robot->construct();
}