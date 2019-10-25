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

#include <utility>
#include <vector>
#include <Eigen/Geometry>
#include <ompl/util/RandomNumbers.h>
#include "TSR.h"

using namespace AtlasMPNet;

TSR::TSR() : BaseXMLReader(), _initialized(false) {
}

TSR::TSR(const Eigen::Affine3d &T0_w, const Eigen::Affine3d &Tw_e, Eigen::Matrix<double, 6, 2> Bw) :
        BaseXMLReader(),
        _T0_w(T0_w),
        _Tw_e(Tw_e),
        _Bw(std::move(Bw)),
        _manipulator_index(-1),
        _relative_body_name("NULL"),
        _relative_link_name(""),
        _initialized(true) {
    _T0_w_inv = _T0_w.inverse();
    _Tw_e_inv = _Tw_e.inverse();
}

int TSR::manipulator_index() const {
    return _manipulator_index;
}

std::string TSR::relative_body_name() const {
    return _relative_body_name;
}

std::string TSR::relative_link_name() const {
    return std::__cxx11::string();
}

OpenRAVE::BaseXMLReader::ProcessElement TSR::startElement(const std::string &name, const OpenRAVE::AttributesList &atts) {
    if (name == _tag_name) {
        if (_tag_open)
            return PE_Ignore;
        else {
            std::istringstream value;
            _initialized = false;
            for (const auto &att:atts) {
                auto key = att.first;
                value.clear();
                value.str(att.second);
                if (key == "manipulator_index")
                    value >> _manipulator_index;
                else if (key == "relative_body_name")
                    value >> _relative_body_name;
                else if (key == "relative_link_name")
                    value >> _relative_link_name;
                else if (key == "t0_w") {
                    // Read in the T0_w matrix
                    for (unsigned int c = 0; c < 3; c++)
                        for (unsigned int r = 0; r < 3; r++)
                            value >> _T0_w.matrix()(r, c);

                    for (unsigned int idx = 0; idx < 3; idx++)
                        value >> _T0_w.translation()(idx);
                } else if (key == "tw_e") {
                    for (unsigned int c = 0; c < 3; c++)
                        for (unsigned int r = 0; r < 3; r++)
                            value >> _Tw_e.matrix()(r, c);

                    for (unsigned int idx = 0; idx < 3; idx++)
                        value >> _Tw_e.translation()(idx);
                } else if (key == "bw") {
                    // Read in the Bw matrix
                    for (unsigned int r = 0; r < 6; r++)
                        for (unsigned int c = 0; c < 2; c++)
                            value >> _Bw(r, c);
                } else
                            RAVELOG_WARN ("Unrecognized attribute %s.", key.c_str());
            }
            _T0_w_inv = _T0_w.inverse();
            _Tw_e_inv = _Tw_e.inverse();
            _initialized = true;
            _tag_open = true;
            return PE_Support;
        }
    } else
        return PE_Pass;
}

bool TSR::endElement(const std::string &name) {
    if (name == _tag_name) {
        _tag_open = false;
        return true;
    } else
        return false;
}

bool TSR::serialize(std::ostream &O) const {
    if (!_initialized)
        throw std::runtime_error("TSR is not initialized.");
    O << "<" << _tag_name;
//    O << " manipulator_index=\"" << _manipulator_index << "\""
//      << " relative_body_name=\"" << _relative_body_name << "\""
//      << " relative_link_name=\"" << _relative_link_name << "\"";
    // T0_w matrix
    O << " T0_w=\"";
    for (unsigned int c = 0; c < 3; c++)
        for (unsigned int r = 0; r < 3; r++)
            if (c==0 && r==0)
                O  << _T0_w.matrix()(r, c);
            else
                O << ' ' << _T0_w.matrix()(r, c);
    for (unsigned int idx = 0; idx < 3; idx++)
        O << ' ' << _T0_w.translation()(idx);
    O << "\"";
    // Tw_e matrix
    O << " Tw_e=\"";
    for (unsigned int c = 0; c < 3; c++)
        for (unsigned int r = 0; r < 3; r++)
            if (c==0 && r==0)
                O  << _Tw_e.matrix()(r, c);
            else
                O << ' ' << _Tw_e.matrix()(r, c);
    for (unsigned int idx = 0; idx < 3; idx++)
        O << ' ' << _Tw_e.translation()(idx);
    O << "\"";
    // Read in the Bw matrix
    O << " Bw=\"";
    for (unsigned int r = 0; r < 6; r++)
        for (unsigned int c = 0; c < 2; c++)
            if (c==0 && r==0)
                O  << _Bw.matrix()(r, c);
            else
                O << ' ' << _Bw.matrix()(r, c);
    O << "\"";
    O << "/>";
    return true;
}

Eigen::Matrix<double, 6, 1> TSR::distance(const Eigen::Affine3d &ee_pose) const {
    Eigen::Matrix<double, 6, 1> dist = Eigen::Matrix<double, 6, 1>::Zero();

    // First compute the pose of the w frame in world coordinates, given the ee_pose
    Eigen::Affine3d w_in_world = ee_pose * _Tw_e_inv;

    // Next compute the pose of the w frame relative to its original pose (as specified by T0_w)
    Eigen::Affine3d w_offset = _T0_w_inv * w_in_world;

    // Now compute the elements of the distance matrix
    dist(0, 0) = w_offset.translation()(0);
    dist(1, 0) = w_offset.translation()(1);
    dist(2, 0) = w_offset.translation()(2);
    dist(3, 0) = atan2(w_offset.rotation()(2, 1), w_offset.rotation()(2, 2));
    dist(4, 0) = -asin(w_offset.rotation()(2, 0));
    dist(5, 0) = atan2(w_offset.rotation()(1, 0), w_offset.rotation()(0, 0));

    return dist;
}

Eigen::Matrix<double, 6, 1> TSR::displacement(const Eigen::Affine3d &ee_pose) const {

    Eigen::Matrix<double, 6, 1> dist = distance(ee_pose);
    Eigen::Matrix<double, 6, 1> disp = Eigen::Matrix<double, 6, 1>::Zero();

    for (unsigned int idx = 0; idx < 6; idx++) {
        if (dist(idx, 0) < _Bw(idx, 0)) {
            disp(idx, 0) = dist(idx, 0) - _Bw(idx, 0);
        } else if (dist(idx, 0) > _Bw(idx, 1)) {
            disp(idx, 0) = dist(idx, 0) - _Bw(idx, 1);
        }
    }

    return disp;
}

Eigen::Affine3d TSR::sampleDisplacementTransform() const {

    // First sample uniformly betwee each of the bounds of Bw
    std::vector<double> d_sample(6);

    ompl::RNG rng;
    for (int idx = 0; idx < d_sample.size(); idx++) {
        if (_Bw(idx, 1) > _Bw(idx, 0)) {
            d_sample[idx] = rng.uniformReal(_Bw(idx, 0), _Bw(idx, 1));
        }
    }

    Eigen::Affine3d return_tf;
    return_tf.translation() << d_sample[0], d_sample[1], d_sample[2];

    // Convert to a transform matrix
    double roll = d_sample[3];
    double pitch = d_sample[4];
    double yaw = d_sample[5];

    double A = cos(yaw);
    double B = sin(yaw);
    double C = cos(pitch);
    double D = sin(pitch);
    double E = cos(roll);
    double F = sin(roll);
    return_tf.linear() << A * C, A * D * F - B * E, B * F + A * D * E,
            B * C, A * E + B * D * F, B * D * E - A * F,
            -D, C * F, C * E;

    return return_tf;
}

Eigen::Affine3d TSR::sample() const {

    Eigen::Affine3d tf = sampleDisplacementTransform();

    return _T0_w * tf * _Tw_e;
}
