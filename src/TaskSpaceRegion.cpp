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
/** \file TaskSpaceRegion.cpp
    \brief Defines the Task Space Region and Task Space Region Chain class.es
 */
#include "TaskSpaceRegion.h"
#include <openrave/openrave.h>

using namespace AtlasMPNet;

OpenRAVE::Vector TaskSpaceRegion::RPYIdentityOffsets[8] = {OpenRAVE::Vector(M_PI, M_PI, M_PI), OpenRAVE::Vector(M_PI, M_PI, -M_PI),
                                                           OpenRAVE::Vector(M_PI, -M_PI, M_PI), OpenRAVE::Vector(M_PI, -M_PI, -M_PI),
                                                           OpenRAVE::Vector(-M_PI, M_PI, M_PI), OpenRAVE::Vector(-M_PI, M_PI, -M_PI),
                                                           OpenRAVE::Vector(-M_PI, -M_PI, M_PI), OpenRAVE::Vector(-M_PI, -M_PI, -M_PI)};

TaskSpaceRegion::TaskSpaceRegion() : BaseXMLReader() {
    manipind = -1;
    Bw[0][0] = 0;
    Bw[0][1] = 0;
    Bw[1][0] = 0;
    Bw[1][1] = 0;
    Bw[2][0] = 0;
    Bw[2][1] = 0;
    Bw[3][0] = 0;
    Bw[3][1] = 0;
    Bw[4][0] = 0;
    Bw[4][1] = 0;
    Bw[5][0] = 0;
    Bw[5][1] = 0;

    _volume = -1;
    _sumbounds = -1;
    _dimensionality = -1;
}

bool TaskSpaceRegion::Initialize(const OpenRAVE::EnvironmentBasePtr &penv_in) {
    _volume = 0;
    _sumbounds = 0;
    _dimensionality = 0;

    for (auto &i : Bw) {
        //compute volume in whatever dimensionality this defines
        //when Bw values are backwards, it signifies an axis flip (not an error)
        if (i[1] != i[0]) {
            _volume = _volume * fabs(i[1] - i[0]);
            _sumbounds = _sumbounds + fabs(i[1] - i[0]);
            _dimensionality++;
        }
    }

    if (strcasecmp(relativebodyname.c_str(), "NULL") == 0) {
        prelativetolink.reset();
    } else {
        OpenRAVE::KinBodyPtr pobject;
        pobject = penv_in->GetKinBody(relativebodyname);
        if (pobject.get() == nullptr) {
                    RAVELOG_INFO("Error: could not find the specified object to attach frame\n");
            return false;
        }

        //find the link
        std::vector<OpenRAVE::KinBody::LinkPtr> vlinks = pobject->GetLinks();
        bool bGotLink = false;
        for (auto &vlink : vlinks) {
            if (strcmp(relativelinkname.c_str(), vlink->GetName().c_str()) == 0) {
                        RAVELOG_INFO("frame link: %s:%s\n", vlink->GetParent()->GetName().c_str(), vlink->GetName().c_str());
                prelativetolink = vlink;
                bGotLink = true;
                break;
            }
        }
        if (!bGotLink) {
                    RAVELOG_INFO("Error: could not find the specified link of the object to attach frame\n");
            return false;
        }
    }

    return true;
}

OpenRAVE::Transform TaskSpaceRegion::GetClosestTransform(const OpenRAVE::Transform &T0_s) const {
    //dw_sample is just used here b/c it's convenient, it's not actually sampling anything
    if (prelativetolink.get() == nullptr)
        T0_link = OpenRAVE::Transform();
    else
        T0_link = prelativetolink->GetTransform();

    Tw_s1 = (T0_link * T0_w).inverse() * T0_s * Tw_e.inverse();

    //convert to task coordinates
    dw_sample[0] = Tw_s1.trans.x;
    dw_sample[1] = Tw_s1.trans.y;
    dw_sample[2] = Tw_s1.trans.z;
    QuatToRPY(&Tw_s1.rot[0], dw_sample[3], dw_sample[4], dw_sample[5]);
    //RAVELOG_INFO("dw_s: %f %f %f %f %f %f\n",dw_sample[0],dw_sample[1],dw_sample[2],dw_sample[3],dw_sample[4],dw_sample[5]);

    for (int i = 0; i < 6; i++) {
        if (dw_sample[i] > Bw[i][1])
            dw_sample[i] = Bw[i][1];
        else if (dw_sample[i] < Bw[i][0])
            dw_sample[i] = Bw[i][0];
    }

    //RAVELOG_INFO("closest: %f %f %f %f %f %f\n",dw_sample[0],dw_sample[1],dw_sample[2],dw_sample[3],dw_sample[4],dw_sample[5]);

    Tw_rand.trans.x = dw_sample[0];
    Tw_rand.trans.y = dw_sample[1];
    Tw_rand.trans.z = dw_sample[2];
    RPYToQuat(&dw_sample[3], &Tw_rand.rot[0]);

    if (prelativetolink.get() == nullptr)
        T0_link = OpenRAVE::Transform();
    else
        T0_link = prelativetolink->GetTransform();

    return (T0_link * T0_w * Tw_rand * Tw_e);
}

OpenRAVE::dReal TaskSpaceRegion::DistanceToTSR(const OpenRAVE::Transform &T0_s, std::vector<OpenRAVE::dReal> &dx) const {
    dx.resize(6);

    if (prelativetolink.get() == nullptr)
        T0_link = OpenRAVE::Transform();
    else
        T0_link = prelativetolink->GetTransform();

    Tw_s1 = (T0_link * T0_w).inverse() * T0_s * Tw_e.inverse();

    //convert to task coordinates
    dx[0] = Tw_s1.trans.x;
    dx[1] = Tw_s1.trans.y;
    dx[2] = Tw_s1.trans.z;

    QuatToRPY(&Tw_s1.rot[0], dx[3], dx[4], dx[5]);

    sumsqr = 0;
    for (int i = 0; i < 6; i++) {
        if (dx[i] > Bw[i][1])
            dx[i] = dx[i] - Bw[i][1];
        else if (dx[i] < Bw[i][0])
            dx[i] = dx[i] - Bw[i][0];
        else
            dx[i] = 0;
        sumsqr += dx[i] * dx[i];
    }

    return sqrt(sumsqr);
}

OpenRAVE::Transform TaskSpaceRegion::GenerateSample() const {
    for (int i = 0; i < 6; i++) {
        frand = RANDOM_FLOAT();
        dw_sample[i] = Bw[i][1] * frand + Bw[i][0] * (1 - frand);
    }

    RPYToQuat(&dw_sample[3], &Tw_rand.rot[0]);

    Tw_rand.trans.x = dw_sample[0];
    Tw_rand.trans.y = dw_sample[1];
    Tw_rand.trans.z = dw_sample[2];

    if (prelativetolink.get() == nullptr)
        T0_link = OpenRAVE::Transform();
    else
        T0_link = prelativetolink->GetTransform();

    return (T0_link * T0_w * Tw_rand * Tw_e);
}

void TaskSpaceRegion::RPYToQuat(const OpenRAVE::dReal *rpy, OpenRAVE::dReal *quat) const {
    _cphi = cos(rpy[0] / 2);
    _sphi = sin(rpy[0] / 2);
    _ctheta = cos(rpy[1] / 2);
    _stheta = sin(rpy[1] / 2);
    _cpsi = cos(rpy[2] / 2);
    _spsi = sin(rpy[2] / 2);

    quat[0] = _cphi * _ctheta * _cpsi + _sphi * _stheta * _spsi;
    quat[1] = _sphi * _ctheta * _cpsi - _cphi * _stheta * _spsi;
    quat[2] = _cphi * _stheta * _cpsi + _sphi * _ctheta * _spsi;
    quat[3] = _cphi * _ctheta * _spsi - _sphi * _stheta * _cpsi;
}

void TaskSpaceRegion::QuatToRPY(const OpenRAVE::dReal *quat, OpenRAVE::dReal &psi, OpenRAVE::dReal &theta, OpenRAVE::dReal &phi) const {
    a = quat[0];
    b = quat[1];
    c = quat[2];
    d = quat[3];

    //psi theta and phi will always be between -pi and pi
    psi = atan2(2 * a * b + 2 * c * d, a * a - b * b - c * c + d * d); //psi
    theta = -asin(2 * b * d - 2 * a * c); //theta
    phi = atan2(2 * a * d + 2 * b * c, a * a + b * b - c * c - d * d); //phi

    //go through all the identities and find which one minimizes the total rotational distance
    //don't need to consider +/-2pi b/c all three angles are between -pi and pi
    _min_dist = 10000;
    for (int i = 0; i < 9; i++) {

        if (i == 0) //for first element, use original values
        {
            _temp_vec.x = psi;
            _temp_vec.y = theta;
            _temp_vec.z = phi;
        } else {
            _temp_vec.x = psi + RPYIdentityOffsets[i - 1].x;
            _temp_vec.y = -theta + RPYIdentityOffsets[i - 1].y;//note that theta is negative
            _temp_vec.z = phi + RPYIdentityOffsets[i - 1].z;
        }

        _temp_dist = _temp_vec.lengthsqr3();
        if (_temp_dist < _min_dist) {
            _min_dist = _temp_dist;
            psi = _temp_vec.x;
            theta = _temp_vec.y;
            phi = _temp_vec.z;
        }
    }
    //RAVELOG_INFO("psi: %f, theta: %f, phi: %f\n",psi,theta,phi);
}

void TaskSpaceRegion::Print() const {
    std::stringstream O;
    O << std::endl;
    O << "Manipulator Pointer: " << manipind << " ";
    O << std::endl;

    O << "Link Pointer: ";
    if (prelativetolink.get() == nullptr)
        O << "NULL";
    else
        O << prelativetolink->GetParent()->GetName() << ":" << prelativetolink->GetName() << " ";
    O << std::endl;

    O << "T0_w:" << std::endl;
    O << T0_w.rot.x << " " << T0_w.rot.y << " " << T0_w.rot.z << " " << T0_w.rot.w << " ";
    O << T0_w.trans.x << " " << T0_w.trans.y << " " << T0_w.trans.z << " ";
    O << std::endl;

    O << "Tw_e:" << std::endl;
    O << Tw_e.rot.x << " " << Tw_e.rot.y << " " << Tw_e.rot.z << " " << Tw_e.rot.w << " ";
    O << Tw_e.trans.x << " " << Tw_e.trans.y << " " << Tw_e.trans.z << " ";
    O << std::endl;

    O << "Bw:" << std::endl;
    O << Bw[0][0] << " ";
    O << Bw[0][1] << " ";
    O << std::endl;
    O << Bw[1][0] << " ";
    O << Bw[1][1] << " ";
    O << std::endl;
    O << Bw[2][0] << " ";
    O << Bw[2][1] << " ";
    O << std::endl;
    O << Bw[3][0] << " ";
    O << Bw[3][1] << " ";
    O << std::endl;
    O << Bw[4][0] << " ";
    O << Bw[4][1] << " ";
    O << std::endl;
    O << Bw[5][0] << " ";
    O << Bw[5][1] << " ";
    O << std::endl;
            RAVELOG_INFO(O.str().c_str());
}

bool TaskSpaceRegion::serialize(std::ostream &O, int type) const {
    if (type == 0) {     // default: simple format
        O << manipind << " ";

        if (prelativetolink.get() == nullptr) {
            O << "NULL" << " ";
        } else {
            O << prelativetolink->GetParent()->GetName() << " ";
            O << prelativetolink->GetName() << " ";
        }

        std::streamsize old_precision = O.precision();
        O.precision(2 + std::numeric_limits<OpenRAVE::dReal>::digits10);

        O << T0_w.rot.x << " " << T0_w.rot.y << " " << T0_w.rot.z << " " << T0_w.rot.w << " ";
        O << T0_w.trans.x << " " << T0_w.trans.y << " " << T0_w.trans.z << " ";

        O << Tw_e.rot.x << " " << Tw_e.rot.y << " " << Tw_e.rot.z << " " << Tw_e.rot.w << " ";
        O << Tw_e.trans.x << " " << Tw_e.trans.y << " " << Tw_e.trans.z << " ";

        O << Bw[0][0] << " ";
        O << Bw[0][1] << " ";

        O << Bw[1][0] << " ";
        O << Bw[1][1] << " ";

        O << Bw[2][0] << " ";
        O << Bw[2][1] << " ";

        O << Bw[3][0] << " ";
        O << Bw[3][1] << " ";

        O << Bw[4][0] << " ";
        O << Bw[4][1] << " ";

        O << Bw[5][0] << " ";
        O << Bw[5][1] << " ";

        O.precision(old_precision);
        return true;
    } else {  // xml format
        O << "<" << _tag_name;
        O << " manipulator_index=\"" << manipind << "\""
          << " relative_body_name=\"" << relativebodyname << "\""
          << " relative_link_name=\"" << relativelinkname << "\"";
        // T0_w matrix
        OpenRAVE::TransformMatrix temptm(T0_w);
        O << " T0_w=\""
          << temptm.m[0] << " "
          << temptm.m[4] << " "
          << temptm.m[8] << " "
          << temptm.m[1] << " "
          << temptm.m[5] << " "
          << temptm.m[9] << " "
          << temptm.m[2] << " "
          << temptm.m[6] << " "
          << temptm.m[10] << " "
          << temptm.trans.x << " "
          << temptm.trans.y << " "
          << temptm.trans.z << "\"";
        // Tw_e matrix
        OpenRAVE::TransformMatrix temptm2(Tw_e);
        O << " Tw_e=\""
          << temptm2.m[0] << " "
          << temptm2.m[4] << " "
          << temptm2.m[8] << " "
          << temptm2.m[1] << " "
          << temptm2.m[5] << " "
          << temptm2.m[9] << " "
          << temptm2.m[2] << " "
          << temptm2.m[6] << " "
          << temptm2.m[10] << " "
          << temptm2.trans.x << " "
          << temptm2.trans.y << " "
          << temptm2.trans.z << "\"";
        // Read in the Bw matrix
        O << " Bw=\"";
        for (unsigned int i = 0; i < 6; i++)
            for (unsigned int j = 0; j < 2; j++)
                if (j == 0 && i == 0)
                    O << Bw[i][j];
                else
                    O << " " << Bw[i][j];
        O << "\"";
        // close tag
        O << "/>";
        return true;
    }
}

bool TaskSpaceRegion::deserialize(std::stringstream &_ss) {
    //RAVELOG_INFO(_ss.str().c_str());
    _ss >> manipind;

    std::string tempstring;
    _ss >> relativebodyname;

    if (relativebodyname != "NULL") {
        _ss >> relativelinkname;
    }

    _ss >> T0_w.rot.x;
    _ss >> T0_w.rot.y;
    _ss >> T0_w.rot.z;
    _ss >> T0_w.rot.w;

    _ss >> T0_w.trans.x;
    _ss >> T0_w.trans.y;
    _ss >> T0_w.trans.z;

    _ss >> Tw_e.rot.x;
    _ss >> Tw_e.rot.y;
    _ss >> Tw_e.rot.z;
    _ss >> Tw_e.rot.w;

    _ss >> Tw_e.trans.x;
    _ss >> Tw_e.trans.y;
    _ss >> Tw_e.trans.z;

    _ss >> Bw[0][0];
    _ss >> Bw[0][1];

    _ss >> Bw[1][0];
    _ss >> Bw[1][1];

    _ss >> Bw[2][0];
    _ss >> Bw[2][1];

    _ss >> Bw[3][0];
    _ss >> Bw[3][1];

    _ss >> Bw[4][0];
    _ss >> Bw[4][1];

    _ss >> Bw[5][0];
    _ss >> Bw[5][1];

    //Print();
    return true;
}

bool TaskSpaceRegion::deserialize_from_matlab(const OpenRAVE::RobotBasePtr &robot_in, const OpenRAVE::EnvironmentBasePtr &penv_in, std::istream &_ss) {
    assert(robot_in.get() != nullptr);

    int tempint;
    std::string tempstring;
    OpenRAVE::TransformMatrix temptm;

    //get pointer to manipulator at this index
    _ss >> tempint;
    if (tempint >= robot_in->GetManipulators().size() || tempint < 0) {
                RAVELOG_INFO("Error: Manipulator index out of bounds\n");
        return false;
    } else
        manipind = tempint;

    _ss >> tempstring;
    if (strcasecmp(tempstring.c_str(), "NULL") == 0) {
        prelativetolink.reset();
    } else {
        OpenRAVE::KinBodyPtr pobject;
        pobject = penv_in->GetKinBody(tempstring);
        if (pobject.get() == nullptr) {
                    RAVELOG_INFO("Error: could not find the specified object to attach frame\n");
            return false;
        }
        _ss >> tempstring;
        //find the link
        std::vector<OpenRAVE::KinBody::LinkPtr> vlinks = pobject->GetLinks();
        bool bGotLink = false;
        for (auto &vlink : vlinks) {
            if (strcmp(tempstring.c_str(), vlink->GetName().c_str()) == 0) {
                        RAVELOG_INFO("frame link: %s:%s\n", vlink->GetParent()->GetName().c_str(), vlink->GetName().c_str());
                prelativetolink = vlink;
                bGotLink = true;
                break;
            }
        }
        if (!bGotLink) {
                    RAVELOG_INFO("Error: could not find the specified link of the object to attach frame\n");
            return false;
        }
    }

    _ss >> temptm.m[0];
    _ss >> temptm.m[4];
    _ss >> temptm.m[8];
    _ss >> temptm.m[1];
    _ss >> temptm.m[5];
    _ss >> temptm.m[9];
    _ss >> temptm.m[2];
    _ss >> temptm.m[6];
    _ss >> temptm.m[10];
    _ss >> temptm.trans.x;
    _ss >> temptm.trans.y;
    _ss >> temptm.trans.z;
    T0_w = OpenRAVE::Transform(temptm);

    _ss >> temptm.m[0];
    _ss >> temptm.m[4];
    _ss >> temptm.m[8];
    _ss >> temptm.m[1];
    _ss >> temptm.m[5];
    _ss >> temptm.m[9];
    _ss >> temptm.m[2];
    _ss >> temptm.m[6];
    _ss >> temptm.m[10];
    _ss >> temptm.trans.x;
    _ss >> temptm.trans.y;
    _ss >> temptm.trans.z;
    Tw_e = OpenRAVE::Transform(temptm);

    _ss >> Bw[0][0];
    _ss >> Bw[0][1];

    _ss >> Bw[1][0];
    _ss >> Bw[1][1];

    _ss >> Bw[2][0];
    _ss >> Bw[2][1];

    _ss >> Bw[3][0];
    _ss >> Bw[3][1];

    _ss >> Bw[4][0];
    _ss >> Bw[4][1];

    _ss >> Bw[5][0];
    _ss >> Bw[5][1];

    //Print();
    return true;
}

OpenRAVE::BaseXMLReader::ProcessElement TaskSpaceRegion::startElement(const std::string &name, const OpenRAVE::AttributesList &atts) {
    if (name == _tag_name) {
        if (_tag_open)
            return PE_Ignore;
        else {
            std::istringstream value;
            for (const auto &att:atts) {
                auto key = att.first;
                value.clear();
                value.str(att.second);
                if (key == "manipulator_index")
                    value >> manipind;
                else if (key == "relative_body_name")
                    value >> relativebodyname;
                else if (key == "relative_link_name")
                    value >> relativelinkname;
                else if (key == "t0_w") {
                    OpenRAVE::TransformMatrix temptm;
                    value >> temptm.m[0]
                          >> temptm.m[4]
                          >> temptm.m[8]
                          >> temptm.m[1]
                          >> temptm.m[5]
                          >> temptm.m[9]
                          >> temptm.m[2]
                          >> temptm.m[6]
                          >> temptm.m[10]
                          >> temptm.trans.x
                          >> temptm.trans.y
                          >> temptm.trans.z;
                    T0_w = OpenRAVE::Transform(temptm);
                } else if (key == "tw_e") {
                    OpenRAVE::TransformMatrix temptm;
                    value >> temptm.m[0]
                          >> temptm.m[4]
                          >> temptm.m[8]
                          >> temptm.m[1]
                          >> temptm.m[5]
                          >> temptm.m[9]
                          >> temptm.m[2]
                          >> temptm.m[6]
                          >> temptm.m[10]
                          >> temptm.trans.x
                          >> temptm.trans.y
                          >> temptm.trans.z;
                    Tw_e = OpenRAVE::Transform(temptm);
                } else if (key == "bw") {
                    // Read in the Bw matrix
                    for (auto &row : Bw)
                        for (double &element : row)
                            value >> element;
                } else
                            RAVELOG_WARN ("Unrecognized attribute %s.", key.c_str());
            }
            _tag_open = true;
            return PE_Support;
        }
    } else
        return PE_Pass;
}

bool TaskSpaceRegion::endElement(const std::string &name) {
    if (name == _tag_name) {
        _tag_open = false;
        return true;
    } else
        return false;
}
