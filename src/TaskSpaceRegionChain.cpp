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

#include "TaskSpaceRegionChain.h"
#include <openrave/openrave.h>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <Eigen/Dense>

using namespace AtlasMPNet;

TaskSpaceRegionChain::TaskSpaceRegionChain(const OpenRAVE::EnvironmentBasePtr &penv_in, const TSRChainParameter &param) {
    this->param = param;
    _mimic_inds = this->param.mimic_inds;
    numdof = -1;
    _bPointTSR = false;

    // find the mimic body according to its name
    if (strcasecmp(param.mimic_body_name.c_str(), "NULL") == 0) {
        _mimicbody.reset();
    } else {
        _mimicbody = penv_in->GetRobot(param.mimic_body_name);
        if (_mimicbody.get() == nullptr) {
                    RAVELOG_INFO("Error: could not find the specified kinbody to make a mimic\n");
        }
    }

    // find the relative-to link according to its body name and link name
    if (strcasecmp(param.relativebodyname.c_str(), "NULL") == 0) {
        prelativetolink.reset();
    } else {
        OpenRAVE::KinBodyPtr pobject;
        pobject = penv_in->GetKinBody(param.relativebodyname);
        if (pobject.get() == nullptr) {
                    RAVELOG_INFO("Error: could not find the specified object to attach frame\n");
        }

        //find the link
        std::vector<OpenRAVE::KinBody::LinkPtr> vlinks = pobject->GetLinks();
        bool bGotLink = false;
        for (auto &vlink : vlinks) {
            if (strcmp(param.relativelinkname.c_str(), vlink->GetName().c_str()) == 0) {
                        RAVELOG_INFO("frame link: %s:%s\n", vlink->GetParent()->GetName().c_str(), vlink->GetName().c_str());
                prelativetolink = vlink;
                bGotLink = true;
                break;
            }
        }
        if (!bGotLink) {
                    RAVELOG_INFO("Error: could not find the specified link of the object to attach frame\n");
        }
    }
    RobotizeTSRChain(penv_in);
}

bool TaskSpaceRegionChain::RobotizeTSRChain(const OpenRAVE::EnvironmentBasePtr &penv_in) {
    bool bFlipAxis;
    if (penv_in.get() == nullptr) {
                RAVELOG_INFO("Environment pointer is null!\n");
        return false;
    }

    _lowerlimits.resize(0);
    _upperlimits.resize(0);

    //store this pointer for later robot destruction
    penv = penv_in;

    char robotname[32], xmlfile[256], robottype[32];
    sprintf(robottype, "GenericRobot");
    sprintf(xmlfile, "TSRChain%lu.robot.xml", (unsigned long int) this);
    sprintf(robotname, "TSRChain%lu", (unsigned long int) this);//give a unique name to this robot

    robot = RaveCreateRobot(penv_in, robottype);
    if (robot.get() == nullptr) {
                RAVELOG_INFO("Failed to create robot %s", robottype);
        return false;
    }

    if (_mimicbody != nullptr) {
        _mimicjointvals_temp.resize(_mimicbody->GetDOF());
        //mimic body may not be starting at 0 position for all joints, so get the joint offsets
        _mimicjointoffsets.resize(_mimicbody->GetDOF());
        _mimicbody->GetDOFValues(_mimicjointoffsets);
    }

    //now write out the XML string for this robot

    std::stringstream O;

    O << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << std::endl;
    O << "<Robot name=\"" << robotname << "\">" << std::endl;
    O << "\t<KinBody>" << std::endl;

    O << "\t\t<Body name = \"Body0\" type=\"dynamic\" enable=\"false\">" << std::endl;
    O << "\t\t\t<Geom type=\"sphere\">" << std::endl;
    O << "\t\t\t\t<Radius>0.01</Radius>" << std::endl;
    O << "\t\t\t\t<diffusecolor>0.3 0.7 0.3</diffusecolor>" << std::endl;
    O << "\t\t\t</Geom>" << std::endl;
    O << "\t\t</Body>" << std::endl;

    unsigned int bodynumber = 1;
    OpenRAVE::Transform Tw0_e = OpenRAVE::Transform();

    for (auto &tsr : param.TSRs) {
        for (int j = 0; j < 6; j++) {
            bFlipAxis = false;
            //don't add a body if there is no freedom in this dimension
            if (tsr.Bw[j][0] == 0 && tsr.Bw[j][1] == 0)
                continue;

            if (tsr.Bw[j][0] == tsr.Bw[j][1]) {
                        RAVELOG_FATAL(
                        "ERROR: TSR Chains are currently unable to deal with cases where two bounds are equal but non-zero, cannot robotize.\n");
                return false;
            }

            //check for axis flip, this is marked by the Bw values being backwards
            if (tsr.Bw[j][0] > tsr.Bw[j][1]) {
                tsr.Bw[j][0] = -tsr.Bw[j][0];
                tsr.Bw[j][1] = -tsr.Bw[j][1];
                bFlipAxis = true;
            }

            //now take care of joint offsets if we are mimicing
            if (_mimicbody.get() != nullptr) {
                //we may only be mimicing some of the joints of the TSR
                if (bodynumber - 1 < _mimic_inds.size()) {
                    tsr.Bw[j][0] = tsr.Bw[j][0] - _mimicjointoffsets[_mimic_inds[bodynumber - 1]];
                    tsr.Bw[j][1] = tsr.Bw[j][1] - _mimicjointoffsets[_mimic_inds[bodynumber - 1]];
                }
            }

            _lowerlimits.push_back(tsr.Bw[j][0]);
            _upperlimits.push_back(tsr.Bw[j][1]);

            O << "\t\t<Body name = \"Body" << bodynumber << "\" type=\"dynamic\" enable=\"false\">" << std::endl;
            O << "\t\t\t<offsetfrom>Body0</offsetfrom>" << std::endl;
            O << "\t\t\t<Translation>" << Tw0_e.trans.x << " " << Tw0_e.trans.y << " " << Tw0_e.trans.z << "</Translation>" << std::endl;
            O << "\t\t\t<Quat>" << Tw0_e.rot.x << " " << Tw0_e.rot.y << " " << Tw0_e.rot.z << " " << Tw0_e.rot.w << "</Quat>" << std::endl;

            if (j < 3)
                O << "\t\t\t<Geom type=\"box\">" << std::endl;
            else
                O << "\t\t\t<Geom type=\"cylinder\">" << std::endl;

            switch (j) {
                case 0:
                    O << "\t\t\t\t<extents>0.06 0.02 0.02</extents>" << std::endl;
                    break;
                case 1:
                    O << "\t\t\t\t<extents>0.02 0.06 0.02</extents>" << std::endl;
                    break;
                case 2:
                    O << "\t\t\t\t<extents>0.02 0.02 0.06</extents>" << std::endl;
                    break;
                case 3:
                    O << "\t\t\t\t<RotationAxis>0 0 1 90</RotationAxis>" << std::endl;
                    O << "\t\t\t\t<Radius>0.02</Radius>" << std::endl;
                    O << "\t\t\t\t<Height>0.08</Height>" << std::endl;
                    break;
                case 4:
                    O << "\t\t\t\t<Radius>0.02</Radius>" << std::endl;
                    O << "\t\t\t\t<Height>0.08</Height>" << std::endl;
                    break;
                case 5:
                    O << "\t\t\t\t<RotationAxis>1 0 0 90</RotationAxis>" << std::endl;
                    O << "\t\t\t\t<Radius>0.02</Radius>" << std::endl;
                    O << "\t\t\t\t<Height>0.08</Height>" << std::endl;
                    break;
                default:
                    break;
            }
            if (j < 3)
                O << "\t\t\t\t<diffusecolor>0.7 0.3 0.3</diffusecolor>" << std::endl;
            else
                O << "\t\t\t\t<diffusecolor>0.3 0.3 0.7</diffusecolor>" << std::endl;
            O << "\t\t\t</Geom>" << std::endl;

            O << "\t\t</Body>" << std::endl;

            if (j < 3)
                O << "\t\t<Joint name=\"J" << bodynumber << "\" type=\"slider\">" << std::endl;
            else
                O << "\t\t<Joint name=\"J" << bodynumber << "\" type=\"hinge\">" << std::endl;

            O << "\t\t\t<Body>Body" << bodynumber - 1 << "</Body>" << std::endl;
            O << "\t\t\t<Body>Body" << bodynumber << "</Body>" << std::endl;
            O << "\t\t\t<offsetfrom>Body" << bodynumber << "</offsetfrom>" << std::endl;
            O << "\t\t\t<weight>1</weight>" << std::endl;
            O << "\t\t\t<maxvel>1</maxvel>" << std::endl;
            O << "\t\t\t<resolution>1</resolution>" << std::endl;

            O << "\t\t\t<limits>" << tsr.Bw[j][0] << " " << tsr.Bw[j][1] << "</limits>" << std::endl;

            switch (j) {
                case 0:
                    O << "\t\t\t<axis>1 0 0</axis>" << std::endl;
                    break;
                case 1:
                    O << "\t\t\t<axis>0 1 0</axis>" << std::endl;
                    break;
                case 2:
                    O << "\t\t\t<axis>0 0 1</axis>" << std::endl;
                    break;
                case 3:
                    if (bFlipAxis)
                        O << "\t\t\t<axis>-1 0 0</axis>" << std::endl;
                    else
                        O << "\t\t\t<axis>1 0 0</axis>" << std::endl;
                    break;
                case 4:
                    if (bFlipAxis)
                        O << "\t\t\t<axis>0 -1 0</axis>" << std::endl;
                    else
                        O << "\t\t\t<axis>0 1 0</axis>" << std::endl;
                    break;
                case 5:
                    if (bFlipAxis)
                        O << "\t\t\t<axis>0 0 -1</axis>" << std::endl;
                    else
                        O << "\t\t\t<axis>0 0 1</axis>" << std::endl;
                    break;
                default:
                    break;
            }

            O << "\t\t</Joint>" << std::endl;

            bodynumber++;
        }
        Tw0_e = Tw0_e * tsr.Tw_e;
    }

    O << "\t\t<Body name = \"Body" << bodynumber << "\" type=\"dynamic\" enable=\"false\">" << std::endl;
    O << "\t\t\t<offsetfrom>Body0</offsetfrom>" << std::endl;
    O << "\t\t\t<Translation>" << Tw0_e.trans.x << " " << Tw0_e.trans.y << " " << Tw0_e.trans.z << "</Translation>" << std::endl;
    O << "\t\t\t<Quat>" << Tw0_e.rot.x << " " << Tw0_e.rot.y << " " << Tw0_e.rot.z << " " << Tw0_e.rot.w << "</Quat>" << std::endl;
    O << "\t\t\t<Geom type=\"sphere\">" << std::endl;
    O << "\t\t\t\t<Radius>0.03</Radius>" << std::endl;
    O << "\t\t\t\t<diffusecolor>0.3 0.7 0.3</diffusecolor>" << std::endl;
    O << "\t\t\t</Geom>" << std::endl;
    O << "\t\t</Body>" << std::endl;

    O << "\t\t<Joint name=\"J" << bodynumber << "\" type=\"hinge\" enable=\"false\">" << std::endl;
    O << "\t\t\t<Body>Body" << bodynumber - 1 << "</Body>" << std::endl;
    O << "\t\t\t<Body>Body" << bodynumber << "</Body>" << std::endl;
    O << "\t\t\t<offsetfrom>Body" << bodynumber << "</offsetfrom>" << std::endl;
    O << "\t\t\t<limits>" << "0 0" << "</limits>" << std::endl;
    O << "\t\t</Joint>" << std::endl;

    O << "\t</KinBody>" << std::endl;


    if (bodynumber > 1) {
        _bPointTSR = false;
        //finally, write out the manipulator parameters
        O << "\t<Manipulator name=\"dummy\">" << std::endl;
        O << "\t\t<base>Body0</base>" << std::endl;
        O << "\t\t<effector>Body" << bodynumber << "</effector>" << std::endl;
        O << "\t</Manipulator>" << std::endl;
    } else {
        _bPointTSR = true;
        numdof = bodynumber - 1;
                RAVELOG_INFO("This is a point TSR, no robotized TSR needed\n");
        return true;
    }

    if (_bPointTSR && param.TSRs.size() != 1) {
                RAVELOG_INFO("Can't yet handle case where the tsr chain has no freedom but multiple TSRs, try making it a chain of length 1\n");
        return false;
    }

    O << "</Robot>" << std::endl;

    robot = penv->ReadRobotXMLData(OpenRAVE::RobotBasePtr(), O.str(), std::list<std::pair<std::string, std::string> >());
    if (robot.get() == nullptr) {
                RAVELOG_INFO("Could not init robot from data!\n");
        return false;
    }

    robot->SetName(robotname);
    penv_in->Add(robot, true);

    if (prelativetolink.get() == nullptr)
        robot->SetTransform(param.TSRs[0].T0_w);
    else
        robot->SetTransform(prelativetolink->GetTransform() * param.TSRs[0].T0_w);    // TODO: this is not the relative that I think

    OpenRAVE::RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    robot->SetActiveDOFs(pmanip->GetArmIndices());
    numdof = bodynumber - 1;
    robot->Enable(false);
    return true;
}

OpenRAVE::dReal
TaskSpaceRegionChain::GetClosestTransform(const OpenRAVE::Transform &T0_s, std::vector<OpenRAVE::dReal> &TSRJointVals, OpenRAVE::Transform &T0_closeset) const {
    Eigen::MatrixXd J(6, numdof);
    Eigen::VectorXd p(6), q(numdof);
    std::vector<OpenRAVE::dReal> Jtrans, Jrot;
    OpenRAVE::Transform Tdiff;
    double squaredNorm = 1000;
    int eeIndex = robot->GetActiveManipulator()->GetEndEffector()->GetIndex();

    for (int it = 0; it < 50; it++) {
        // enforce bound
        for (int i = 0; i < numdof; i++) {
            if (TSRJointVals[i] > _upperlimits[i])
                TSRJointVals[i] = _upperlimits[i];
            else if (TSRJointVals[i] < _lowerlimits[i])
                TSRJointVals[i] = _lowerlimits[i];
        }
        // forward kinematic to check if reached
        robot->SetActiveDOFValues(TSRJointVals);
        T0_closeset = robot->GetActiveManipulator()->GetEndEffectorTransform();
        Tdiff = T0_s.inverse() * T0_closeset;
        squaredNorm = Tdiff.trans.lengthsqr3() + Tdiff.rot.y * Tdiff.rot.y + Tdiff.rot.z * Tdiff.rot.z + Tdiff.rot.w * Tdiff.rot.w;
        if (squaredNorm < 1e-16)
            break;
        // move one step
        robot->CalculateActiveJacobian(eeIndex, T0_closeset.trans, Jtrans);
        robot->CalculateActiveRotationJacobian(eeIndex, T0_closeset.rot, Jrot);
        // copy to Eigen
        p[0] = T0_closeset.trans.x - T0_s.trans.x;
        p[1] = T0_closeset.trans.y - T0_s.trans.y;
        p[2] = T0_closeset.trans.z - T0_s.trans.z;
        p[3] = Tdiff.rot.y;
        p[4] = Tdiff.rot.z;
        p[5] = Tdiff.rot.w;
        for (int col = 0; col < numdof; col++) {
            for (int row = 0; row < 3; row++) {
                J(row, col) = Jtrans[row * numdof + col];
            }
            J(3, col) = T0_s.rot[0] * Jrot[1 * numdof + col]
                        - T0_s.rot[1] * Jrot[0 * numdof + col]
                        - T0_s.rot[2] * Jrot[3 * numdof + col]
                        + T0_s.rot[3] * Jrot[2 * numdof + col];
            J(4, col) = T0_s.rot[0] * Jrot[2 * numdof + col]
                        + T0_s.rot[1] * Jrot[3 * numdof + col]
                        - T0_s.rot[2] * Jrot[0 * numdof + col]
                        - T0_s.rot[3] * Jrot[1 * numdof + col];
            J(5, col) = T0_s.rot[0] * Jrot[3 * numdof + col]
                        - T0_s.rot[1] * Jrot[2 * numdof + col]
                        + T0_s.rot[2] * Jrot[1 * numdof + col]
                        - T0_s.rot[3] * Jrot[0 * numdof + col];
        }
        q = J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(p);
        for (int i = 0; i < numdof; i++) {
            TSRJointVals[i] -= q[i];
        }
        // TODO: maybe we need line-search here
    }
    return sqrt(squaredNorm);
}

bool TaskSpaceRegionChain::GetChainJointLimits(OpenRAVE::dReal *lowerlimits, OpenRAVE::dReal *upperlimits) const {
    for (unsigned int i = 0; i < _lowerlimits.size(); i++) {
        lowerlimits[i] = _lowerlimits[i];
        upperlimits[i] = _upperlimits[i];
                RAVELOG_DEBUG("lower: %f   upper: %f\n", lowerlimits[i], upperlimits[i]);
    }

    return true;
}

bool TaskSpaceRegionChain::GetChainJointLimits(std::vector<OpenRAVE::dReal> &lowerlimits, std::vector<OpenRAVE::dReal> &upperlimits) const {
    for (unsigned int i = 0; i < _lowerlimits.size(); i++) {
        lowerlimits[i] = _lowerlimits[i];
        upperlimits[i] = _upperlimits[i];
                RAVELOG_DEBUG("lower: %f   upper: %f\n", lowerlimits[i], upperlimits[i]);
    }

    return true;
}

bool TaskSpaceRegionChain::ExtractMimicDOFValues(const OpenRAVE::dReal *TSRValues, OpenRAVE::dReal *MimicDOFVals) const {
    //NOTE: THIS ASSUMES MIMIC INDS ARE THE 1ST N DOF OF THE CHAIN, THIS MAY CHANGE!!!!
    for (unsigned int i = 0; i < _mimic_inds.size(); i++)
        MimicDOFVals[i] = TSRValues[i];

    return true;
}

//NOTE: THIS ASSUMES MIMIC INDS ARE THE 1ST N DOF OF THE CHAIN, THIS MAY CHANGE!!!!
bool TaskSpaceRegionChain::MimicValuesToFullMimicBodyValues(const OpenRAVE::dReal *TSRJointVals, std::vector<OpenRAVE::dReal> &mimicbodyvals) {
    if (_mimicbody == nullptr)
        return false;

    mimicbodyvals.resize(_mimicbody->GetDOF());
    _mimicbody->GetDOFValues(mimicbodyvals);
    for (unsigned int i = 0; i < _mimic_inds.size(); i++) {
        mimicbodyvals[_mimic_inds[i]] = _mimicjointoffsets[_mimic_inds[i]] + TSRJointVals[i];
    }
    return true;
}

bool TaskSpaceRegionChain::ApplyMimicValuesToMimicBody(const OpenRAVE::dReal *TSRJointVals) {
    if (_mimicbody == nullptr)
        return false;
    MimicValuesToFullMimicBodyValues(TSRJointVals, _mimicjointvals_temp);
    _mimicbody->SetJointValues(_mimicjointvals_temp, true);
    return true;
}

void TaskSpaceRegionChain::DestoryRobotizedTSRChain() {
    if (robot != nullptr) {
        penv->Remove(robot);
    }
}
