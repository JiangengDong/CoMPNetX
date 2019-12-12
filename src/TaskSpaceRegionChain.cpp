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

#include "TaskSpaceRegion.h"

using namespace AtlasMPNet;

bool TaskSpaceRegionChain::Initialize(const OpenRAVE::EnvironmentBasePtr &penv_in) {
    _sumbounds = 0;
    for (int i = 0; i < TSRChain.size(); i++) {
        if (!TSRChain[i].Initialize(penv_in)) {
                    RAVELOG_INFO("Error: Failed to initialize TSR %d\n", i);
            return false;
        }

        _sumbounds += TSRChain[i].GetSumOfBounds();
    }
    if (_sumbounds == 0)//for point TSRs
        _sumbounds = 0.001;

    if (strcasecmp(mimicbodyname.c_str(), "NULL") == 0) {
        _pmimicbody.reset();
    } else {
        _pmimicbody = penv_in->GetRobot(mimicbodyname);
        if (_pmimicbody.get() == nullptr) {
                    RAVELOG_INFO("Error: could not find the specified kinbody to make a mimic\n");
            return false;
        }

    }
    return true;
}

bool TaskSpaceRegionChain::RobotizeTSRChain(const OpenRAVE::EnvironmentBasePtr &penv_in, OpenRAVE::RobotBasePtr &probot_out) {
    bool bFlipAxis;
    if (penv_in.get() == nullptr) {
                RAVELOG_INFO("Environment pointer is null!\n");
        probot_out = OpenRAVE::RobotBasePtr();
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
        probot_out = OpenRAVE::RobotBasePtr();
        return false;
    }

    if (_pmimicbody != nullptr) {
        _mimicjointvals_temp.resize(_pmimicbody->GetDOF());
        //mimic body may not be starting at 0 position for all joints, so get the joint offsets
        _mimicjointoffsets.resize(_pmimicbody->GetDOF());
        _pmimicbody->GetDOFValues(_mimicjointoffsets);
    }

    //now write out the XML string for this robot

    std::stringstream O;

    O << "<?xml version=\"1.0\" encoding=\"utf-8\"?>" << std::endl;
    O << "<Robot name=\"" << robotname << "\">" << std::endl;
    O << "\t<KinBody>" << std::endl;

    O << "\t\t<Body name = \"Body0\" type=\"dynamic\">" << std::endl;
    O << "\t\t\t<Geom type=\"sphere\">" << std::endl;
    O << "\t\t\t\t<Radius>0.01</Radius>" << std::endl;
    O << "\t\t\t\t<diffusecolor>0.3 0.7 0.3</diffusecolor>" << std::endl;
    O << "\t\t\t</Geom>" << std::endl;
    O << "\t\t</Body>" << std::endl;

    int bodynumber = 1;
    Tw0_e = OpenRAVE::Transform();

    for (auto &i : TSRChain) {
        for (int j = 0; j < 6; j++) {
            bFlipAxis = false;
            //don't add a body if there is no freedom in this dimension
            if (i.Bw[j][0] == 0 && i.Bw[j][1] == 0)
                continue;

            //TODO: If the bounds are equal and non-zero, do something reasonable
            if (i.Bw[j][0] == i.Bw[j][1]) {
                        RAVELOG_FATAL("ERROR: TSR Chains are currently unable to deal with cases where two bounds are equal but non-zero, cannot robotize.\n");
                probot_out = OpenRAVE::RobotBasePtr();
                return false;
            }

            //check for axis flip, this is marked by the Bw values being backwards
            if (i.Bw[j][0] > i.Bw[j][1]) {
                i.Bw[j][0] = -i.Bw[j][0];
                i.Bw[j][1] = -i.Bw[j][1];
                bFlipAxis = true;
            }

            //now take care of joint offsets if we are mimicing
            if (_pmimicbody.get() != nullptr) {
                //we may only be mimicing some of the joints of the TSR
                if (bodynumber - 1 < _mimicinds.size()) {
                    i.Bw[j][0] = i.Bw[j][0] - _mimicjointoffsets[_mimicinds[bodynumber - 1]];
                    i.Bw[j][1] = i.Bw[j][1] - _mimicjointoffsets[_mimicinds[bodynumber - 1]];
                }
            }

            _lowerlimits.push_back(i.Bw[j][0]);
            _upperlimits.push_back(i.Bw[j][1]);

            O << "\t\t<Body name = \"Body" << bodynumber << "\" type=\"dynamic\">" << std::endl;
            O << "\t\t\t<offsetfrom>Body0</offsetfrom>" << std::endl;
            O << "\t\t\t<Translation>" << Tw0_e.trans.x << " " << Tw0_e.trans.y << " " << Tw0_e.trans.z << "</Translation>" << std::endl;
            O << "\t\t\t<Quat>" << Tw0_e.rot.x << " " << Tw0_e.rot.y << " " << Tw0_e.rot.z << " " << Tw0_e.rot.w << "</Quat>" << std::endl;

            if (j < 3)
                O << "\t\t\t<Geom type=\"box\">" << std::endl;
            else
                O << "\t\t\t<Geom type=\"cylinder\">" << std::endl;

            switch (j) {
                case 0:
                    O << "\t\t\t\t<extents>0.04 0.02 0.02</extents>" << std::endl;
                    break;
                case 1:
                    O << "\t\t\t\t<extents>0.02 0.04 0.02</extents>" << std::endl;
                    break;
                case 2:
                    O << "\t\t\t\t<extents>0.02 0.02 0.04</extents>" << std::endl;
                    break;
                case 3:
                    O << "\t\t\t\t<RotationAxis>0 0 1 90</RotationAxis>" << std::endl;
                    O << "\t\t\t\t<Radius>0.03</Radius>" << std::endl;
                    O << "\t\t\t\t<Height>0.07</Height>" << std::endl;
                    break;
                case 4:
                    O << "\t\t\t\t<Radius>0.03</Radius>" << std::endl;
                    O << "\t\t\t\t<Height>0.07</Height>" << std::endl;
                    break;
                case 5:
                    O << "\t\t\t\t<RotationAxis>1 0 0 90</RotationAxis>" << std::endl;
                    O << "\t\t\t\t<Radius>0.03</Radius>" << std::endl;
                    O << "\t\t\t\t<Height>0.07</Height>" << std::endl;
                    break;
                default:
                    break;
            }

            O << "\t\t\t\t<diffusecolor>0.3 0.7 0.3</diffusecolor>" << std::endl;
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

            O << "\t\t\t<limits>" << i.Bw[j][0] << " " << i.Bw[j][1] << "</limits>" << std::endl;

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
        Tw0_e = Tw0_e * i.Tw_e;
    }

    //now add a geometry to the last body with the offset of the last TSR, this will be the target for the manipulator
    //NOTE: this is appending a body, not making a new one
    OpenRAVE::Transform Told = Tw0_e;
    Tw0_e = TSRChain[TSRChain.size() - 1].Tw_e;

    O << "\t\t<Body name = \"Body" << bodynumber - 1 << "\" type=\"dynamic\">" << std::endl;
    O << "\t\t\t<Geom type=\"sphere\">" << std::endl;
    O << "\t\t\t\t<Translation>" << Tw0_e.trans.x << " " << Tw0_e.trans.y << " " << Tw0_e.trans.z << "</Translation>" << std::endl;
    O << "\t\t\t\t<Quat>" << Tw0_e.rot.x << " " << Tw0_e.rot.y << " " << Tw0_e.rot.z << " " << Tw0_e.rot.w << "</Quat>" << std::endl;
    O << "\t\t\t\t<Radius>0.03</Radius>" << std::endl;
    O << "\t\t\t\t<diffusecolor>0.3 0.7 0.3</diffusecolor>" << std::endl;
    O << "\t\t\t</Geom>" << std::endl;
    O << "\t\t</Body>" << std::endl;


    O << "\t</KinBody>" << std::endl;


    if (bodynumber > 1) {
        _bPointTSR = false;
        //finally, write out the manipulator parameters
        O << "\t<Manipulator name=\"dummy\">" << std::endl;
        O << "\t\t<base>Body0</base>" << std::endl;
        O << "\t\t<effector>Body" << bodynumber - 1 << "</effector>" << std::endl;
        O << "\t</Manipulator>" << std::endl;
    } else {
        _bPointTSR = true;
        numdof = bodynumber - 1;
                RAVELOG_INFO("This is a point TSR, no robotized TSR needed\n");
        probot_out = OpenRAVE::RobotBasePtr();
        return true;
    }

    if (_bPointTSR && TSRChain.size() != 1) {
                RAVELOG_INFO("Can't yet handle case where the tsr chain has no freedom but multiple TSRs, try making it a chain of length 1\n");
        probot_out = OpenRAVE::RobotBasePtr();
        return false;
    }


    O << "</Robot>" << std::endl;

    robot = penv->ReadRobotXMLData(OpenRAVE::RobotBasePtr(), O.str(), std::list<std::pair<std::string, std::string> >());

    if (robot.get() == nullptr) {
                RAVELOG_INFO("Could not init robot from data!\n");
        probot_out = OpenRAVE::RobotBasePtr();
        return false;
    }

    if (robot.get() == nullptr) {
                RAVELOG_INFO("Robot is NULL!\n");
        probot_out = OpenRAVE::RobotBasePtr();
        return false;
    }

    robot->SetName(robotname);
    penv_in->Add(robot, true);

    if (TSRChain[0].prelativetolink.get() == nullptr)
        robot->SetTransform(TSRChain[0].T0_w);
    else
        robot->SetTransform(TSRChain[0].prelativetolink->GetTransform() * TSRChain[0].T0_w);


    _pIkSolver = RaveCreateIkSolver(penv_in, "GeneralIK");

    if (_pIkSolver.get() == nullptr) {
                RAVELOG_INFO("Cannot create IK solver, make sure you have the GeneralIK plugin loadable by openrave\n");
        probot_out = OpenRAVE::RobotBasePtr();
        return false;
    }

    OpenRAVE::RobotBase::ManipulatorPtr pmanip = robot->GetActiveManipulator();
    _pIkSolver->Init(pmanip);
    numdof = bodynumber - 1;

    //initialize parameters to send to ik solver
    ikparams.resize(12);
    ikparams[0] = 1;
    ikparams[1] = robot->GetActiveManipulatorIndex();
    ikparams[9] = 0; //don't do any balancing
    ikparams[10] = 0; //select the mode
    ikparams[11] = 0; //do rotation

    //note that openrave will creat traj files for each TSR in your .openrave directory, you may want to remove them

    robot->Enable(false);

    probot_out = robot;
    return true;
}

OpenRAVE::dReal TaskSpaceRegionChain::TransformDifference(const OpenRAVE::Transform &tm_ref, const OpenRAVE::Transform &tm_targ) const {
    _tmtemp = tm_ref.inverse() * tm_targ;

    _dx[0] = _tmtemp.trans.x;
    _dx[1] = _tmtemp.trans.y;
    _dx[2] = _tmtemp.trans.z;

    TSRChain[0].QuatToRPY(&_tmtemp.rot[0], _dx[3], _dx[4], _dx[5]);

    _sumsqr = 0;
    for (int i = 0; i < 6; i++)
        _sumsqr += _dx[i] * _dx[i];
    //RAVELOG_INFO("dx: %f %f %f %f %f %F\n",dx[0],dx[1],dx[2],dx[3],dx[4],dx[5]);

    return sqrt(_sumsqr);
}

OpenRAVE::dReal TaskSpaceRegionChain::GetClosestTransform(const OpenRAVE::Transform &T0_s, OpenRAVE::dReal *TSRJointVals, OpenRAVE::Transform &T0_closeset) const {
    if (_bPointTSR || TSRChain.size() == 1) {
        T0_closeset = TSRChain[0].GetClosestTransform(T0_s);
        return TSRChain[0].DistanceToTSR(T0_s, _dx);
    }

    assert(robot.get() != nullptr);

    assert(TSRJointVals != nullptr);


    OpenRAVE::Transform Ttarg = T0_s * TSRChain.back().Tw_e.inverse();

    ikparams[2] = Ttarg.rot.x;
    ikparams[3] = Ttarg.rot.y;
    ikparams[4] = Ttarg.rot.z;
    ikparams[5] = Ttarg.rot.w;
    ikparams[6] = Ttarg.trans.x;
    ikparams[7] = Ttarg.trans.y;
    ikparams[8] = Ttarg.trans.z;


    std::vector<OpenRAVE::dReal> q0, q_s;
    q_s.resize(robot->GetDOF());
    robot->GetDOFValues(q_s);

    //RAVELOG_INFO("TSRJointVals: %f %f\n",TSRJointVals[0],TSRJointVals[1]);

    for (int i = 0; i < q_s.size(); i++)
        q_s[i] = TSRJointVals[i];

    q0 = q_s;
    boost::shared_ptr<std::vector<OpenRAVE::dReal> > pq_s(new std::vector<OpenRAVE::dReal>);

    _pIkSolver->Solve(OpenRAVE::IkParameterization(), q0, ikparams, false, pq_s);
    q_s = *pq_s;

    robot->SetJointValues(q_s, true);
    for (int i = 0; i < q_s.size(); i++)
        TSRJointVals[i] = q_s[i];

    //RAVELOG_INFO("TSRJointVals: %f %f\n",TSRJointVals[0],TSRJointVals[1]);
    OpenRAVE::Transform Ttemp = robot->GetActiveManipulator()->GetEndEffectorTransform();
    T0_closeset =  Ttemp * TSRChain.back().Tw_e;
    return TransformDifference(Ttarg, Ttemp);
}

bool TaskSpaceRegionChain::serialize(std::ostream &O, int type) const {
    if (type == 0) {     // simple format
        O << " ";

        O << (int) bSampleStartFromChain << " ";
        O << (int) bSampleGoalFromChain << " ";

        O << (int) bConstrainToChain << " ";


        O << TSRChain.size();
        for (const auto &i : TSRChain) {
            O << " ";
            if (!i.serialize(O)) {
                        RAVELOG_INFO("ERROR SERIALIZING TSR CHAIN\n");
                return false;
            }
            O << " ";
        }

        if (_pmimicbody.get() == nullptr)
            O << " " << "NULL" << " ";
        else
            O << " " << _pmimicbody->GetName() << " ";


        O << " " << _mimicinds.size() << " ";
        for (int _mimicind : _mimicinds) {
            O << " " << _mimicind << " ";
        }

        return true;
    } else {      // xml format
        O << "<" << _tag_name << " ";
        O << "purpose=\""
          << (int) bSampleStartFromChain << " "
          << (int) bSampleGoalFromChain << " "
          << (int) bConstrainToChain << "\" ";
        O << "mimic_body_name=\""
          << mimicbodyname << "\" ";
        O << "mimic_body_index=\"";
        for (int i = 0; i < _mimicinds.size(); i++) {
            if (i == 0)
                O << _mimicinds[i];
            else
                O << " " << _mimicinds[i];
        }
        O << "\">" << std::endl;
        for (const auto &tsr:TSRChain) {
            O << tsr << std::endl;
        }
        O << "</" << _tag_name << ">" << std::endl;
        return true;
    }
}

bool TaskSpaceRegionChain::deserialize(std::stringstream &_ss) {

    _ss >> bSampleStartFromChain;
    _ss >> bSampleGoalFromChain;
    _ss >> bConstrainToChain;

    if (!bSampleGoalFromChain && !bSampleStartFromChain && !bConstrainToChain)
                RAVELOG_INFO ("WARNING: This chain is not sampled or constrained to, are you sure you defined it correctly?\n");

    int temp;
    _ss >> temp;
    TSRChain.resize(temp);


    for (int i = 0; i < temp; i++) {
        if (!TSRChain[i].deserialize(_ss)) {
                    RAVELOG_INFO("ERROR DESERIALIZING TSR CHAIN\n");
            return false;
        }
    }

    _ss >> mimicbodyname;

    if (mimicbodyname != "NULL") {
        _ss >> temp;
        _mimicinds.resize(temp);
        for (int i = 0; i < temp; i++) {
            _ss >> _mimicinds[i];
        }
    }

    return true;
}

bool TaskSpaceRegionChain::deserialize_from_matlab(const OpenRAVE::RobotBasePtr &robot_in, const OpenRAVE::EnvironmentBasePtr &penv_in, std::istream &_ss) {
    _ss >> bSampleStartFromChain;
    _ss >> bSampleGoalFromChain;
    _ss >> bConstrainToChain;

    if (!bSampleGoalFromChain && !bSampleStartFromChain && !bConstrainToChain)
                RAVELOG_INFO ("WARNING: This chain is not sampled or constrained to, are you sure you defined it correctly?\n");


    int temp;
    std::string tempstring;

    _ss >> temp;

    TSRChain.resize(temp);

    for (int i = 0; i < temp; i++) {
        if (!TSRChain[i].deserialize_from_matlab(robot_in, penv_in, _ss)) {
                    RAVELOG_INFO("ERROR DESERIALIZING TSR CHAIN FROM MATLAB\n");
            return false;
        }
    }

    _ss >> tempstring;
    if (strcasecmp(tempstring.c_str(), "NULL") == 0) {
        _pmimicbody.reset();
    } else {
        _pmimicbody = penv_in->GetRobot(tempstring);
        if (_pmimicbody.get() == nullptr) {
                    RAVELOG_INFO("Error: could not find the specified mimic body\n");
            return false;
        }

        _ss >> temp;
        _mimicinds.resize(temp);
        for (int i = 0; i < temp; i++) {
            _ss >> _mimicinds[i];
        }
    }
    return true;
}

OpenRAVE::BaseXMLReader::ProcessElement TaskSpaceRegionChain::startElement(const std::string &name, const OpenRAVE::AttributesList &atts) {
    if (name == _tag_name) {
        if (_tag_open) {
            return PE_Ignore;
        } else {
            std::istringstream value;
            for (const auto &att:atts) {
                auto key = att.first;
                value.clear();
                auto value_str = att.second;
                value.str(value_str.erase(value_str.find_last_not_of(' ') + 1));
                if (key == "purpose") {
                    value >> bSampleStartFromChain >> bSampleGoalFromChain >> bConstrainToChain;
                    if (!bSampleGoalFromChain && !bSampleStartFromChain && !bConstrainToChain)
                                RAVELOG_INFO ("WARNING: This chain is not sampled or constrained to, are you sure you defined it correctly?\n");
                } else if (key == "mimic_body_name") {
                    value >> mimicbodyname;
                } else if (key == "mimic_body_index") {
                    int temp;
                    while (!value.eof()) {
                        temp = -1;
                        value >> temp;
                        if (temp == -1) {
                                    RAVELOG_ERROR ("Unexpected character.");
                            break;
                        }
                        _mimicinds.emplace_back(temp);
                    }
                } else
                            RAVELOG_WARN ("Unrecognized attribute %s.", key.c_str());
            }
            _tag_open = true;
            return PE_Support;
        }
    } else if (name == "tsr") {
        if (_tag_open) {
            _temp_tsr.startElement(name, atts);
            return PE_Support;
        } else {
                    RAVELOG_WARN("TSR cannot be placed outside TSRChain tags.");
            return PE_Ignore;
        }
    } else {
        return PE_Pass;
    }
}

bool TaskSpaceRegionChain::endElement(const std::string &name) {
    if (name == _tag_name) {
        _tag_open = false;
        return true;
    } else if (name == "tsr") {
        if (_tag_open) {
            _temp_tsr.endElement(name);
            TSRChain.emplace_back(_temp_tsr);
        }
        return false;
    } else {
        return false;
    }
}

OpenRAVE::Transform TaskSpaceRegionChain::GenerateSample(){
    for (int i = 1; i < TSRChain.size(); i++)
        TSRChain[i].T0_w = TSRChain[i - 1].GenerateSample();

    return TSRChain[TSRChain.size() - 1].GenerateSample();
}

bool TaskSpaceRegionChain::GetChainJointLimits(OpenRAVE::dReal *lowerlimits, OpenRAVE::dReal *upperlimits) const {
    for (int i = 0; i < _lowerlimits.size(); i++) {
        lowerlimits[i] = _lowerlimits[i];
        upperlimits[i] = _upperlimits[i];
                RAVELOG_DEBUG("lower: %f   upper: %f\n", lowerlimits[i], upperlimits[i]);
    }

    return true;
}

bool TaskSpaceRegionChain::ExtractMimicDOFValues(const OpenRAVE::dReal *TSRValues, OpenRAVE::dReal *MimicDOFVals) const {
    //NOTE: THIS ASSUMES MIMIC INDS ARE THE 1ST N DOF OF THE CHAIN, THIS MAY CHANGE!!!!
    for (int i = 0; i < _mimicinds.size(); i++)
        MimicDOFVals[i] = TSRValues[i];


    return true;
}

//NOTE: THIS ASSUMES MIMIC INDS ARE THE 1ST N DOF OF THE CHAIN, THIS MAY CHANGE!!!!
bool TaskSpaceRegionChain::MimicValuesToFullMimicBodyValues(const OpenRAVE::dReal *TSRJointVals, std::vector<OpenRAVE::dReal> &mimicbodyvals) {
    if (_pmimicbody == nullptr)
        return false;

    mimicbodyvals.resize(_pmimicbody->GetDOF());
    _pmimicbody->GetDOFValues(mimicbodyvals);
    for (int i = 0; i < _mimicinds.size(); i++) {
        mimicbodyvals[_mimicinds[i]] = _mimicjointoffsets[_mimicinds[i]] + TSRJointVals[i];
    }

    return true;
}

bool TaskSpaceRegionChain::ApplyMimicValuesToMimicBody(const OpenRAVE::dReal *TSRJointVals) {
    if (_pmimicbody == nullptr)
        return false;

    MimicValuesToFullMimicBodyValues(TSRJointVals, _mimicjointvals_temp);

    _pmimicbody->SetJointValues(_mimicjointvals_temp, true);
    return true;
}

void TaskSpaceRegionChain::DestoryRobotizedTSRChain() {
    if (robot != nullptr) {
        penv->Remove(robot);
    }
}
