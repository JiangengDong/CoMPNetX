//
// Created by jiangeng on 10/12/19.
//
#include "Parameters.h"

#include <Eigen/Dense>
#include <ompl/base/ScopedState.h>
#include <openrave/openrave.h>
#include <sstream>

using namespace CoMPNetX;

/*
 * implementation of SolverParameters
 */
SimpleXMLReader::ProcessElement SolverParameter::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    const static std::map<std::string, SolverType> str2type = {
        {"rrt", RRT},
        {"rrtstar", RRTstar},
        {"rrtconnect", RRTConnect},
        {"compnet", CoMPNet},
        {"compnetx", CoMPNetX}};
    if (name == tag_name_) {
        if (tag_open_)
            return PE_Ignore;
        else {
            std::istringstream value;
            for (const auto &att : atts) {
                auto key = att.first;
                value.clear();
                value.str(att.second);
                if (key == "time")
                    value >> time_;
                else if (key == "range")
                    value >> range_;
                else if (key == "type") {
                    type_ = str2type.at(value.str());
                } else
                    RAVELOG_WARN("Unrecognized attribute %s.", key.c_str());
            }
            tag_open_ = true;
            return PE_Support;
        }
    } else
        return PE_Pass;
}

bool SolverParameter::endElement(std::string const &name) {
    if (name == tag_name_) {
        tag_open_ = false;
        return true;
    } else
        return false;
}

bool SolverParameter::serialize(std::ostream &O) const {
    const static std::map<SolverType, std::string> type2str = {
        {RRT, "rrt"},
        {RRTstar, "rrtstar"},
        {RRTConnect, "rrtconnect"},
        {CoMPNet, "compnet"},
        {CoMPNetX, "compnetx"}};
    O << "<" << tag_name_
      << " type=\"" << type2str.at(type_) << "\""
      << " time=\"" << time_ << "\""
      << " range=\"" << range_ << "\""
      << "/>";
    return true;
}

/*
 * implementation of ConstraintParameters
 */
SimpleXMLReader::ProcessElement ConstraintParameter::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    const static std::map<std::string, SpaceType> str2type = {
        {"projection", PROJECTION},
        {"proj", PROJECTION},
        {"atlas", ATLAS},
        {"tangent_bundle", TANGENT_BUNDLE},
        {"tangent-bundle", TANGENT_BUNDLE},
        {"tb", TANGENT_BUNDLE}};
    if (name == tag_name_) {
        if (tag_open_)
            return PE_Ignore;
        else {
            std::istringstream value;
            for (const auto &att : atts) {
                auto key = att.first;
                value.clear();
                value.str(att.second);
                if (key == "tolerance")
                    value >> tolerance_;
                else if (key == "max_iter")
                    value >> max_iter_;
                else if (key == "delta")
                    value >> delta_;
                else if (key == "lambda")
                    value >> lambda_;
                else if (key == "type") {
                    type_ = str2type.at(value.str());
                } else
                    RAVELOG_WARN("Unrecognized attribute %s.", key.c_str());
            }
            tag_open_ = true;
            return PE_Support;
        }
    } else
        return PE_Pass;
}

bool ConstraintParameter::endElement(std::string const &name) {
    if (name == tag_name_) {
        tag_open_ = false;
        return true;
    } else
        return false;
}

bool ConstraintParameter::serialize(std::ostream &O) const {
    const static std::map<SpaceType, std::string> type2str = {
        {PROJECTION, "projection"},
        {ATLAS, "atlas"},
        {TANGENT_BUNDLE, "tangent_bundle"}};
    O << "<" << tag_name_
      << " type=\"" << type2str.at(type_) << "\""
      << " tolerance=\"" << tolerance_ << "\""
      << " max_iter=\"" << max_iter_ << "\""
      << " delta=\"" << delta_ << "\""
      << " lambda=\"" << lambda_ << "\""
      << "/>";
    return true;
}

/*
 * implementation of AtlasParameters
 */

SimpleXMLReader::ProcessElement AtlasParameter::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    if (name == tag_name_) {
        if (tag_open_)
            return PE_Ignore;
        else {
            std::istringstream value;
            for (const auto &att : atts) {
                auto key = att.first;
                value.clear();
                value.str(att.second);
                if (key == "exploration")
                    value >> exploration_;
                else if (key == "epsilon")
                    value >> epsilon_;
                else if (key == "rho")
                    value >> rho_;
                else if (key == "alpha")
                    value >> alpha_;
                else if (key == "max_charts")
                    value >> max_charts_;
                else if (key == "using_bias")
                    value >> using_bias_;
                else if (key == "separate")
                    value >> separate_;
                else
                    RAVELOG_WARN("Unrecognized attribute %s.", key.c_str());
            }
            tag_open_ = true;
            return PE_Support;
        }
    } else
        return PE_Pass;
}

bool AtlasParameter::endElement(std::string const &name) {
    if (name == tag_name_) {
        tag_open_ = false;
        return true;
    } else
        return false;
}

bool AtlasParameter::serialize(std::ostream &O) const {
    O << "<" << tag_name_
      << " exploration=\"" << exploration_ << "\""
      << " epsilon=\"" << epsilon_ << "\""
      << " rho=\"" << rho_ << "\""
      << " alpha=\"" << alpha_ << "\""
      << " max_charts=\"" << max_charts_ << "\""
      << " using_bias=\"" << using_bias_ << "\""
      << " separate=\"" << separate_ << "\""
      << "/>";
    return true;
}

SimpleXMLReader::ProcessElement TSRParameter::startElement(const std::string &name, const std::list<std::pair<std::string, std::string>> &atts) {
    if (name == tag_name_) {
        if (tag_open_)
            return PE_Ignore;
        else {
            std::istringstream value;
            for (const auto &att : atts) {
                auto key = att.first;
                value.clear();
                value.str(att.second);
                if (key == "t0_w") {
                    OpenRAVE::TransformMatrix temptm;
                    value >> temptm.m[0] >> temptm.m[4] >> temptm.m[8] >> temptm.m[1] >> temptm.m[5] >> temptm.m[9] >> temptm.m[2] >> temptm.m[6] >> temptm.m[10] >> temptm.trans.x >> temptm.trans.y >> temptm.trans.z;
                    T0_w_ = OpenRAVE::Transform(temptm);
                } else if (key == "tw_e") {
                    OpenRAVE::TransformMatrix temptm;
                    value >> temptm.m[0] >> temptm.m[4] >> temptm.m[8] >> temptm.m[1] >> temptm.m[5] >> temptm.m[9] >> temptm.m[2] >> temptm.m[6] >> temptm.m[10] >> temptm.trans.x >> temptm.trans.y >> temptm.trans.z;
                    Tw_e_ = OpenRAVE::Transform(temptm);
                } else if (key == "bw") {
                    // Read in the Bw matrix
                    for (auto &row : Bw_)
                        for (double &element : row)
                            value >> element;
                } else
                    RAVELOG_WARN("Unrecognized attribute %s.", key.c_str());
            }
            tag_open_ = true;
            return PE_Support;
        }
    } else
        return PE_Pass;
}

bool TSRParameter::endElement(const std::string &name) {
    if (name == tag_name_) {
        tag_open_ = false;
        return true;
    } else
        return false;
}

bool TSRParameter::serialize(std::ostream &O) const {
    O << "<" << tag_name_;
    // T0_w matrix
    OpenRAVE::TransformMatrix temptm(T0_w_);
    O << " T0_w=\""
      << temptm.m[0] << " " << temptm.m[4] << " " << temptm.m[8] << " "
      << temptm.m[1] << " " << temptm.m[5] << " " << temptm.m[9] << " "
      << temptm.m[2] << " " << temptm.m[6] << " " << temptm.m[10] << " "
      << temptm.trans.x << " " << temptm.trans.y << " " << temptm.trans.z << "\"";
    // Tw_e matrix
    OpenRAVE::TransformMatrix temptm2(Tw_e_);
    O << " Tw_e=\""
      << temptm2.m[0] << " " << temptm2.m[4] << " " << temptm2.m[8] << " "
      << temptm2.m[1] << " " << temptm2.m[5] << " " << temptm2.m[9] << " "
      << temptm2.m[2] << " " << temptm2.m[6] << " " << temptm2.m[10] << " "
      << temptm2.trans.x << " " << temptm2.trans.y << " " << temptm2.trans.z << "\"";
    // Read in the Bw matrix
    O << " Bw=\"";
    for (unsigned int i = 0; i < 6; i++)
        for (unsigned int j = 0; j < 2; j++)
            if (j == 0 && i == 0)
                O << Bw_[i][j];
            else
                O << " " << Bw_[i][j];
    O << "\"/>";
    return true;
}

SimpleXMLReader::ProcessElement TSRChainParameter::startElement(const std::string &name, const std::list<std::pair<std::string, std::string>> &atts) {
    const static std::map<std::string, PurposeType> str2type = {
        {"constraint", CONSTRAINT},
        {"sample_start", SAMPLE_START},
        {"sample_goal", SAMPLE_GOAL}};
    if (name == tag_name_) {
        if (tag_open_) {
            return PE_Ignore;
        } else {
            std::istringstream value;
            for (const auto &att : atts) {
                auto key = att.first;
                value.clear();
                value.str(att.second);
                if (key == "manipulator_index")
                    value >> manipind_;
                else if (key == "relative_body_name")
                    value >> relativebodyname_;
                else if (key == "relative_link_name")
                    value >> relativelinkname_;
                else if (key == "purpose") {
                    purpose_ = str2type.at(value.str());
                } else if (key == "mimic_body_name") {
                    value >> mimic_body_name_;
                } else if (key == "mimic_body_index") {
                    if (att.second.empty())
                        continue;
                    int temp;
                    while (!value.eof()) {
                        temp = -1;
                        value >> temp;
                        if (temp == -1) {
                            RAVELOG_ERROR("Unexpected character.");
                            break;
                        }
                        mimic_inds_.emplace_back(temp);
                    }
                } else
                    RAVELOG_WARN("Unrecognized attribute %s.", key.c_str());
            }
            tag_open_ = true;
            return PE_Support;
        }
    } else if (name == "tsr") {
        if (tag_open_) {
            temp_tsr_.startElement(name, atts);
            return PE_Support;
        } else {
            RAVELOG_WARN("TSR cannot be placed outside TSRChain tags.");
            return PE_Ignore;
        }
    } else {
        return PE_Pass;
    }
}

bool TSRChainParameter::endElement(const std::string &name) {
    if (name == tag_name_) {
        if (tag_open_) {
            tag_open_ = false;
            return true;
        } else
            return false;
    } else if (name == "tsr") {
        if (tag_open_) {
            temp_tsr_.endElement(name);
            TSRs_.emplace_back(temp_tsr_);
            temp_tsr_.reset();
        }
        return false;
    } else {
        return false;
    }
}

bool TSRChainParameter::serialize(std::ostream &O) const {
    O << "<" << tag_name_ << " ";
    O << "purpose=\"" << purpose_ << "\" "
      << "manipulator_index=\"" << manipind_ << "\" "
      << "relative_body_name=\"" << relativebodyname_ << "\" "
      << "relative_link_name=\"" << relativelinkname_ << "\" ";
    O << "mimic_body_name=\"" << mimic_body_name_ << "\" ";
    O << "mimic_body_index=\"";
    for (unsigned int i = 0; i < mimic_inds_.size(); i++) {
        if (i == 0)
            O << mimic_inds_[i];
        else
            O << " " << mimic_inds_[i];
    }
    O << "\">" << std::endl;
    for (const auto &tsr : TSRs_) {
        O << tsr << std::endl;
    }
    O << "</" << tag_name_ << ">" << std::endl;
    return true;
}

bool TSRChainParameter::reset() {
    purpose_ = CONSTRAINT;
    manipind_ = -1;
    relativebodyname_ = "NULL";
    relativelinkname_ = "NULL";
    mimic_body_name_ = "NULL";
    mimic_inds_.clear();
    TSRs_.clear();
    return true;
}

OpenRAVE::BaseXMLReader::ProcessElement MPNetParameter::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    if (name == tag_name_) {
        if (tag_open_)
            return PE_Ignore;
        else {
            std::istringstream value;
            for (const auto &att : atts) {
                auto key = att.first;
                value.clear();
                value.str(att.second);
                if (key == "pnet_path") {
                    value >> pnet_path_;
                } else if (key == "dnet_path") {
                    value >> dnet_path_;
                    if (dnet_path_ != "")
                        use_dnet_ = true;
                } else if (key == "dnet_threshold") {
                    value >> dnet_threshold_;
                } else if (key == "dnet_coeff") {
                    value >> dnet_coeff_;
                } else if (key == "predict_tsr") {
                    if (value.str() == "true")
                        use_tsr_ = true;
                    else
                        use_tsr_ = false;
                } else if (key == "voxel_path") {
                    voxel_path_ = att.second;
                } else if (key == "ohot_path") {
                    ohot_path_ = att.second;
                } else
                    RAVELOG_WARN("Unrecognized attribute %s.", key.c_str());
            }
            tag_open_ = true;
            return PE_Support;
        }
    } else
        return PE_Pass;
}

bool MPNetParameter::endElement(std::string const &name) {
    if (name == tag_name_) {
        tag_open_ = false;
        return true;
    } else
        return false;
}

bool MPNetParameter::serialize(std::ostream &O) const {
    O << "<" << tag_name_
      << " pnet_path=\"" << pnet_path_ << "\""
      << " voxel_path=\"" << voxel_path_ << "\""
      << " ohot_path=\"" << ohot_path_ << "\""
      << " dnet_path=\"" << dnet_path_ << "\""
      << " dnet_threshold=\"" << dnet_threshold_ << "\""
      << " dnet_coeff=\"" << dnet_coeff_ << "\""
      << " predict_tsr=\"" << (use_tsr_ ? "true" : "false") << "\""
      << "/>";
    return true;
}

/*
 * implementation of PlannnerParameters
 */
Parameters::Parameters() : OpenRAVE::PlannerBase::PlannerParameters() {
    _vXMLParameters.emplace_back(solver_parameter_.getTagName());
    _vXMLParameters.emplace_back(constraint_parameter_.getTagName());
    _vXMLParameters.emplace_back(atlas_parameter_.getTagName());
    _vXMLParameters.emplace_back(_tsrchain_temp.getTagName());
    _vXMLParameters.emplace_back(mpnet_parameter_.getTagName());
    _vXMLParameters.emplace_back("tsr");        // Avoid hard code the tag names. Improve this part later
    _vXMLParameters.emplace_back("tsr_chains"); // Avoid hard code the tag names. Improve this part later
}

Parameters::~Parameters() = default;

bool Parameters::getStartState(double *start) const {
    const int dof = GetDOF();
    for (int i = 0; i < dof; i++) {
        start[i] = vinitialconfig[i];
    }
    return true;
}

bool Parameters::getStartState(ompl::base::ScopedState<> &start) const {
    const int dof = GetDOF();
    for (int i = 0; i < dof; i++) {
        start[i] = vinitialconfig[i];
    }
    return true;
}

bool Parameters::getStartState(std::vector<double> &start) const {
    const int dof = GetDOF();
    start.clear();
    start.resize(dof);
    for (int i = 0; i < dof; i++) {
        start[i] = vinitialconfig[i];
    }
    return true;
}

bool Parameters::getGoalState(double *goal) const {
    const int dof = GetDOF();
    for (int i = 0; i < dof; i++) {
        goal[i] = vgoalconfig[i];
    }
    return true;
}

bool Parameters::getGoalState(ompl::base::ScopedState<> &goal) const {
    const int dof = GetDOF();
    for (int i = 0; i < dof; i++) {
        goal[i] = vgoalconfig[i];
    }
    return true;
}

bool Parameters::getGoalState(std::vector<double> &goal) const {
    const int dof = GetDOF();
    goal.clear();
    goal.resize(dof);
    for (int i = 0; i < dof; i++) {
        goal[i] = vgoalconfig[i];
    }
    return true;
}

bool Parameters::serialize(std::ostream &O, int options) const {
    if (!OpenRAVE::PlannerBase::PlannerParameters::serialize(O, options)) {
        return false;
    }
    O << solver_parameter_ << std::endl
      << constraint_parameter_ << std::endl
      << atlas_parameter_ << std::endl;
    for (const auto &tsr_chain : tsrchains_) {
        O << tsr_chain << std::endl;
    }
    O << mpnet_parameter_ << std::endl;
    return !!O;
}

OpenRAVE::BaseXMLReader::ProcessElement Parameters::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    OpenRAVE::BaseXMLReader::ProcessElement status;
    if ((status = OpenRAVE::PlannerBase::PlannerParameters::startElement(name, atts)) != PE_Pass)
        return status;
    if ((status = solver_parameter_.startElement(name, atts)) != PE_Pass)
        return status;
    if ((status = constraint_parameter_.startElement(name, atts)) != PE_Pass)
        return status;
    if ((status = atlas_parameter_.startElement(name, atts)) != PE_Pass)
        return status;
    if ((status = _tsrchain_temp.startElement(name, atts)) != PE_Pass)
        return status;
    if ((status = mpnet_parameter_.startElement(name, atts)) != PE_Pass)
        return status;
    if (name == "tsr_chains")
        return PE_Support;
    return PE_Pass;
}

bool Parameters::endElement(std::string const &name) {
    if (_tsrchain_temp.endElement(name)) {
        tsrchains_.emplace_back(_tsrchain_temp);
        _tsrchain_temp.reset();
        return false;
    } else if (name == "tsr_chains") {
        return false;
    } else
        return !atlas_parameter_.endElement(name) &&
               !constraint_parameter_.endElement(name) &&
               !solver_parameter_.endElement(name) &&
               !mpnet_parameter_.endElement(name) &&
               PlannerParameters::endElement(name);
}
