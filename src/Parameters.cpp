//
// Created by jiangeng on 10/12/19.
//
#include "Parameters.h"

#include <openrave/openrave.h>
#include <ompl/base/ScopedState.h>
#include <sstream>

using namespace AtlasMPNet;

/*
 * implementation of SolverParameters
 */
SimpleXMLReader::ProcessElement SolverParameters::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    if (name == _tag_name) {
        if (_tag_open)
            return PE_Ignore;
        else {
            std::istringstream value;
            for (const auto &att:atts) {
                auto key = att.first;
                value.clear();
                value.str(att.second);
                if (key == "time")
                    value >> time_;
                else if (key == "range")
                    value >> range_;
                else if (key == "type") {
                    int tmp;
                    value >> tmp;
                    type_ = static_cast<SolverType>(tmp);
                } else
                            RAVELOG_WARN ("Unrecognized attribute %s.", key.c_str());
            }
            _tag_open = true;
            return PE_Support;
        }
    } else
        return PE_Pass;
}

bool SolverParameters::endElement(std::string const &name) {
    if (name == _tag_name) {
        _tag_open = false;
        return true;
    } else
        return false;
}

bool SolverParameters::serialize(std::ostream &O) const {
    O << "<" << _tag_name
      << " time=\"" << time_ << "\""
      << " range=\"" << range_ << "\""
      << "/>";
    return true;
}

/*
 * implementation of ConstraintParameters
 */
SimpleXMLReader::ProcessElement ConstraintParameters::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    if (name == _tag_name) {
        if (_tag_open)
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
                    int tmp;
                    value >> tmp;
                    type_ = static_cast<SpaceType>(tmp);
                } else
                            RAVELOG_WARN ("Unrecognized attribute %s.", key.c_str());
            }
            _tag_open = true;
            return PE_Support;
        }
    } else
        return PE_Pass;
}

bool ConstraintParameters::endElement(std::string const &name) {
    if (name == _tag_name) {
        _tag_open = false;
        return true;
    } else
        return false;
}

bool ConstraintParameters::serialize(std::ostream &O) const {
    O << "<" << _tag_name
      << " type=\"" << type_ << "\""
      << " tolerance=\"" << tolerance_ << "\""
      << " max_iter=\"" << max_iter_ << "\""
      << " delta=\"" << delta_ << "\""
      << " lambda=\"" << lambda_ << "\""
      << "/>";
}

/*
 * implementation of AtlasParameters
 */

SimpleXMLReader::ProcessElement AtlasParameters::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    if (name == _tag_name) {
        if (_tag_open)
            return PE_Ignore;
        else {
            std::istringstream value;
            for (const auto &att:atts) {
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
                            RAVELOG_WARN ("Unrecognized attribute %s.", key.c_str());
            }
            _tag_open = true;
            return PE_Support;
        }
    } else
        return PE_Pass;
}

bool AtlasParameters::endElement(std::string const &name) {
    if (name == _tag_name) {
        _tag_open = false;
        return true;
    } else
        return false;
}

bool AtlasParameters::serialize(std::ostream &O) const {
    O << "<" << _tag_name
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

SimpleXMLReader::ProcessElement TSRParameters::startElement(const std::string &name, const std::list<std::pair<std::string, std::string> > &atts) {
    if (name == _tag_name) {
        if (_tag_open)
            return PE_Ignore;
        else {
            std::istringstream value;
            for (const auto &att:atts) {
                auto key = att.first;
                value.clear();
                value.str(att.second);
                if (key == "t0_w") {
                    OpenRAVE::TransformMatrix temptm;
                    value >> temptm.m[0] >> temptm.m[4] >> temptm.m[8]
                          >> temptm.m[1] >> temptm.m[5] >> temptm.m[9]
                          >> temptm.m[2] >> temptm.m[6] >> temptm.m[10]
                          >> temptm.trans.x >> temptm.trans.y >> temptm.trans.z;
                    T0_w = OpenRAVE::Transform(temptm);
                } else if (key == "tw_e") {
                    OpenRAVE::TransformMatrix temptm;
                    value >> temptm.m[0] >> temptm.m[4] >> temptm.m[8]
                          >> temptm.m[1] >> temptm.m[5] >> temptm.m[9]
                          >> temptm.m[2] >> temptm.m[6] >> temptm.m[10]
                          >> temptm.trans.x >> temptm.trans.y >> temptm.trans.z;
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

bool TSRParameters::endElement(const std::string &name) {
    if (name == _tag_name) {
        _tag_open = false;
        return true;
    } else
        return false;
}

bool TSRParameters::serialize(std::ostream &O) const {
    O << "<" << _tag_name;
    // T0_w matrix
    OpenRAVE::TransformMatrix temptm(T0_w);
    O << " T0_w=\""
      << temptm.m[0] << " " << temptm.m[4] << " " << temptm.m[8] << " "
      << temptm.m[1] << " " << temptm.m[5] << " " << temptm.m[9] << " "
      << temptm.m[2] << " " << temptm.m[6] << " " << temptm.m[10] << " "
      << temptm.trans.x << " " << temptm.trans.y << " " << temptm.trans.z << "\"";
    // Tw_e matrix
    OpenRAVE::TransformMatrix temptm2(Tw_e);
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
                O << Bw[i][j];
            else
                O << " " << Bw[i][j];
    O << "\"/>";
    return true;
}

SimpleXMLReader::ProcessElement TSRChainParameters::startElement(const std::string &name, const std::list<std::pair<std::string, std::string> > &atts) {
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
                if (key == "manipulator_index")
                    value >> manipind;
                else if (key == "relative_body_name")
                    value >> relativebodyname;
                else if (key == "relative_link_name")
                    value >> relativelinkname;
                else if (key == "purpose") {
                    int temp;
                    value >> temp;
                    purpose = static_cast<PurposeType>(temp);
                } else if (key == "mimic_body_name") {
                    value >> mimic_body_name;
                } else if (key == "mimic_body_index") {
                    int temp;
                    while (!value.eof()) {
                        temp = -1;
                        value >> temp;
                        if (temp == -1) {
                                    RAVELOG_ERROR ("Unexpected character.");
                            break;
                        }
                        mimic_inds.emplace_back(temp);
                    }
                } else
                            RAVELOG_WARN ("Unrecognized attribute %s.", key.c_str());
            }
            _tag_open = true;
            return PE_Support;
        }
    } else if (name == "tsr") {
        if (_tag_open) {
            temp_tsr.startElement(name, atts);
            return PE_Support;
        } else {
                    RAVELOG_WARN("TSR cannot be placed outside TSRChain tags.");
            return PE_Ignore;
        }
    } else {
        return PE_Pass;
    }
}

bool TSRChainParameters::endElement(const std::string &name) {
    if (name == _tag_name) {
        _tag_open = false;
        return true;
    } else if (name == "tsr") {
        if (_tag_open) {
            temp_tsr.endElement(name);
            TSRs.emplace_back(temp_tsr);
            temp_tsr.reset();
        }
        return false;
    } else {
        return false;
    }
}

bool TSRChainParameters::serialize(std::ostream &O) const {
    O << "<" << _tag_name << " ";
    O << "purpose=\"" << purpose << "\" "
      << "manipulator_index=\"" << manipind << "\" "
      << "relative_body_name=\"" << relativebodyname << "\" "
      << "relative_link_name=\"" << relativelinkname << "\" ";
    O << "mimic_body_name=\"" << mimic_body_name << "\" ";
    O << "mimic_body_index=\"";
    for (int i = 0; i < mimic_inds.size(); i++) {
        if (i == 0)
            O << mimic_inds[i];
        else
            O << " " << mimic_inds[i];
    }
    O << "\">" << std::endl;
    for (const auto &tsr:TSRs) {
        O << tsr << std::endl;
    }
    O << "</" << _tag_name << ">" << std::endl;
    return true;
}

/*
 * implementation of PlannnerParameters
 */
Parameters::Parameters() : OpenRAVE::PlannerBase::PlannerParameters() {
    _vXMLParameters.emplace_back(solver_parameters_.getTagName());
    _vXMLParameters.emplace_back(constraint_parameters_.getTagName());
    _vXMLParameters.emplace_back(atlas_parameters_.getTagName());
    _vXMLParameters.emplace_back(tsrchain_parameters_.getTagName());
    _vXMLParameters.emplace_back("tsr");    // Avoid hard code the tag names. Improve this part later
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
    for (int i = 0; i < dof; i++) {
        goal[i] = vgoalconfig[i];
    }
    return true;
}

bool Parameters::serialize(std::ostream &O, int options) const {
    if (!OpenRAVE::PlannerBase::PlannerParameters::serialize(O, options)) {
        return false;
    }
    O << solver_parameters_ << std::endl
      << constraint_parameters_ << std::endl
      << atlas_parameters_ << std::endl
      << tsrchain_parameters_ << std::endl;
    return !!O;
}

OpenRAVE::BaseXMLReader::ProcessElement Parameters::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    OpenRAVE::BaseXMLReader::ProcessElement status;
    if ((status = OpenRAVE::PlannerBase::PlannerParameters::startElement(name, atts)) != PE_Pass)
        return status;
    if ((status = solver_parameters_.startElement(name, atts)) != PE_Pass)
        return status;
    if ((status = constraint_parameters_.startElement(name, atts)) != PE_Pass)
        return status;
    if ((status = atlas_parameters_.startElement(name, atts)) != PE_Pass)
        return status;
    if ((status = tsrchain_parameters_.startElement(name, atts)) != PE_Pass)
        return status;
}

bool Parameters::endElement(std::string const &name) {
    return !tsrchain_parameters_.endElement(name) &&
           !atlas_parameters_.endElement(name) &&
           !constraint_parameters_.endElement(name) &&
           !solver_parameters_.endElement(name) &&
           PlannerParameters::endElement(name);
}

