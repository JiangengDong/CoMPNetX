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
                else
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
                else
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
                else if (key == "using_tb")
                    value >> using_tb_;
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
      << " using_tb=\"" << using_tb_ << "\""
      << " separate=\"" << separate_ << "\""
      << "/>";
    return true;
}

/*
 * implementation of PlannnerParameters
 */
Parameters::Parameters() : OpenRAVE::PlannerBase::PlannerParameters() {
    _vXMLParameters.emplace_back("planner_parameters");
    _vXMLParameters.emplace_back("constraint_parameters");
    _vXMLParameters.emplace_back("atlas_parameters");
    _vXMLParameters.emplace_back("tsr_chain");
    _vXMLParameters.emplace_back("tsr");
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
    O << planner_parameters_ << std::endl
      << constraint_parameters_ << std::endl
      << atlas_parameters_ << std::endl
      << tsrchain_parameters_ << std::endl;
    return !!O;
}

OpenRAVE::BaseXMLReader::ProcessElement Parameters::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    OpenRAVE::BaseXMLReader::ProcessElement status;
    if ((status = OpenRAVE::PlannerBase::PlannerParameters::startElement(name, atts)) != PE_Pass)
        return status;
    if ((status = planner_parameters_.startElement(name, atts)) != PE_Pass)
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
           !planner_parameters_.endElement(name) &&
           PlannerParameters::endElement(name);
}

