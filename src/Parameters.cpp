//
// Created by jiangeng on 10/12/19.
//
#include "Parameters.h"

#include <openrave/openrave.h>
#include <ompl/base/ScopedState.h>
#include <sstream>

using namespace AtlasMPNet;

std::ostream &AtlasMPNet::operator<<(std::ostream &O, const SolverParameters &v) {
    O << "<planner_parameters"
      << " time=\"" << v.time_ << "\""
      << " range=\"" << v.range_ << "\""
      << "/>";
    return O;
}

bool SolverParameters::deserialize(std::list<std::pair<std::string, std::string>> const &atts) {
    std::istringstream value;
    for (const auto &att:atts) {
        auto name = att.first;
        value.clear();
        value.str(att.second);
        if (name == "time")
            value >> time_;
        else if (name == "range")
            value >> range_;
        else
                    RAVELOG_WARN ("Unrecognized attribute.");
    }
    return true;
}

std::ostream &AtlasMPNet::operator<<(std::ostream &O, const ConstraintParameters &v) {
    O << "<constraint_parameters"
      << " tolerance=\"" << v.tolerance_ << "\""
      << " max_iter=\"" << v.max_iter_ << "\""
      << " delta=\"" << v.delta_ << "\""
      << " lambda=\"" << v.lambda_ << "\""
      << "/>";
    return O;
}

bool ConstraintParameters::deserialize(std::list<std::pair<std::string, std::string>> const &atts) {
    std::istringstream value;
    for (const auto &att : atts) {
        auto name = att.first;
        value.clear();
        value.str(att.second);
        if (name == "tolerance")
            value >> tolerance_;
        else if (name == "max_iter")
            value >> max_iter_;
        else if (name == "delta")
            value >> delta_;
        else if (name == "lambda")
            value >> lambda_;
        else
                    RAVELOG_WARN ("Unrecognized attribute.");
    }
    return true;
}

std::ostream &AtlasMPNet::operator<<(std::ostream &O, const AtlasParameters &v) {
    O << "<atlas_parameters"
      << " exploration=\"" << v.exploration_ << "\""
      << " epsilon=\"" << v.epsilon_ << "\""
      << " rho=\"" << v.rho_ << "\""
      << " alpha=\"" << v.alpha_ << "\""
      << " max_charts=\"" << v.max_charts_ << "\""
      << " using_bias=\"" << v.using_bias_ << "\""
      << " using_tb=\"" << v.using_tb_ << "\""
      << " separate=\"" << v.separate_ << "\""
      << "/>";
    return O;
}

bool AtlasParameters::deserialize(std::list<std::pair<std::string, std::string>> const &atts) {
    std::istringstream value;
    for (const auto &att:atts) {
        auto name = att.first;
        value.clear();
        value.str(att.second);
        if (name == "exploration")
            value >> exploration_;
        else if (name == "epsilon")
            value >> epsilon_;
        else if (name == "rho")
            value >> rho_;
        else if (name == "alpha")
            value >> alpha_;
        else if (name == "max_charts")
            value >> max_charts_;
        else if (name == "using_bias")
            value >> using_bias_;
        else if (name == "using_tb")
            value >> using_tb_;
        else if (name == "separate")
            value >> separate_;
        else
                    RAVELOG_WARN ("Unrecognized attribute.");
    }
    return true;
}

Parameters::Parameters() : OpenRAVE::PlannerBase::PlannerParameters() {
    _vXMLParameters.emplace_back("planner_parameters");
    _vXMLParameters.emplace_back("constraint_parameters");
    _vXMLParameters.emplace_back("atlas_parameters");
}

Parameters::~Parameters() = default;

bool Parameters::getStartState(ompl::base::ScopedState<> &start) const {
    const int dof = GetDOF();
    for (int i = 0; i < dof; i++) {
        start[i] = vinitialconfig[i];
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

bool Parameters::serialize(std::ostream &O, int options=0) const {
    if (!OpenRAVE::PlannerBase::PlannerParameters::serialize(O, options)) {
        return false;
    }
    O << planner_parameters_ << std::endl
      << constraint_parameters_ << std::endl
      << atlas_parameters_ << std::endl;
    return !!O;
}

OpenRAVE::BaseXMLReader::ProcessElement Parameters::startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) {
    if (_tagOpen) {
        return PE_Ignore;
    }
    switch (OpenRAVE::PlannerBase::PlannerParameters::startElement(name, atts)) {
        case PE_Support:
            return PE_Support;
        case PE_Ignore:
            return PE_Ignore;
        case PE_Pass:
            break;
    }
    if (name == "planner_parameters")
        _tagOpen = planner_parameters_.deserialize(atts);
    else if (name == "constraint_parameters")
        _tagOpen = constraint_parameters_.deserialize(atts);
    else if (name == "atlas_parameters")
        _tagOpen = atlas_parameters_.deserialize(atts);
    else
        return PE_Pass;
    return _tagOpen ? PE_Support : PE_Pass;
}

bool Parameters::endElement(std::string const &name) {
    if (name == "planner_parameters" || name == "constraint_parameters" || name == "atlas_parameters") {
        if (!_ss.str().empty()) {
            std::ostringstream info;
            info << "The " << name << " tag should not have children.";
                    RAVELOG_WARN(info.str());
        }
        _tagOpen = false;
        return false;
    }
    else
        return PlannerParameters::endElement(name);
}

