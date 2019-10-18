//
// Created by jiangeng on 10/12/19.
//
#include "Parameters.h"

#include <openrave/plugin.h>
#include <Eigen/Dense>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>

using namespace AtlasMPNet;

Parameters::Parameters() : OpenRAVE::PlannerBase::PlannerParameters() {
    _vXMLParameters.emplace_back("using_bias");
    _vXMLParameters.emplace_back("using_tb");
    _vXMLParameters.emplace_back("tolerance");
    _vXMLParameters.emplace_back("max_iter");
    _vXMLParameters.emplace_back("delta");
    _vXMLParameters.emplace_back("lambda");
    _vXMLParameters.emplace_back("exploration");
    _vXMLParameters.emplace_back("epsilon");
    _vXMLParameters.emplace_back("rho");
    _vXMLParameters.emplace_back("alpha");
    _vXMLParameters.emplace_back("max_charts");
    _vXMLParameters.emplace_back("separate");
}

Parameters::~Parameters() = default;

bool Parameters::getStartState(ompl::base::ScopedState<> &start) const {
    const int dof = GetDOF();// TODO: is this the right dof?
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

bool Parameters::serialize(std::ostream &O, int options) const {
    if (!OpenRAVE::PlannerBase::PlannerParameters::serialize(O, options)) {
        return false;
    }
    O << "<tolerance>" << tolerance << "</tolerance>"
      << "<max_iter>" << max_iter << "</max_iter>"
      << "<delta>" << delta << "</delta>"
      << "<lambda>" << lambda << "</lambda>"
      << "<time>" << time << "</time>"
      << "<range>" << range << "</range>"
      << "<exploration>" << exploration << "</exploration>"
      << "<epsilon>" << epsilon << "</epsilon>"
      << "<rho>" << rho << "</rho>"
      << "<alpha>" << alpha << "</alpha>"
      << "<max_chart>" << max_charts << "</max_chart>"
      << "<using_bias>" << using_bias_ << "</using_bias>"
      << "<using_tb>" << using_tb_ << "</using_tb>"
      << "<separate>" << separate << "</separate>";
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
    _tagOpen =
            name == "tolerance" ||
            name == "max_iter" ||
            name == "delta" ||
            name == "lambda" ||
            name == "time" ||
            name == "range" ||
            name == "exploration" ||
            name == "epsilon" ||
            name == "rho" ||
            name == "alpha" ||
            name == "max_charts" ||
            name == "using_bias" ||
            name == "using_tb" ||
            name == "separate";
    return _tagOpen ? PE_Support : PE_Pass;
}

bool Parameters::endElement(std::string const &name) {
    if (_tagOpen) {
        if(name == "tolerance")
            _ss >> tolerance;
        else if (name == "max_iter")
            _ss >> max_iter;
        else if (name == "delta")
            _ss >> delta;
        else if (name == "lambda")
            _ss >> lambda;
        else if (name == "time")
            _ss >> time;
        else if (name == "range")
            _ss >> range;
        else if (name == "exploration")
            _ss >> exploration;
        else if (name == "epsilon")
            _ss >> epsilon;
        else if (name == "rho")
            _ss >> rho;
        else if (name == "alpha")
            _ss >> alpha;
        else if (name == "max_charts")
            _ss >> max_charts;
        else if (name == "using_bias")
            _ss >> using_bias_;
        else if (name == "using_tb")
            _ss >> using_tb_;
        else if (name == "separate")
            _ss >> separate;
        _tagOpen = false;
        return false;
    }
    else
        return PlannerParameters::endElement(name);
}
