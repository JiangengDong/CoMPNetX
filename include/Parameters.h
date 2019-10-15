//
// Created by jiangeng on 10/11/19.
//

#ifndef ATLASMPNET_PARAMETERS_H
#define ATLASMPNET_PARAMETERS_H

#include <openrave/plugin.h>
#include <Eigen/Dense>
#include <ompl/base/ScopedState.h>

namespace AtlasMPNet {
    class Parameters : public OpenRAVE::PlannerBase::PlannerParameters {
    public:
        Parameters();
        ~Parameters() override;
        bool getStartState(ompl::base::ScopedState<> &start) const;
        bool getGoalState(ompl::base::ScopedState<> &goal) const;

        // TODO: change default value
        double tolerance = 0.1;
        unsigned int max_iter = 100;
        double delta = 0.1;
        double lambda = 0.1;
        double exploration = 0.1;
        double epsilon = 0.1;
        double rho = 0.1;
        double alpha = 0.1;
        unsigned int max_charts = 10;
        bool separate = false;
    };

    typedef boost::shared_ptr<Parameters> ParametersPtr;
    typedef boost::shared_ptr<Parameters const> ParametersConstPtr;
    typedef boost::weak_ptr<Parameters> ParametersWeakPtr;
    typedef boost::weak_ptr<Parameters const> ParametersWeakConstPtr;
}

#endif //ATLASMPNET_PARAMETERS_H
