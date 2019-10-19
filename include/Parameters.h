//
// Created by jiangeng on 10/11/19.
//

#ifndef ATLASMPNET_PARAMETERS_H
#define ATLASMPNET_PARAMETERS_H

#include <openrave/openrave.h>
#include <Eigen/Dense>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>

namespace AtlasMPNet {
    class Parameters : public OpenRAVE::PlannerBase::PlannerParameters {
    public:
        typedef boost::shared_ptr<Parameters> Ptr;
        typedef boost::shared_ptr<Parameters const> ConstPtr;
        typedef boost::weak_ptr<Parameters> WeakPtr;
        typedef boost::weak_ptr<Parameters const> WeakConstPtr;

        Parameters();

        ~Parameters() override;

        bool getStartState(ompl::base::ScopedState<> &start) const;

        bool getGoalState(ompl::base::ScopedState<> &goal) const;

        bool serialize(std::ostream &O, int options) const override;

        ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

        bool endElement(std::string const &name) override;

        /* parameters for general constraint */
        double tolerance = 1e-4; // Constraint satisfaction tolerance.
        unsigned int max_iter = 50; // Maximum number sample tries per sample.
        /* parameters for general constraint configuration space */
        double delta = 0.05; // Step-size for discrete geodesic on manifold.
        double lambda = 2.0; // Maximum `wandering` allowed during traversal. Must be greater than 1.
        /* parameters for planner */
        double time = 5.0; // Planning time allowed.
        double range = 0; // Planner `range` value for planners that support this parameter. Automatically determined otherwise (when 0).
        /* parameters for atlas space and tangent bundle space */
        double exploration = 0.75; // Value in [0, 1] which tunes balance of refinement and exploration in atlas sampling.
        double epsilon = 0.05; // Maximum distance from an atlas chart to the manifold. Must be positive.
        double rho = 5; // Maximum radius for an atlas chart. Must be positive.
        double alpha = ompl::magic::ATLAS_STATE_SPACE_ALPHA; // Maximum angle between an atlas chart and the manifold. Must be in [0, PI/2].
        unsigned int max_charts = 200; // Maximum number of atlas charts that can be generated during one manifold traversal.
        bool using_bias_ = false; // Sets whether the atlas should use frontier-biased chart sampling rather than uniform.
        bool using_tb_ = false;
        /* parameters for atlas space only */
        bool separate = false; // Sets that the atlas should not compute chart separating halfspaces.

    private:
        bool _tagOpen = false;
    };
}

#endif //ATLASMPNET_PARAMETERS_H
