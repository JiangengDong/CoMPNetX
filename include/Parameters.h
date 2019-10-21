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
    class SolverParameters {
    public:
        double time = 5.0; // Planning time allowed.
        double range = 0; // Planner `range` value for planners that support this parameter. Automatically determined otherwise (when 0).

        friend std::ostream &operator<<(std::ostream &O, const SolverParameters &v);

        bool deserialize(std::list<std::pair<std::string, std::string>> const &atts);
    };

    class ConstraintParameters {
    public:
        double tolerance = ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE; // Constraint satisfaction tolerance.
        unsigned int max_iter = ompl::magic::CONSTRAINT_PROJECTION_MAX_ITERATIONS; // Maximum number sample tries per sample.
        double delta = ompl::magic::CONSTRAINED_STATE_SPACE_DELTA; // Step-size for discrete geodesic on manifold.
        double lambda = ompl::magic::CONSTRAINED_STATE_SPACE_LAMBDA; // Maximum `wandering` allowed during traversal. Must be greater than 1.

        friend std::ostream &operator<<(std::ostream &O, const ConstraintParameters &v);

        bool deserialize(std::list<std::pair<std::string, std::string>> const &atts);
    };

    class AtlasParameters {
    public:
        double exploration = ompl::magic::ATLAS_STATE_SPACE_EXPLORATION; // Value in [0, 1] which tunes balance of refinement and exploration in atlas sampling.
        double epsilon = ompl::magic::ATLAS_STATE_SPACE_EPSILON; // Maximum distance from an atlas chart to the manifold. Must be positive.
        double rho = ompl::magic::ATLAS_STATE_SPACE_RHO_MULTIPLIER; // Maximum radius for an atlas chart. Must be positive.
        double alpha = ompl::magic::ATLAS_STATE_SPACE_ALPHA; // Maximum angle between an atlas chart and the manifold. Must be in [0, PI/2].
        unsigned int max_charts = ompl::magic::ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION; // Maximum number of atlas charts that can be generated during one manifold traversal.
        bool using_bias = false; // Sets whether the atlas should use frontier-biased chart sampling rather than uniform.
        bool using_tb = false; // Sets whether the constrained configuration space will use tangent bundle.

        bool separate = false; // Sets that the atlas should not compute chart separating halfspaces.

        friend std::ostream &operator<<(std::ostream &O, const AtlasParameters &v);

        bool deserialize(std::list<std::pair<std::string, std::string>> const &atts);
    };
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

        SolverParameters planner_parameters_;
        ConstraintParameters constraint_parameters_;
        AtlasParameters atlas_parameters_;

    private:
        bool _tagOpen = false;
    };
}

#endif //ATLASMPNET_PARAMETERS_H
