//
// Created by jiangeng on 10/11/19.
//

#ifndef ATLASMPNET_PARAMETERS_H
#define ATLASMPNET_PARAMETERS_H

#include <openrave/openrave.h>
#include <Eigen/Dense>
#include <utility>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>

#include "TSR.h"

namespace AtlasMPNet {
    class SimpleXMLReader : public OpenRAVE::BaseXMLReader {
    public:
        explicit SimpleXMLReader(std::string tag_name) : BaseXMLReader(), _tag_name(std::move(tag_name)) {}

        virtual bool serialize(std::ostream &O) const = 0;

        friend std::ostream &operator<<(std::ostream &O, const SimpleXMLReader &v) {
            v.serialize(O);
            return O;
        }

        void characters(const std::string &ch) override {}

    protected:
        bool _tag_open = false;
        const std::string _tag_name;
    };

    class TSRChainParameters : public SimpleXMLReader {
    public:
        std::vector<AtlasMPNet::TSR::Ptr> TSRs_;

        TSRChainParameters() : SimpleXMLReader("tsr_chain") {}

        ProcessElement startElement(const std::string &name, const OpenRAVE::AttributesList &atts) override;

        bool endElement(const std::string &name) override;

        bool serialize(std::ostream &O) const override;

    private:
        AtlasMPNet::TSR::Ptr _active_tsr;
    };

    /*! \brief General planner parameters
     *
     * This class is named after "SolverParameters" to avoid the conflict with OpenRAVE::PlannnerBase::PlannerParameters.
     * It contains parameters for planners such as planning time, and provides input and output behaviors in the XML format.
     */
    class SolverParameters : public SimpleXMLReader {
    public:
        double time_ = 5.0; // Planning time allowed.
        double range_ = 0; // Planner `range` value for planners that support this parameter. Automatically determined otherwise (when 0).

        SolverParameters() : SimpleXMLReader("planner_parameters") {}

        ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

        bool endElement(std::string const &name) override;

        bool serialize(std::ostream &O) const override;
    };

    /*! \brief Parameters for constraint and constrained configuration space
     *
     * Provides input and output behaviors in the XML format.
     */
    class ConstraintParameters : public SimpleXMLReader {
    public:
        double tolerance_ = ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE; // Constraint satisfaction tolerance.
        unsigned int max_iter_ = ompl::magic::CONSTRAINT_PROJECTION_MAX_ITERATIONS; // Maximum number sample tries per sample.
        double delta_ = ompl::magic::CONSTRAINED_STATE_SPACE_DELTA; // Step-size for discrete geodesic on manifold.
        double lambda_ = ompl::magic::CONSTRAINED_STATE_SPACE_LAMBDA; // Maximum `wandering` allowed during traversal. Must be greater than 1.

        ConstraintParameters() : SimpleXMLReader("constraint_parameters") {}

        ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

        bool endElement(std::string const &name) override;

        bool serialize(std::ostream &O) const override;
    };

    /*! \brief Parameters for Atlas configuration space
     *
     * Provides input and output behaviors in the XML format.
     */
    class AtlasParameters : public SimpleXMLReader {
    public:
        double exploration_ = ompl::magic::ATLAS_STATE_SPACE_EXPLORATION; // Value in [0, 1] which tunes balance of refinement and exploration in atlas sampling.
        double epsilon_ = ompl::magic::ATLAS_STATE_SPACE_EPSILON; // Maximum distance from an atlas chart to the manifold. Must be positive.
        double rho_ = ompl::magic::ATLAS_STATE_SPACE_RHO_MULTIPLIER; // Maximum radius for an atlas chart. Must be positive.
        double alpha_ = ompl::magic::ATLAS_STATE_SPACE_ALPHA; // Maximum angle between an atlas chart and the manifold. Must be in [0, PI/2].
        unsigned int max_charts_ = ompl::magic::ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION; // Maximum number of atlas charts that can be generated during one manifold traversal.
        bool using_bias_ = false; // Sets whether the atlas should use frontier-biased chart sampling rather than uniform.
        bool using_tb_ = false; // Sets whether the constrained configuration space will use tangent bundle.

        bool separate_ = false; // Sets that the atlas should not compute chart separating halfspaces.

        AtlasParameters() : SimpleXMLReader("atlas_parameters") {}

        ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

        bool endElement(std::string const &name) override;

        bool serialize(std::ostream &O) const override;
    };

    /*! \brief The parameters consisting of SolverParameters, ConstraintParameters and AtlasParameters.
     *
     * This is a complete class that is able to read from and write to a XML format string.
     * It will detect three tags, which are "planner_parameters", "constraint_parameters" and "atlas_parameters", and will relay the attribute list to corresponding class.
     */
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
        TSRChainParameters tsrchain_parameters_;
    };
}

#endif //ATLASMPNET_PARAMETERS_H
