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

        virtual const std::string &getTagName() {
            return _tag_name;
        }

    protected:
        bool _tag_open = false;
        std::string _tag_name;
    };

    /*! \brief General planner parameters
     *
     * This class is named after "SolverParameters" to avoid the conflict with OpenRAVE::PlannnerBase::PlannerParameters.
     * It contains parameters for planners such as planning time, and provides input and output behaviors in the XML format.
     */
    class SolverParameters : public SimpleXMLReader {
    public:
        enum SolverType {
            RRT = 0, RRTStar, RRTConnect, MPNet
        } type_ = RRT;
        double time_ = 5.0; // Planning time allowed.
        double range_ = 0; // Planner `range` value for planners that support this parameter. Automatically determined otherwise (when 0).

        SolverParameters() : SimpleXMLReader("solver_parameters") {}

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
        enum SpaceType {
            PROJECTION = 0, ATLAS, TANGENT_BUNDLE
        } type_ = ATLAS;
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

        bool separate_ = false; // (Only used in atlas state space!) Sets that the atlas should not compute chart separating halfspaces.

        AtlasParameters() : SimpleXMLReader("atlas_parameters") {}

        ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

        bool endElement(std::string const &name) override;

        bool serialize(std::ostream &O) const override;
    };

    class TSRParameters : public SimpleXMLReader {
    public:
        OpenRAVE::Transform T0_w; ///< the center of the TSR relative to the link it is attached to (or relative to world frame)
        OpenRAVE::Transform Tw_e; ///< the end-effector offset of this TSR
        OpenRAVE::dReal Bw[6][2]{}; ///< matrix defining maximum and minimum allowable deviation from T0_w in x,y,z,roll,pitch,and yaw

        TSRParameters() : SimpleXMLReader("tsr") {}

        ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

        bool endElement(std::string const &name) override;

        bool serialize(std::ostream &O) const override;

        void reset() {
            T0_w = OpenRAVE::Transform();
            Tw_e = OpenRAVE::Transform();
            for (auto &i : Bw)
                for (double &j : i)
                    j = 0;
        }
    };

    class TSRChainParameters : public SimpleXMLReader {
    public:
        enum PurposeType {
            CONSTRAINT = 0, SAMPLE_START, SAMPLE_GOAL
        } purpose = CONSTRAINT;
        int manipind = -1; ///< this specifies the index of the manipulator of the robot that is associated with this TSR
        std::string relativebodyname="NULL"; ///< name of the body T0_w is attached to (NULL = world frame)
        std::string relativelinkname="NULL"; ///< name of the link T0_w is attached to (NULL = world frame)
        std::string mimic_body_name = "NULL";
        std::vector<int> mimic_inds;
        std::vector<TSRParameters> TSRs;

        TSRChainParameters() : SimpleXMLReader("tsr_chain") {}

        ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

        bool endElement(std::string const &name) override;

        bool serialize(std::ostream &O) const override;

    private:
        TSRParameters temp_tsr;
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

        bool getStartState(double *start) const;

        bool getStartState(ompl::base::ScopedState<> &start) const;

        bool getStartState(std::vector<double> &start) const;

        bool getGoalState(double *goal) const;

        bool getGoalState(ompl::base::ScopedState<> &goal) const;

        bool getGoalState(std::vector<double> &goal) const;

        bool serialize(std::ostream &O, int options) const override;

        ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

        bool endElement(std::string const &name) override;

        SolverParameters solver_parameters_;
        ConstraintParameters constraint_parameters_;
        AtlasParameters atlas_parameters_;
        TSRChainParameters tsrchain_parameters_;
    };
}

#endif //ATLASMPNET_PARAMETERS_H
