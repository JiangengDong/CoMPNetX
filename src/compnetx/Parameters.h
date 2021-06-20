//
// Created by jiangeng on 10/11/19.
//

#ifndef COMPNETX_PARAMETERS_H
#define COMPNETX_PARAMETERS_H

#include <Eigen/Dense>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <openrave/openrave.h>
#include <utility>

namespace CoMPNetX {
class SimpleXMLReader : public OpenRAVE::BaseXMLReader {
public:
    explicit SimpleXMLReader(std::string tag_name) : BaseXMLReader(), tag_name_(std::move(tag_name)) {}

    virtual bool serialize(std::ostream &O) const = 0;

    friend std::ostream &operator<<(std::ostream &O, const SimpleXMLReader &v) {
        v.serialize(O);
        return O;
    }

    void characters(const std::string &ch) override {}

    virtual const std::string &getTagName() {
        return tag_name_;
    }

protected:
    bool tag_open_ = false;
    std::string tag_name_;
};

/*! \brief General planner parameters
     *
     * This class is named after "SolverParameter" to avoid the conflict with OpenRAVE::PlannnerBase::PlannerParameters.
     * It contains parameters for planners such as planning time, and provides input and output behaviors in the XML format.
     */
class SolverParameter : public SimpleXMLReader {
public:
    enum SolverType {
        RRT = 0,
        RRTstar,
        RRTConnect,
        CoMPNet,
        CoMPNetX
    } type_ = RRT;
    double time_ = 5.0; // Planning time allowed.
    double range_ = 0;  // Planner `range` value for planners that support this parameter. Automatically determined otherwise (when 0).

    SolverParameter() : SimpleXMLReader("solver_parameters") {}

    ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

    bool endElement(std::string const &name) override;

    bool serialize(std::ostream &O) const override;
};

/*! \brief Parameters for constraint and constrained configuration space
     *
     * Provides input and output behaviors in the XML format.
     */
class ConstraintParameter : public SimpleXMLReader {
public:
    enum SpaceType {
        PROJECTION = 0,
        ATLAS,
        TANGENT_BUNDLE
    } type_ = ATLAS;
    double tolerance_ = ompl::magic::CONSTRAINT_PROJECTION_TOLERANCE;           // Constraint satisfaction tolerance.
    unsigned int max_iter_ = ompl::magic::CONSTRAINT_PROJECTION_MAX_ITERATIONS; // Maximum number sample tries per sample.
    double delta_ = ompl::magic::CONSTRAINED_STATE_SPACE_DELTA;                 // Step-size for discrete geodesic on manifold.
    double lambda_ = ompl::magic::CONSTRAINED_STATE_SPACE_LAMBDA;               // Maximum `wandering` allowed during traversal. Must be greater than 1.

    ConstraintParameter() : SimpleXMLReader("constraint_parameters") {}

    ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

    bool endElement(std::string const &name) override;

    bool serialize(std::ostream &O) const override;
};

/*! \brief Parameters for Atlas configuration space
     *
     * Provides input and output behaviors in the XML format.
     */
class AtlasParameter : public SimpleXMLReader {
public:
    double exploration_ = ompl::magic::ATLAS_STATE_SPACE_EXPLORATION;                   // Value in [0, 1] which tunes balance of refinement and exploration in atlas sampling.
    double epsilon_ = ompl::magic::ATLAS_STATE_SPACE_EPSILON;                           // Maximum distance from an atlas chart to the manifold. Must be positive.
    double rho_ = ompl::magic::ATLAS_STATE_SPACE_RHO_MULTIPLIER;                        // Maximum radius for an atlas chart. Must be positive.
    double alpha_ = ompl::magic::ATLAS_STATE_SPACE_ALPHA;                               // Maximum angle between an atlas chart and the manifold. Must be in [0, PI/2].
    unsigned int max_charts_ = ompl::magic::ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION; // Maximum number of atlas charts that can be generated during one manifold traversal.
    bool using_bias_ = false;                                                           // Sets whether the atlas should use frontier-biased chart sampling rather than uniform.

    bool separate_ = false; // (Only used in atlas state space!) Sets that the atlas should not compute chart separating halfspaces.

    AtlasParameter() : SimpleXMLReader("atlas_parameters") {}

    ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

    bool endElement(std::string const &name) override;

    bool serialize(std::ostream &O) const override;
};

class TSRParameter : public SimpleXMLReader {
public:
    OpenRAVE::Transform T0_w_;   ///< the center of the TSR relative to the link it is attached to (or relative to world frame)
    OpenRAVE::Transform Tw_e_;   ///< the end-effector offset of this TSR
    OpenRAVE::dReal Bw_[6][2]{}; ///< matrix defining maximum and minimum allowable deviation from T0_w in x,y,z,roll,pitch,and yaw

    TSRParameter() : SimpleXMLReader("tsr") {}

    ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

    bool endElement(std::string const &name) override;

    bool serialize(std::ostream &O) const override;

    void reset() {
        T0_w_ = OpenRAVE::Transform();
        Tw_e_ = OpenRAVE::Transform();
        for (auto &i : Bw_)
            for (double &j : i)
                j = 0;
    }
};

class TSRChainParameter : public SimpleXMLReader {
public:
    enum PurposeType {
        CONSTRAINT = 0,
        SAMPLE_START,
        SAMPLE_GOAL
    } purpose_ = CONSTRAINT;
    int manipind_ = -1;                     ///< this specifies the index of the manipulator of the robot that is associated with this TSR
    std::string relativebodyname_ = "NULL"; ///< name of the body T0_w is attached to (NULL = world frame)
    std::string relativelinkname_ = "NULL"; ///< name of the link T0_w is attached to (NULL = world frame)
    std::string mimic_body_name_ = "NULL";
    std::vector<int> mimic_inds_;
    std::vector<TSRParameter> TSRs_;

    TSRChainParameter() : SimpleXMLReader("tsr_chain") {}

    ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

    bool endElement(std::string const &name) override;

    bool serialize(std::ostream &O) const override;

    bool reset();

private:
    TSRParameter temp_tsr_;
};

class MPNetParameter : public SimpleXMLReader {
public:
    std::string pnet_path_;
    std::string dnet_path_;
    std::string voxel_path;
    std::string ohot_path;
    bool use_dnet_ = false;
    bool use_tsr_ = false;
    double dnet_threshold_;
    double dnet_coeff_;

    MPNetParameter() : SimpleXMLReader("mpnet") {}

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

    bool getStartState(double *start) const;

    bool getStartState(ompl::base::ScopedState<> &start) const;

    bool getStartState(std::vector<double> &start) const;

    bool getGoalState(double *goal) const;

    bool getGoalState(ompl::base::ScopedState<> &goal) const;

    bool getGoalState(std::vector<double> &goal) const;

    bool serialize(std::ostream &O, int options) const override;

    ProcessElement startElement(std::string const &name, std::list<std::pair<std::string, std::string>> const &atts) override;

    bool endElement(std::string const &name) override;

    SolverParameter solver_parameter_;
    ConstraintParameter constraint_parameter_;
    AtlasParameter atlas_parameter_;
    std::vector<TSRChainParameter> tsrchains_;
    MPNetParameter mpnet_parameter_;

private:
    TSRChainParameter _tsrchain_temp;
};
} // namespace CoMPNetX

#endif //COMPNETX_PARAMETERS_H
