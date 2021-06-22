//
// Created by jiangeng on 3/29/20.
//

#ifndef COMPNETX_MPNETWITHTSRSAMPLER_H
#define COMPNETX_MPNETWITHTSRSAMPLER_H

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <openrave/openrave.h>
#include <torch/script.h>
#include <utility>

#include "Parameters.h"
#include "RobotHelper.h"
#include "TaskSpaceRegionChain.h"
#include "planner/MPNetSampler.h"

namespace CoMPNetX {
class MPNetWithTSRSampler : public MPNetSampler {
public:
    typedef std::shared_ptr<MPNetWithTSRSampler> Ptr;
    MPNetWithTSRSampler(const ompl::base::StateSpace *space,
                        OpenRAVE::RobotBasePtr robot,
                        std::vector<TaskSpaceRegionChain::Ptr> tsrchains,
                        MPNetParameter param);

    bool sampleMPNet(const ompl::base::State *start, const ompl::base::State *goal, ompl::base::State *sample) override;

private:
    // torch component
    torch::jit::script::Module pnet_;
    torch::jit::script::Module dnet_;
    torch::Tensor ohot_;
    torch::Tensor voxel_;
    bool use_dnet_;
    double dnet_coeff_;
    double dnet_threshold_;

    // normalize and boundary
    bool use_tsr_;
    unsigned int dof_robot_;
    std::vector<unsigned int> dof_tsrchains_;
    unsigned int space_dim_;
    unsigned int pnet_dim_; // dimension of config
    std::vector<double> scale_factor_;
    std::vector<double> lower_limits_, upper_limits_;

    // handshake
    OpenRAVE::RobotBasePtr robot_;
    std::vector<TaskSpaceRegionChain::Ptr> tsrchains_;
    std::vector<RobotHelper> manip_iktools_;

    std::vector<double> tensorToVector(const torch::Tensor &tensor);
    torch::Tensor stateToTensor(const ompl::base::State *from);
    void EnforceBound(std::vector<double> &val);
};
} // namespace CoMPNetX

#endif //COMPNETX_MPNETWITHTSRSAMPLER_H
