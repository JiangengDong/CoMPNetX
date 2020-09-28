//
// Created by jiangeng on 3/29/20.
//

#ifndef ATLASMPNET_MPNETXSAMPLER_H
#define ATLASMPNET_MPNETXSAMPLER_H

#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <openrave/openrave.h>
#include <torch/script.h>
#include <utility>

#include "Parameters.h"
#include "RobotHelper.h"
#include "TaskSpaceRegionChain.h"

namespace AtlasMPNet {
    class MPNetXSampler : public ompl::base::StateSampler {
    public:
        typedef std::shared_ptr<MPNetXSampler> Ptr;
        MPNetXSampler(const ompl::base::StateSpace *space,
                     OpenRAVE::RobotBasePtr robot,
                     std::vector<TaskSpaceRegionChain::Ptr> tsrchains,
                     MPNetParameter param);

        bool sample(const ompl::base::State *start, const ompl::base::State *goal, ompl::base::State *sample);

        void sampleUniform(ompl::base::State *state) override {
        }

        void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override {
        }

        void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override {
        }

    private:
        torch::jit::script::Module pnet_;
        torch::jit::script::Module dnet_;
        torch::Tensor ohot_;
        torch::Tensor voxel_;

        unsigned int dim_; // dimension of config
        std::vector<double> _scale_factor, _lower_limits, _upper_limits;
        OpenRAVE::RobotBasePtr robot_;
        unsigned int dof_robot_;
        std::vector<TaskSpaceRegionChain::Ptr> tsrchains_;
        std::vector<RobotHelper> manip_iktools_;
        std::vector<unsigned int> dof_tsrchains_;

        bool use_dnet;
        bool use_voxel;
        bool predict_tsr;
        double dnet_coeff;
        double dnet_threshold;

        torch::Tensor vectorToTensor(const std::vector<double> &src);

        std::vector<double> tensorToVector(const torch::Tensor &tensor);

        static std::vector<float> loadData(const std::string &filename, unsigned int n);

        bool EnforceBound(std::vector<double> &val);

        torch::Tensor stateToTensor(const ompl::base::State *from);
        void tensorToState(const torch::Tensor &from, ompl::base::State *to);
    };
} // namespace AtlasMPNet

#endif //ATLASMPNET_MPNETXSAMPLER_H
