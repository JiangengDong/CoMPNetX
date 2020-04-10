//
// Created by jiangeng on 3/29/20.
//

#ifndef ATLASMPNET_MPNETSAMPLER_H
#define ATLASMPNET_MPNETSAMPLER_H

#include <ompl/base/StateSampler.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <torch/script.h>
#include <openrave/openrave.h>
#include <utility>

#include "TaskSpaceRegionChain.h"

namespace AtlasMPNet {
    class MPNetSampler : public ompl::base::StateSampler {
    public:
        typedef std::shared_ptr<MPNetSampler> Ptr;
        MPNetSampler(const ompl::base::StateSpace *space,
                     OpenRAVE::RobotBasePtr robot,
                     std::vector<TaskSpaceRegionChain::Ptr> tsrchains);

        bool sample(const ompl::base::State *start, const ompl::base::State *goal, ompl::base::State *sample);

        void sampleUniform(ompl::base::State *state) override {
        }

        void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override {
        }

        void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override {
        }


    private:
        torch::jit::script::Module pnet_;
        torch::Tensor ohot_;
        torch::Tensor voxel_;
        torch::Tensor goal_;
        unsigned int dim_;  // dimension of config
        std::vector<double> _scale_factor;
        OpenRAVE::RobotBasePtr robot_;
        unsigned int dof_robot_;
        std::vector<TaskSpaceRegionChain::Ptr> tsrchains_;
        std::vector<unsigned int> dof_tsrchains_;

        torch::Tensor toTensor(const std::vector<double> &src);

        std::vector<double> toVector(const torch::Tensor &tensor);

        static std::vector<double> loadData(const std::string &filename, unsigned int n);
    };
}


#endif //ATLASMPNET_MPNETSAMPLER_H