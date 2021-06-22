//
// Created by jiangeng on 3/29/20.
//

#ifndef COMPNETX_MPNETSAMPLER_H
#define COMPNETX_MPNETSAMPLER_H

#include <highfive/H5Easy.hpp>
#include <ompl/base/StateSampler.h>
#include <ompl/base/StateSpace.h>
#include <openrave/openrave.h>
#include <utility>

#include "Parameters.h"
#include "TaskSpaceRegionChain.h"

namespace CoMPNetX {
class MPNetSampler : public ompl::base::StateSampler {
public:
    typedef std::shared_ptr<MPNetSampler> Ptr;
    MPNetSampler(const ompl::base::StateSpace *space) : ompl::base::StateSampler(space) {}

    virtual bool sampleMPNet(const ompl::base::State *start, const ompl::base::State *goal, ompl::base::State *sample) = 0;

    void sampleUniform(ompl::base::State *state) override {}

    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override {}

    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override {}

protected:
    static std::vector<float> loadHDF5Dataset(const std::string &filename) {
        auto colon_pos = filename.find(':');
        std::string name = filename.substr(0, colon_pos);
        std::string dataset_name = filename.substr(colon_pos + 1, filename.size());
        H5Easy::File file(name, H5Easy::File::ReadOnly);
        auto data = H5Easy::load<std::vector<float>>(file, dataset_name);
        return data;
    }
};
} // namespace CoMPNetX

#endif //COMPNETX_MPNETSAMPLER_H
