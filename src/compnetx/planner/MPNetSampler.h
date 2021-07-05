/***********************************************************************

Copyright (c) 2020, University of California, San Diego
All rights reserved.

Author: Jiangeng Dong <jid103@ucsd.edu>

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

  Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.

  Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*************************************************************************/

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
