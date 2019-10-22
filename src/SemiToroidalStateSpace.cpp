/***********************************************************************

Copyright (c) 2015, Carnegie Mellon University
All rights reserved.

Authors: Chris Dellin <cdellin@gmail.com>

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

#include "SemiToroidalStateSpace.h"

#include <ompl/util/Exception.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/lexical_cast.hpp>

using namespace AtlasMPNet;

SemiToroidalStateSpace::SemiToroidalStateSpace(unsigned int dim) : ompl::base::RealVectorStateSpace(dim), is_wrapping_(dim, false), range_(dim, 0) {
}

void SemiToroidalStateSpace::setIsWrapping(const std::vector<bool> &isWrapping) {
    if (is_wrapping_.size() != dimension_)
        throw ompl::Exception("IsWrapping does not match dimension of state space: expected dimension " +
                              std::to_string(dimension_) + " but got dimension " +
                              std::to_string(is_wrapping_.size()));
    is_wrapping_ = isWrapping;
}

double SemiToroidalStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const {
    double dist = 0.0, diff = 0;
    const auto *s1 = state1->as<StateType>();
    const auto *s2 = state2->as<StateType>();

    for (unsigned int i = 0; i < dimension_; ++i) {
        diff = s1->values[i] - s2->values[i];
        if (is_wrapping_[i]) {
            diff = fmod(diff, range_[i]);
            // The result of fmod will be in [-range_[i], range_[i]], so I have to limit it to [-0.5range_[i], 0.5range[i]].
            // Why can't the fmod function obey the common modulus definition???
            if (diff > 0.5 * range_[i])
                diff -= range_[i];
            else if (diff < -0.5 * range_[i])
                diff += range_[i];
        }
        dist += diff * diff;
    }
    return sqrt(dist);
}

bool SemiToroidalStateSpace::equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const {
    double diff = 0;
    const auto *s1 = state1->as<StateType>();
    const auto *s2 = state2->as<StateType>();

    for (unsigned int i = 0; i < dimension_; ++i) {
        diff = s1->values[i] - s2->values[i];
        if (is_wrapping_[i]) {
            diff = fmod(diff, range_[i]);
            if (diff > 0.5 * range_[i])
                diff -= range_[i];
            else if (diff < -0.5 * range_[i])
                diff += range_[i];
        }
        if (fabs(diff) > std::numeric_limits<double>::epsilon() * 2.0)
            return false;
    }
    return true;
}

void SemiToroidalStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t, ompl::base::State *state) const {
    const auto *rfrom = from->as<StateType>();
    const auto *rto = to->as<StateType>();
    const StateType *rstate = state->as<StateType>();
    double diff;

    for (unsigned int i = 0; i < dimension_; ++i) {
        diff = rto->values[i] - rfrom->values[i];
        if (!is_wrapping_[i]) {
            rstate->values[i] = rfrom->values[i] + diff * t;
        } else {
            if (diff > 0.5 * range_[i])
                diff -= range_[i];
            else if (diff < -0.5 * range_[i])
                diff += range_[i];
            rstate->values[i] = rfrom->values[i] + diff * t;
            if (rstate->values[i] > bounds_.high[i])
                rstate->values[i] -= range_[i];
            else if (rstate->values[i] < bounds_.low[i])
                rstate->values[i] += range_[i];
        }
    }
}

void SemiToroidalStateSpace::enforceBounds(ompl::base::State *state) const {
    auto *statet = state->as<StateType>();
    for (unsigned int i = 0; i < dimension_; ++i) {
        if (is_wrapping_[i]) {
            statet->values[i] = fmod(statet->values[i], range_[i]);
            if (statet->values[i] > bounds_.high[i])
                statet->values[i] -= range_[i];
            else if (statet->values[i] < bounds_.low[i])
                statet->values[i] += range_[i];
        } else {
            if (statet->values[i] > bounds_.high[i])
                statet->values[i] = bounds_.high[i];
            else if (statet->values[i] < bounds_.low[i])
                statet->values[i] = bounds_.low[i];
        }
    }
}

void SemiToroidalStateSpace::setBounds(const ompl::base::RealVectorBounds &bounds) {
    ompl::base::RealVectorStateSpace::setBounds(bounds);
    for (unsigned int i = 0; i < dimension_; ++i) {
        range_[i] = bounds_.high[i] - bounds_.low[i];
    }
}

double SemiToroidalStateSpace::getMaximumExtent() const {
    double e = 0.0;
    for (unsigned int i = 0; i < dimension_; ++i) {
        double d = bounds_.high[i] - bounds_.low[i];
        if (is_wrapping_[i])
            d *= 0.5;
        e += d * d;
    }
    return sqrt(e);
}
