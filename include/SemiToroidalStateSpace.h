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

#ifndef ATLASMPNET_SEMITOROIDALSTATESPACE_H
#define ATLASMPNET_SEMITOROIDALSTATESPACE_H

#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace AtlasMPNet {

    /*! \brief Semi-torus OMPL state space for wrapping dimensions.
     *
     * This is a generalization of ompl::base::RealVectorStateSpace
     * which supports wrapping on each dimension individually
     * (that is, the dimension's lower bound and upper bound correspond).
     * This is especially useful for handling robots with circular joints.
     * It should be funcionally equivalent to building a coupound state
     * space with many SO(2) components, except it should be faster because
     * all states are stored contiguously.
     */
    class SemiToroidalStateSpace : public ompl::base::RealVectorStateSpace {
    public:
        typedef std::shared_ptr<SemiToroidalStateSpace> Ptr;

        explicit SemiToroidalStateSpace(unsigned int dim = 0);

        virtual void setIsWrapping(const std::vector<bool> &isWrapping);

        virtual const std::vector<bool> &getIsWrapping() const { return is_wrapping_; }

        double getMaximumExtent() const override;

        double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;

        bool equalStates(const ompl::base::State *state1, const ompl::base::State *state2) const override;

        void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;

        void enforceBounds(ompl::base::State *state) const override;

        // WARNING: if you want to use this class, you must always call the setBounds method with a SemiToroidalStateSpace object or pointer,
        // because I cannot override this function but only hide the version of the base class.
        void setBounds(const ompl::base::RealVectorBounds &bounds);

    private:
        std::vector<bool> is_wrapping_;
        std::vector<double> range_;
    };

} // namespace AtlasMPNet

#endif // ATLASMPNET_SEMITOROIDALSTATESPACE_H
