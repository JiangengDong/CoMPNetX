//
// Created by jiangeng on 10/13/19.
//

#ifndef ATLASMPNET_STATEVALIDITYCHECKER_H
#define ATLASMPNET_STATEVALIDITYCHECKER_H

#include <ompl/base/StateValidityChecker.h>
#include <boost/shared_ptr.hpp>

namespace AtlasMPNet {
    // TODO: implement this with the openrave collision checker
    class StateValidityChecker : public ompl::base::StateValidityChecker {

    };
    typedef std::shared_ptr<StateValidityChecker> StateValidityCheckerPtr;
}


#endif //ATLASMPNET_STATEVALIDITYCHECKER_H
