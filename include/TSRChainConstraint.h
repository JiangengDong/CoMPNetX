//
// Created by jiangeng on 10/11/19.
//

#ifndef ATLASMPNET_TSRCHAINCONSTRAINT_H
#define ATLASMPNET_TSRCHAINCONSTRAINT_H

#include <ompl/base/Constraint.h>

#include "Parameters.h"

namespace AtlasMPNet {
    class TSRChainConstraint : public ompl::base::Constraint {
    public:
        TSRChainConstraint(unsigned int ambientDim, unsigned int coDim);

    };
    typedef std::shared_ptr<TSRChainConstraint> TSRChainConstraintPtr;
}
#endif //ATLASMPNET_TSRCHAINCONSTRAINT_H
