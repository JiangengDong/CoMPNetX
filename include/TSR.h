//
// Created by jiangeng on 10/12/19.
//

#ifndef ATLASMPNET_TSR_H
#define ATLASMPNET_TSR_H

#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>

namespace AtlasMPNet {
    class TSR {
    public:
        TSR();

        Eigen::Affine3d T0_w_;
        Eigen::Affine3d T0_w_inv_;
        Eigen::Affine3d Tw_e_;
        Eigen::Affine3d Tw_e_inv_;
        Eigen::Matrix<double, 6, 2> Bw_;
        int manipulater_index_;
        std::string relative_body_name_;
        std::string relative_link_name_;
        bool initialized_;
    };
}


#endif //ATLASMPNET_TSR_H
