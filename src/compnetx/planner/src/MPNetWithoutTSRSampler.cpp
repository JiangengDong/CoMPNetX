//
// Created by jiangeng on 3/29/20.
//

#include "planner/MPNetWithoutTSRSampler.h"
#include <highfive/H5Easy.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

CoMPNetX::MPNetWithoutTSRSampler::MPNetWithoutTSRSampler(const ompl::base::StateSpace *space,
                                                         OpenRAVE::RobotBasePtr robot,
                                                         std::vector<TaskSpaceRegionChain::Ptr> tsrchains,
                                                         MPNetParameter param) : MPNetSampler(space),
                                                                                 robot_(robot),
                                                                                 tsrchains_(std::move(tsrchains)) {

    use_tsr_ = false;

    dof_robot_ = robot->GetActiveDOF();

    dof_tsrchains_.clear();
    auto manips = robot->GetManipulators();
    for (auto &tsrchain : tsrchains_) {
        dof_tsrchains_.emplace_back(tsrchain->GetNumDOF());
        manip_iktools_.emplace_back(RobotHelper(robot, manips[tsrchain->GetManipInd()]));
    }

    space_dim_ = space_->getDimension();
    pnet_dim_ = dof_robot_;

    auto bound = space->as<ompl::base::RealVectorStateSpace>()->getBounds();
    upper_limits_ = bound.high;
    lower_limits_ = bound.low;
    scale_factor_ = std::vector<double>{6.1083, 2.668, 3.4033, 3.194, 6.118, 3.6647, 6.118, 3.6086, 2.62952, 2.0635998, 6.284, 6.284, 6.284};

    std::string pnet_filename = param.pnet_path_;
    pnet_ = torch::jit::load(pnet_filename);
    pnet_.to(at::kCUDA);
    OMPL_DEBUG("Load %s successfully.", pnet_filename.c_str());

    use_dnet_ = param.use_dnet_;
    if (param.use_dnet_) {
        std::string dnet_filename = param.dnet_path_;
        dnet_ = torch::jit::load(dnet_filename);
        dnet_.to(at::kCUDA);
        OMPL_DEBUG("Load %s successfully.", dnet_filename.c_str());
        dnet_coeff_ = param.dnet_coeff_;
        dnet_threshold_ = param.dnet_threshold_;
    }

    std::string ohot_filename = param.ohot_path_;
    std::vector<float> ohot_vec = loadHDF5Dataset(ohot_filename);
    ohot_ = torch::from_blob(ohot_vec.data(), {1, (long)ohot_vec.size()}).clone();
    OMPL_DEBUG("Load %s successfully.", ohot_filename.c_str());

    std::string voxel_filename = param.voxel_path_;
    std::vector<float> voxel_vec = loadHDF5Dataset(voxel_filename);
    voxel_ = torch::from_blob(voxel_vec.data(), {1, (long)voxel_vec.size()}).clone();
    OMPL_DEBUG("Load %s successfully.", voxel_filename.c_str());
}

bool CoMPNetX::MPNetWithoutTSRSampler::sampleMPNet(const ompl::base::State *start, const ompl::base::State *goal, ompl::base::State *sample) {
    // sample a robot config with MPNet
    torch::Tensor pnet_input, pnet_output;
    {
        torch::NoGradGuard guard;
        pnet_input = torch::cat({voxel_, ohot_, stateToTensor(start), stateToTensor(goal)}, 1).to(at::kCUDA);
        pnet_output = pnet_.forward({pnet_input}).toTensor().to(at::kCPU);
    }

    if (use_dnet_) {
        auto pnet_output_temp = torch::autograd::Variable(pnet_output.clone()).detach().set_requires_grad(true);
        auto dnet_input = torch::cat({voxel_, ohot_, pnet_output_temp}, 1).to(at::kCUDA);
        auto dnet_output = dnet_.forward({dnet_input}).toTensor();
        dnet_output.backward();
        auto grad = pnet_output_temp.grad();
        torch::Tensor dnet_output_temp = dnet_output.to(at::kCPU);
        if (dnet_output_temp.accessor<float, 2>()[0][0] > dnet_threshold_) {
            pnet_output -= dnet_coeff_ * grad;
        }
    }

    // handshake
    std::vector<double> sample_config(space_dim_);
    space_->copyToReals(sample_config, start); // copy the tsr value of the start state
    std::vector<double> robot_sample = tensorToVector(pnet_output);
    EnforceBound(robot_sample);
    robot_->SetActiveDOFValues(robot_sample);
    OpenRAVE::Transform Ttsr, Trobot;
    unsigned int offset = dof_robot_;
    for (unsigned int i = 0; i < tsrchains_.size(); i++) {
        auto tsrchain = tsrchains_[i];
        std::vector<double> tsrchain_config(sample_config.begin() + offset, sample_config.begin() + offset + dof_tsrchains_[i]);
        Trobot = manip_iktools_[i].GetEndEffectorTransform();
        tsrchain->GetClosestTransform(Trobot, tsrchain_config, Ttsr);
        manip_iktools_[i].GetClosestTransform(Ttsr, robot_sample, Trobot);
        std::copy(tsrchain_config.begin(), tsrchain_config.end(), sample_config.begin() + offset);
        offset += dof_tsrchains_[i];
    }
    std::copy(robot_sample.begin(), robot_sample.end(), sample_config.begin());
    // handshake finish
    space_->copyFromReals(sample, sample_config);
    return true;
}

std::vector<double> CoMPNetX::MPNetWithoutTSRSampler::tensorToVector(const torch::Tensor &tensor) {
    auto data = tensor.accessor<float, 2>()[0];
    std::vector<double> dest(pnet_dim_);
    for (unsigned int i = 0; i < pnet_dim_; i++) {
        dest[i] = static_cast<float>(data[i]) * scale_factor_[i]; // unnormailize here
    }
    return dest;
}

torch::Tensor CoMPNetX::MPNetWithoutTSRSampler::stateToTensor(const ompl::base::State *from) {
    std::vector<float> scaled_src(pnet_dim_);
    const auto &from_state = *(from->as<ompl::base::ConstrainedStateSpace::StateType>());
    for (unsigned int i = 0; i < pnet_dim_; i++) {
        scaled_src[i] = from_state[i] / scale_factor_[i];
    }
    return torch::from_blob(scaled_src.data(), {1, pnet_dim_}).clone();
}

void CoMPNetX::MPNetWithoutTSRSampler::EnforceBound(std::vector<double> &val) {
    for (unsigned int i = 0; i < dof_robot_; i++) {
        if (val[i] < lower_limits_[i])
            val[i] = lower_limits_[i];
        else if (val[i] > upper_limits_[i])
            val[i] = upper_limits_[i];
    }
}
