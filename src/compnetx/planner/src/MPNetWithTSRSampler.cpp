//
// Created by jiangeng on 3/29/20.
//

#include "planner/MPNetWithTSRSampler.h"
#include <ATen/core/grad_mode.h>
#include <highfive/H5Easy.hpp>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/util/Console.h>

CoMPNetX::MPNetWithTSRSampler::MPNetWithTSRSampler(const ompl::base::StateSpace *space,
                                                   OpenRAVE::RobotBasePtr robot,
                                                   std::vector<TaskSpaceRegionChain::Ptr> tsrchains,
                                                   MPNetParameter param) : MPNetSampler(space),
                                                                           robot_(robot),
                                                                           tsrchains_(std::move(tsrchains)) {
    use_tsr_ = true;

    dof_robot_ = robot->GetActiveDOF();
    dof_tsrchains_.clear();
    auto manips = robot->GetManipulators();
    for (auto &tsrchain : tsrchains_) {
        dof_tsrchains_.emplace_back(tsrchain->GetNumDOF());
        manip_iktools_.emplace_back(RobotHelper(robot, manips[tsrchain->GetManipInd()]));
    }

    space_dim_ = space_->getDimension();
    pnet_dim_ = 13;

    robot->GetActiveDOFLimits(lower_limits_, upper_limits_);

    lower_limits_.emplace_back(-1.8594740);
    lower_limits_.emplace_back(-1.13057968);
    lower_limits_.emplace_back(-0.5839896);
    lower_limits_.emplace_back(-3.142);
    lower_limits_.emplace_back(-3.142);
    lower_limits_.emplace_back(-3.142);

    upper_limits_.emplace_back(1.7491616);
    upper_limits_.emplace_back(1.4989415);
    upper_limits_.emplace_back(1.47961);
    upper_limits_.emplace_back(3.142);
    upper_limits_.emplace_back(3.142);
    upper_limits_.emplace_back(3.142);

    scale_factor_.resize(13, 1.0);
    for (unsigned int i = 0; i < 13; i++) {
        scale_factor_[i] = upper_limits_[i] - lower_limits_[i];
    }

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

bool CoMPNetX::MPNetWithTSRSampler::sampleMPNet(const ompl::base::State *start, const ompl::base::State *goal, ompl::base::State *sample) {
    // sample a robot config with MPNet
    torch::Tensor pnet_input, pnet_output;
    {
        torch::NoGradGuard guard;
        pnet_input = torch::cat({voxel_, ohot_, stateToTensor(start), stateToTensor(goal)}, 1).to(at::kCUDA);
        pnet_output = pnet_.forward({pnet_input}).toTensor().to(at::kCPU);
    }

    if (use_dnet_) {
        auto pnet_output_temp = torch::autograd::Variable(pnet_output.clone()).detach().set_requires_grad(true);
        auto dnet_input = torch::cat({voxel_, ohot_, pnet_output_temp}, 1).to(at::kCUDA); // warn: dnet cannot work with compnetx yet
        auto dnet_output = dnet_.forward({dnet_input}).toTensor();
        dnet_output.backward();
        auto grad = pnet_output_temp.grad();
        torch::Tensor dnet_output_temp = dnet_output.to(at::kCPU);                   //d=0.2,s=0.01 and d=0.01 and s=0.1 are good
        if (dnet_output_temp.accessor<float, 2>()[0][0] > 0.1 && space_dim_ == 13) { // project juice/fuze_bottle/coke_can
            pnet_output -= 0.1 * grad;
        } else if (dnet_output_temp.accessor<float, 2>()[0][0] > 1.2 && space_dim_ == 11) { // project plasticmug/mugblack/pitcher
            pnet_output -= 0.01 * grad;
        }
    }

    auto sample_config = tensorToVector(pnet_output);
    EnforceBound(sample_config);
    // handshake
    std::vector<double> robot_sample(sample_config.begin(), sample_config.begin() + dof_robot_);
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

std::vector<double> CoMPNetX::MPNetWithTSRSampler::tensorToVector(const torch::Tensor &tensor) {
    auto data = tensor.accessor<float, 2>()[0];
    std::vector<double> dest(space_dim_);
    if (space_dim_ == 13) { // juice/fuze_bottle/coke_can
        for (unsigned int i = 0; i < space_dim_; i++) {
            dest[i] = static_cast<float>(data[i]) * scale_factor_[i]; // unnormailize here
        }
    } else if (space_dim_ == 11) { // plasticmug/teakettle/mugred/mugblack/pitcher
        for (unsigned int i = 0; i < 10; i++) {
            dest[i] = static_cast<float>(data[i]) * scale_factor_[i]; // unnormailize here
        }
        dest[10] = static_cast<float>(data[12]) * scale_factor_[12];
    } else { // TODO: add door
        OMPL_ERROR("Invalid ambient space dimension!");
    }
    return dest;
}

torch::Tensor CoMPNetX::MPNetWithTSRSampler::stateToTensor(const ompl::base::State *from) {
    std::vector<float> scaled_src(13);
    if (space_dim_ == 13) { // juice/fuze_bottle/coke_can
        const auto &from_state = *(from->as<ompl::base::ConstrainedStateSpace::StateType>());
        for (unsigned int i = 0; i < space_dim_; i++) {
            scaled_src[i] = from_state[i] / scale_factor_[i];
        }
        return torch::from_blob(scaled_src.data(), {1, 13}).clone();
    } else if (space_dim_ == 11) { // plasticmug/mugblack/pitcher
        const auto &from_state = *(from->as<ompl::base::ConstrainedStateSpace::StateType>());
        for (unsigned int i = 0; i < 10; i++) {
            scaled_src[i] = from_state[i] / scale_factor_[i];
        }
        scaled_src[10] = 0.0;
        scaled_src[11] = 0.0;
        scaled_src[12] = from_state[10] / scale_factor_[12];
        return torch::from_blob(scaled_src.data(), {1, 13}).clone();
    } else {
        OMPL_ERROR("Invalid ambient space dimension!");
        return torch::Tensor{};
    }
}

void CoMPNetX::MPNetWithTSRSampler::EnforceBound(std::vector<double> &val) {
    for (unsigned int i = 0; i < space_dim_; i++) { // enforcing TSR as well
        if (val[i] < lower_limits_[i])
            val[i] = lower_limits_[i];
        else if (val[i] > upper_limits_[i])
            val[i] = upper_limits_[i];
    }
}
