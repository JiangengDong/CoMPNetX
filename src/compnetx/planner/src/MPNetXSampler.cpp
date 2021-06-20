//
// Created by jiangeng on 3/29/20.
//

#include "planner/MPNetXSampler.h"

#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>

CoMPNetX::MPNetXSampler::MPNetXSampler(const ompl::base::StateSpace *space,
                                       OpenRAVE::RobotBasePtr robot,
                                       std::vector<TaskSpaceRegionChain::Ptr> tsrchains,
                                       MPNetParameter param) : ompl::base::StateSampler(space),
                                                               robot_(robot),
                                                               tsrchains_(std::move(tsrchains)) {
    dnet_coeff = param.dnet_coeff_;
    dnet_threshold = param.dnet_threshold_;

    dim_ = 0;
    dof_robot_ = robot->GetActiveDOF();
    dim_ += dof_robot_;

    dof_tsrchains_.clear();
    auto manips = robot->GetManipulators();
    for (auto &tsrchain : tsrchains_) {
        dof_tsrchains_.emplace_back(tsrchain->GetNumDOF());
        dim_ += dof_tsrchains_.back();
        manip_iktools_.emplace_back(RobotHelper(robot, manips[tsrchain->GetManipInd()]));
    }

    _scale_factor.resize(13, 1.0);
    robot->GetActiveDOFLimits(_lower_limits, _upper_limits);

    _lower_limits.emplace_back(-1.8594740);
    _lower_limits.emplace_back(-1.13057968);
    _lower_limits.emplace_back(-0.5839896);
    _lower_limits.emplace_back(-3.142);
    _lower_limits.emplace_back(-3.142);
    _lower_limits.emplace_back(-3.142);

    _upper_limits.emplace_back(1.7491616);
    _upper_limits.emplace_back(1.4989415);
    _upper_limits.emplace_back(1.47961);
    _upper_limits.emplace_back(3.142);
    _upper_limits.emplace_back(3.142);
    _upper_limits.emplace_back(3.142);

    for (unsigned int i = 0; i < 13; i++) {
        _scale_factor[i] = _upper_limits[i] - _lower_limits[i];
    }

    std::string pnet_filename = param.pnet_path_;
    pnet_ = torch::jit::load(pnet_filename);
    pnet_.to(at::kCUDA);
    OMPL_DEBUG("Load %s successfully.", pnet_filename.c_str());

    if (param.use_dnet_) {
        use_dnet = true;
        std::string dnet_filename = param.dnet_path_;
        dnet_ = torch::jit::load(dnet_filename);
        dnet_.to(at::kCUDA);
        OMPL_DEBUG("Load %s successfully.", dnet_filename.c_str());
    }

    std::string ohot_filename = param.ohot_path;
    std::vector<float> ohot_vec = loadData(ohot_filename, 128);
    ohot_ = torch::from_blob(ohot_vec.data(), {1, 128}).clone();
    OMPL_DEBUG("Load %s successfully.", ohot_filename.c_str());

    if (param.use_voxel) {
        use_voxel = true;
        std::string voxel_filename = param.voxel_path;
        std::vector<float> voxel_vec = loadData(voxel_filename, 256);
        voxel_ = torch::from_blob(voxel_vec.data(), {1, 256}).clone();
        OMPL_DEBUG("Load %s successfully.", voxel_filename.c_str());
    }

    predict_tsr = param.use_tsr_;
}

bool CoMPNetX::MPNetXSampler::sample(const ompl::base::State *start, const ompl::base::State *goal, ompl::base::State *sample) {
    // sample a robot config with MPNet
    auto pnet_input = use_voxel ? (torch::cat({voxel_, ohot_, stateToTensor(start), stateToTensor(goal)}, 1).to(at::kCUDA))
                                : (torch::cat({ohot_, stateToTensor(start), stateToTensor(goal)}, 1).to(at::kCUDA));
    auto pnet_output = pnet_.forward({pnet_input}).toTensor().to(at::kCPU);

    if (use_dnet) {
        auto pnet_output_temp = torch::autograd::Variable(pnet_output.clone()).detach().set_requires_grad(true);
        auto dnet_input = torch::cat({voxel_, ohot_, pnet_output_temp}, 1).to(at::kCUDA); // warn: dnet cannot work with compnetx yet
        auto dnet_output = dnet_.forward({dnet_input}).toTensor();
        dnet_output.backward();
        auto grad = pnet_output_temp.grad();
        torch::Tensor dnet_output_temp = dnet_output.to(at::kCPU);             //d=0.2,s=0.01 and d=0.01 and s=0.1 are good
        if (dnet_output_temp.accessor<float, 2>()[0][0] > 0.1 && dim_ == 13) { // project juice/fuze_bottle/coke_can
            pnet_output -= 0.1 * grad;
        } else if (dnet_output_temp.accessor<float, 2>()[0][0] > 1.2 && dim_ == 11) { // project plasticmug/mugblack/pitcher
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

std::vector<double> CoMPNetX::MPNetXSampler::tensorToVector(const torch::Tensor &tensor) {
    auto data = tensor.accessor<float, 2>()[0];
    std::vector<double> dest(dim_);
    if (dim_ == 13) { // juice/fuze_bottle/coke_can
        for (unsigned int i = 0; i < dim_; i++) {
            dest[i] = static_cast<float>(data[i]) * _scale_factor[i]; // unnormailize here
        }
        return dest;
    } else if (dim_ == 11) { // plasticmug/mugblack/pitcher
        for (unsigned int i = 0; i < 10; i++) {
            dest[i] = static_cast<float>(data[i]) * _scale_factor[i]; // unnormailize here
        }
        dest[10] = static_cast<float>(data[12]) * _scale_factor[12];
        return dest;
    }
}

torch::Tensor CoMPNetX::MPNetXSampler::vectorToTensor(const std::vector<double> &vec) {
    std::vector<float> scaled_src(dim_);
    for (unsigned int i = 0; i < dim_; i++) {
        scaled_src[i] = vec[i] / _scale_factor[i];
    }
    return torch::from_blob(scaled_src.data(), {1, dim_}).clone();
}

torch::Tensor CoMPNetX::MPNetXSampler::stateToTensor(const ompl::base::State *from) {
    std::vector<float> scaled_src(13);
    if (dim_ == 13) { // juice/fuze_bottle/coke_can
        const auto &from_state = *(from->as<ompl::base::ConstrainedStateSpace::StateType>());
        for (unsigned int i = 0; i < dim_; i++) {
            scaled_src[i] = from_state[i] / _scale_factor[i];
        }
        return torch::from_blob(scaled_src.data(), {1, 13}).clone();
    } else if (dim_ == 11) { // plasticmug/mugblack/pitcher
        const auto &from_state = *(from->as<ompl::base::ConstrainedStateSpace::StateType>());
        for (unsigned int i = 0; i < 10; i++) {
            scaled_src[i] = from_state[i] / _scale_factor[i];
        }
        scaled_src[10] = 0.0;
        scaled_src[11] = 0.0;
        scaled_src[12] = from_state[10] / _scale_factor[12];
        return torch::from_blob(scaled_src.data(), {1, 13}).clone();
    }
}

void CoMPNetX::MPNetXSampler::tensorToState(const torch::Tensor &from, ompl::base::State *to) {
    auto data = from.accessor<float, 2>()[0];
    auto &to_state = *(to->as<ompl::base::ConstrainedStateSpace::StateType>());
    float val;
    for (unsigned int i = 0; i < dim_; i++) {
        val = static_cast<float>(data[i]) * _scale_factor[i];
        if (val < _lower_limits[i])
            to_state[i] = _lower_limits[i];
        else if (val > _upper_limits[i])
            to_state[i] = _upper_limits[i];
        else
            to_state[i] = val;
    }
}

std::vector<float> CoMPNetX::MPNetXSampler::loadData(const std::string &filename, unsigned int n) {
    std::ifstream file(filename);
    std::string line;
    std::vector<float> vec(n);
    for (unsigned int i = 0; i < n; i++) {
        if (!getline(file, line))
            break;
        vec[i] = std::stof(line);
    }
    file.close();
    return vec;
}

bool CoMPNetX::MPNetXSampler::EnforceBound(std::vector<double> &val) {
    for (unsigned int i = 0; i < dim_; i++) { // enforcing TSR as well
        if (val[i] < _lower_limits[i])
            val[i] = _lower_limits[i];
        else if (val[i] > _upper_limits[i])
            val[i] = _upper_limits[i];
    }
}
