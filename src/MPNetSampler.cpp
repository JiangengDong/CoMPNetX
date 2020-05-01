//
// Created by jiangeng on 3/29/20.
//

#include "MPNetSampler.h"
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>


AtlasMPNet::MPNetSampler::MPNetSampler(const ompl::base::StateSpace *space,
                                       OpenRAVE::RobotBasePtr robot,
                                       std::vector<TaskSpaceRegionChain::Ptr> tsrchains,
                                       MPNetParameter param) :
        ompl::base::StateSampler(space),
        robot_(robot),
        tsrchains_(std::move(tsrchains)) {
    dim_ = 0;
    dof_robot_ = robot->GetActiveDOF();
    dim_ += dof_robot_;

    dof_tsrchains_.clear();
    manips_ = robot->GetManipulators();
    for (auto &tsrchain : tsrchains_) {
        dof_tsrchains_.emplace_back(tsrchain->GetNumDOF());
        dim_ += dof_tsrchains_.back();
        robot_helpers_.emplace_back(RobotHelper(robot, manips_[tsrchain->GetManipInd()]));
    }

    _scale_factor.resize(dof_robot_, 1.0);
    robot->GetActiveDOFLimits(_lower_limits, _upper_limits);
    for(unsigned int i=0; i<dof_robot_; i++) {
        _scale_factor[i] = _upper_limits[i] - _lower_limits[i];
    }

    std::string pnet_filename=param.pnet_path;
    pnet_ = torch::jit::load(pnet_filename);
    pnet_.to(at::kCUDA);
    OMPL_DEBUG("Load %s successfully.", pnet_filename.c_str());

    std::string dnet_filename = param.dnet_path;
    dnet_ = torch::jit::load(dnet_filename);
    dnet_.to(at::kCUDA);
    OMPL_DEBUG("Load %s successfully.", dnet_filename.c_str());

    std::string ohot_filename=param.ohot_path;
    std::vector<float> ohot_vec = loadData(ohot_filename, 128);
    ohot_ = torch::from_blob(ohot_vec.data(), {1, 128}).clone();
    OMPL_DEBUG("Load %s successfully.", ohot_filename.c_str());

    std::string voxel_filename=param.voxel_path;
    std::vector<float> voxel_vec = loadData(voxel_filename, 256);
    voxel_ = torch::from_blob(voxel_vec.data(), {1, 256}).clone();
    OMPL_DEBUG("Load %s successfully.", voxel_filename.c_str());
}

bool AtlasMPNet::MPNetSampler::sample(const ompl::base::State *start, const ompl::base::State *goal, ompl::base::State *sample) {
    std::vector<double> start_config(dim_), goal_config(dim_), sample_config(dim_);
    space_->copyToReals(start_config, start);
    space_->copyToReals(goal_config, goal);

    // sample a robot config with MPNet
    std::vector<double> robot_start(start_config.begin(), start_config.begin() + dof_robot_);
    std::vector<double> robot_goal(goal_config.begin(), goal_config.begin() + dof_robot_);
    auto pnet_input = torch::cat({voxel_, ohot_, toTensor(robot_start), toTensor(robot_goal)}, 1).to(at::kCUDA);
    auto pnet_output = pnet_.forward({pnet_input}).toTensor().to(at::kCPU);

    auto pnet_output_temp = torch::autograd::Variable(pnet_output.clone()).detach().set_requires_grad(true);
    auto dnet_input = torch::cat({voxel_, ohot_, pnet_output_temp}, 1).to(at::kCUDA);
    auto dnet_output = dnet_.forward({dnet_input}).toTensor();
    dnet_output.backward();
    auto grad = pnet_output_temp.grad();
    torch::Tensor dnet_output_temp = dnet_output.to(at::kCPU);
    if (dnet_output_temp.accessor<float, 2>()[0][0]>0.3) {
        pnet_output -= 0.4 * grad;
    }

    auto robot_sample = toVector(pnet_output);
    robot_helpers_[0].EnforceBound(robot_sample);

    robot_->SetActiveDOFValues(robot_sample, 0);
    OpenRAVE::Transform Ttsr, Trobot;
    unsigned int offset = dof_robot_;
    for(unsigned int i=0; i<tsrchains_.size(); i++){
        auto tsrchain = tsrchains_[i];
        std::vector<double> tsrchain_config(start_config.begin() + offset, start_config.begin() + offset + dof_tsrchains_[i]);
        Trobot = robot_helpers_[i].GetEndEffectorTransform();
        tsrchain->GetClosestTransform(Trobot, tsrchain_config, Ttsr);
        robot_helpers_[i].GetClosestTransform(Ttsr, robot_sample, Trobot);
        std::copy(tsrchain_config.begin(), tsrchain_config.end(), sample_config.begin() + offset);
        offset += dof_tsrchains_[i];
    }
    std::copy(robot_sample.begin(), robot_sample.end(), sample_config.begin());

    space_->copyFromReals(sample, sample_config);
    return true;
}

std::vector<double> AtlasMPNet::MPNetSampler::toVector(const torch::Tensor &tensor) {
    auto data = tensor.accessor<float, 2>()[0];
    std::vector<double> dest(dof_robot_);
    for (unsigned int i = 0; i < dof_robot_; i++) {
        dest[i] = static_cast<float>(data[i]) * _scale_factor[i];
    }
    return dest;
}

torch::Tensor AtlasMPNet::MPNetSampler::toTensor(const std::vector<double> &vec) {
    std::vector<float> scaled_src(dof_robot_);
    for (unsigned int i = 0; i < dof_robot_; i++) {
        scaled_src[i] = vec[i] / _scale_factor[i];
    }
    return torch::from_blob(scaled_src.data(), {1, dof_robot_}).clone();
}

std::vector<float> AtlasMPNet::MPNetSampler::loadData(const std::string &filename, unsigned int n) {
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
