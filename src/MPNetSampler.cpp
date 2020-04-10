//
// Created by jiangeng on 3/29/20.
//

#include "MPNetSampler.h"


AtlasMPNet::MPNetSampler::MPNetSampler(const ompl::base::StateSpace *space,
                                       OpenRAVE::RobotBasePtr robot,
                                       std::vector<TaskSpaceRegionChain::Ptr> tsrchains) :
        ompl::base::StateSampler(space),
        robot_(std::move(robot)),
        tsrchains_(std::move(tsrchains)) {
    dim_ = 0;
    dof_robot_ = robot_->GetActiveDOF();
    dim_ += dof_robot_;
    dof_tsrchains_.clear();
    for (auto &tsrchain : tsrchains_) {
        dof_tsrchains_.emplace_back(tsrchain->GetNumDOF());
        dim_ += dof_tsrchains_.back();
    }
    _scale_factor.resize(dim_, 1.0);

    std::string pnet_filename="/workspaces/AtlasMPNet/temp/ctpnet_annotated_gpu4.pt";  // TODO: turn into arguments
    pnet_ = torch::jit::load(pnet_filename);
    pnet_.to(at::kCUDA);
    OMPL_INFORM("Load PNet successfully.");

    std::string ohot_filename="/workspaces/AtlasMPNet/temp/seen_reps_txt4/e_0_s_110_coke_can_pp_ohot.csv";  // TODO: turn into arguments
    std::vector<double> ohot_vec = loadData(ohot_filename, 128);
    ohot_ = torch::from_blob(ohot_vec.data(), {1, 128});
    OMPL_INFORM("Load ohot successfully.");

    std::string voxel_filename="/workspaces/AtlasMPNet/temp/seen_reps_txt4/e_0_s_110_coke_can_voxel.csv"; // TODO: turn into arguments
    std::vector<double> voxel_vec = loadData(voxel_filename, 256);
    voxel_ = torch::from_blob(voxel_vec.data(), {1, 256});
    OMPL_INFORM("Load voxel successfully.");

    std::vector<double> goal;
    goal_ = torch::from_blob(goal.data(), {1, dim_});
    OMPL_INFORM("Set goal successfully.");
}

bool AtlasMPNet::MPNetSampler::sample(const ompl::base::State *start, const ompl::base::State *goal, ompl::base::State *sample) {
    std::vector<double> start_config, goal_config, sample_config;
    space_->copyToReals(start_config, start);
    space_->copyToReals(goal_config, goal);

    // sample a robot config with MPNet
    unsigned int offset = 0;
    std::vector<double> robot_start(start_config.begin() + offset, start_config.begin() + offset + dof_robot_),
            robot_goal(goal_config.begin() + offset, goal_config.begin() + offset + dof_robot_);
    auto input = torch::cat({voxel_, ohot_, toTensor(robot_start), toTensor(robot_goal)}, 1).to(at::kCUDA);
    auto robot_sample = toVector(pnet_.forward({input}).toTensor().to(at::kCPU));
    std::copy(robot_sample.begin(), robot_sample.end(), sample_config.begin() + offset);
    offset += dof_robot_;

    // get the corresponding tsrchain config
    std::vector<OpenRAVE::RobotBase::ManipulatorPtr> manips = robot_->GetManipulators();
    robot_->SetActiveDOFValues(robot_sample);
    OpenRAVE::Transform Ttemp;
    for (unsigned int i = 0; i < tsrchains_.size(); i++) {
        auto tsrchain = tsrchains_[i];
        std::vector<double> tsrchain_config(start_config.begin() + offset, start_config.begin() + offset + dof_tsrchains_[i]);
        tsrchain->GetClosestTransform(manips[tsrchain->GetManipInd()]->GetEndEffectorTransform(), tsrchain_config, Ttemp);
        std::copy(tsrchain_config.begin(), tsrchain_config.end(), sample_config.begin() + offset);
        offset += dof_tsrchains_[i];
    }

    space_->copyFromReals(sample, sample_config);
    return true;
}

std::vector<double> AtlasMPNet::MPNetSampler::toVector(const torch::Tensor &tensor) {
    auto data = tensor.accessor<float, 2>()[0];
    std::vector<double> dest(dof_robot_);
    for (unsigned int i = 0; i < dof_robot_; i++) {
        dest[i] = static_cast<double>(data[i]) * _scale_factor[i];
    }
    return dest;
}

torch::Tensor AtlasMPNet::MPNetSampler::toTensor(const std::vector<double> &vec) {
    std::vector<double> scaled_src(dof_robot_);
    for (unsigned int i = 0; i < dof_robot_; i++) {
        scaled_src[i] = vec[i] / _scale_factor[i];
    }
    return torch::from_blob(scaled_src.data(), {1, dof_robot_});
}

std::vector<double> AtlasMPNet::MPNetSampler::loadData(const std::string &filename, unsigned int n) {
    std::ifstream file(filename);
    std::string line;
    std::vector<double> vec(n);
    for (unsigned int i = 0; i < n; i++) {
        if (!getline(file, line))
            break;
        vec.emplace_back(std::stod(line));
    }
    file.close();
    return vec;
}
