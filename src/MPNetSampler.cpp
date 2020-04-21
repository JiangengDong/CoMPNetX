//
// Created by jiangeng on 3/29/20.
//

#include "MPNetSampler.h"


AtlasMPNet::MPNetSampler::MPNetSampler(const ompl::base::StateSpace *space,
                                       OpenRAVE::RobotBasePtr robot,
                                       std::vector<TaskSpaceRegionChain::Ptr> tsrchains, 
                                       MPNetParameter param) :
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

    _scale_factor.resize(dof_robot_, 1.0);
    std::vector<OpenRAVE::dReal> lower_limits, upper_limits;
    robot_->GetActiveDOFLimits(lower_limits, upper_limits);
    for(unsigned int i=0; i<dof_robot_; i++) {
        _scale_factor[i] = upper_limits[i] - lower_limits[i];
    }

    std::string pnet_filename=param.model_path;
    pnet_ = torch::jit::load(pnet_filename);
    pnet_.to(at::kCUDA);
    OMPL_DEBUG("Load %s successfully.", pnet_filename.c_str());

    std::string ohot_filename=param.ohot_path;
    std::vector<float> ohot_vec = loadData(ohot_filename, 128);
    ohot_ = torch::from_blob(ohot_vec.data(), {1, 128}).clone();
    OMPL_DEBUG("Load %s successfully.", ohot_filename.c_str());

    std::string voxel_filename=param.voxel_path;
    std::vector<float> voxel_vec = loadData(voxel_filename, 256);
    voxel_ = torch::from_blob(voxel_vec.data(), {1, 256}).clone();
    OMPL_DEBUG("Load %s successfully.", voxel_filename.c_str());

    std::vector<float> goal;
    goal_ = torch::from_blob(goal.data(), {1, dim_}); // TODO: delete this
}

bool AtlasMPNet::MPNetSampler::sample(const ompl::base::State *start, const ompl::base::State *goal, ompl::base::State *sample) {
    std::vector<double> start_config(dim_), goal_config(dim_), sample_config(dim_);
    space_->copyToReals(start_config, start);
    space_->copyToReals(goal_config, goal);

    // sample a robot config with MPNet
    unsigned int offset = 0;
    std::vector<double> robot_start(start_config.begin() + offset, start_config.begin() + offset + dof_robot_),
            robot_goal(goal_config.begin() + offset, goal_config.begin() + offset + dof_robot_);
    auto input = torch::cat({voxel_, ohot_, toTensor(robot_start), toTensor(robot_goal)}, 1).to(at::kCUDA);
    auto output = pnet_.forward({input}).toTensor().to(at::kCPU);
    auto robot_sample = toVector(output);
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
        vec[i] = std::stod(line);
    }
    file.close();
    return vec;
}
