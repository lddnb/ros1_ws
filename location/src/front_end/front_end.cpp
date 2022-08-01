#include <vector>
#include "front_end/front_end.hpp"
#include <pcl/common/transforms.h>
#include <glog/logging.h>

namespace location {
proto::FrontEndOptions CreateFrontEndOptions(common::LuaParameterDictionary* const lua_parameter_dictionary) {
    proto::FrontEndOptions options;
    options.set_local_frame_num(lua_parameter_dictionary->GetInt("local_frame_num"));
    options.set_key_frame_distance(lua_parameter_dictionary->GetDouble("key_frame_distance"));
    options.set_frame_filter_resolution(lua_parameter_dictionary->GetDouble("frame_filter_resolution"));
    options.set_loal_map_filter_resolution(lua_parameter_dictionary->GetDouble("loal_map_filter_resolution"));
    options.set_res(lua_parameter_dictionary->GetDouble("res"));
    options.set_step_size(lua_parameter_dictionary->GetDouble("step_size"));
    options.set_trans_eps(lua_parameter_dictionary->GetDouble("trans_eps"));
    options.set_max_iter(lua_parameter_dictionary->GetInt("max_iter"));
    return options;
}

FrontEnd::FrontEnd(const proto::FrontEndOptions & options)
: options_(options)
{
    init_pose_ = Eigen::Matrix4f::Identity();
    step_pose = Eigen::Matrix4f::Identity();
    last_pose = Eigen::Matrix4f::Identity();
    predict_pose = Eigen::Matrix4f::Identity();
    last_key_frame_pose = Eigen::Matrix4f::Identity();

    registration_ptr_ = std::make_shared<NDT>(options_);
    frame_filter_ = std::make_shared<common::Filter>(options_.frame_filter_resolution());
    local_map_filter_ = std::make_shared<common::Filter>(options_.loal_map_filter_resolution());
}

void FrontEnd::UpdateLaserOdom(const CloudData & cloud_data, Eigen::Matrix4f & laser_odom)
{
    Frame current_frame_;
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, index);

    CloudData::CLOUD_PTR filter_cloud_ptr(new CloudData::CLOUD());
    // LOG(INFO) << "num before filter : " << current_frame_.cloud_data.cloud_ptr->points.size();
    frame_filter_->VoxelFilter(current_frame_.cloud_data.cloud_ptr, filter_cloud_ptr);
    // LOG(INFO) << "num after filter : " << filter_cloud_ptr->points.size();

    if (local_map_frame_.size() == 0) {
        current_frame_.pose = init_pose_;
        UpdateLocalMap(current_frame_);
        laser_odom = current_frame_.pose;
        return;
    }

    registration_ptr_->ScanMatch(filter_cloud_ptr, predict_pose, current_frame_.pose);
    laser_odom = current_frame_.pose;

    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    if (Eigen::Affine3f(last_key_frame_pose.inverse() * current_frame_.pose).translation().norm() > 
        options_.key_frame_distance())
    {
        UpdateLocalMap(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }
}

void FrontEnd::SetInitPose(Eigen::Matrix4f & init_pose)
{
    init_pose_ = init_pose;
}

bool FrontEnd::UpdateLocalMap(const Frame & new_key_frame)
{
    Frame key_frame = new_key_frame;
    //? really need?
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));

    CloudData::CLOUD_PTR tranform_cloud_ptr(new CloudData::CLOUD());
    local_map_frame_.emplace_back(new_key_frame);

    if (local_map_frame_.size() > static_cast<size_t>(options_.local_frame_num())) {
        local_map_frame_.pop_front();
    }
    local_map_ptr_.reset(new CloudData::CLOUD());
    for (auto & frame : local_map_frame_) {
        pcl::transformPointCloud(*frame.cloud_data.cloud_ptr,
                                 *tranform_cloud_ptr,
                                 frame.pose);
        *local_map_ptr_ += * tranform_cloud_ptr;
    }

    if (local_map_frame_.size() < static_cast<size_t>(options_.local_frame_num())) {
        registration_ptr_->SetTargetCloud(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_->VoxelFilter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetTargetCloud(filtered_local_map_ptr);
    }
    LOG(INFO) << "Update local map with new key frame";
    return true;
}

}