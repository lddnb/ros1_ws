#include <vector>
#include "front_end/front_end.hpp"
#include "back_end/back_end.hpp"
#include <pcl/common/transforms.h>
#include <glog/logging.h>
#include <iomanip>

namespace location {
proto::FrontEndOptions CreateFrontEndOptions(common::LuaParameterDictionary* const lua_parameter_dictionary) {
    proto::FrontEndOptions options;
    options.set_local_frame_num(lua_parameter_dictionary->GetInt("local_frame_num"));
    options.set_key_frame_distance(lua_parameter_dictionary->GetDouble("key_frame_distance"));
    options.set_frame_filter_resolution(lua_parameter_dictionary->GetDouble("frame_filter_resolution"));
    options.set_loal_map_filter_resolution(lua_parameter_dictionary->GetDouble("loal_map_filter_resolution"));
    options.set_display_filter_resolution(lua_parameter_dictionary->GetDouble("display_filter_resolution"));
    options.set_res(lua_parameter_dictionary->GetDouble("res"));
    options.set_step_size(lua_parameter_dictionary->GetDouble("step_size"));
    options.set_trans_eps(lua_parameter_dictionary->GetDouble("trans_eps"));
    options.set_max_iter(lua_parameter_dictionary->GetInt("max_iter"));
    *options.mutable_back_end_options() = CreateBackEndOptions(lua_parameter_dictionary->GetDictionary("back_end_options").get());
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

    registration_ptr_ = std::make_unique<NDT>(options_);
    frame_filter_ = std::make_unique<common::VoxelFilter>(options_.frame_filter_resolution());
    local_map_filter_ = std::make_unique<common::VoxelFilter>(options_.loal_map_filter_resolution());
    display_filter_ptr_ = std::make_unique<common::VoxelFilter>(options_.display_filter_resolution());
    current_scan_ptr_.reset(new CloudData::CLOUD());
    has_new_local_map_ = false;
}

void FrontEnd::UpdateLaserOdom(const CloudData & cloud_data, Eigen::Matrix4f & laser_odom)
{
    Frame current_frame_;
    current_frame_.cloud_data.time = cloud_data.time;
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, index);

    CloudData::CLOUD_PTR filter_cloud_ptr(new CloudData::CLOUD());
    // LOG(INFO) << "num before filter : " << current_frame_.cloud_data.cloud_ptr->points.size();
    frame_filter_->Filter(current_frame_.cloud_data.cloud_ptr, filter_cloud_ptr);
    // LOG(INFO) << "num after filter : " << filter_cloud_ptr->points.size();

    if (local_map_frame_.size() == 0) {
        current_frame_.pose = init_pose_;
        UpdateLocalMap(current_frame_);
        laser_odom = current_frame_.pose;
        last_pose = init_pose_;
        predict_pose = init_pose_;
        last_key_frame_pose = init_pose_;
        return;
    }
    // static int match_num = 0;
    // LOG(INFO) << "~~~~~ No." << match_num++ << " ~~~~~";
    // LOG(INFO) << "local map frame size: " << local_map_frame_.size();
    // LOG(INFO) << "Point Cloud TimeStamp: " << std::fixed << std::setprecision(3) << cloud_data.time;
    // float x, y, z, roll, pitch, yaw;
    // pcl::getTranslationAndEulerAngles (Eigen::Affine3f(predict_pose), x, y, z, roll, pitch, yaw);
    // LOG(INFO) << "predict_pose: " << std::endl << "Translation: " << x << ", " << y << ", " << z << std::endl
    //           << "Rotation:    " << roll << ", " << pitch << ", " << yaw;

    registration_ptr_->ScanMatch(filter_cloud_ptr, predict_pose, current_scan_ptr_, current_frame_.pose);
    laser_odom = current_frame_.pose;

    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // if (Eigen::Affine3f(last_key_frame_pose.inverse() * current_frame_.pose).translation().norm() > 
    //     options_.key_frame_distance())
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > options_.key_frame_distance())
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
    //! TODO: really need?
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
    has_new_local_map_ = true;

    if (local_map_frame_.size() < 10) {
        registration_ptr_->SetTargetCloud(local_map_ptr_);
    } else {
        CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
        local_map_filter_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetTargetCloud(filtered_local_map_ptr);
    }
    key_frame.cloud_data.cloud_ptr.reset();
    LOG(INFO) << "Update local map with new key frame";
    return true;
}

bool FrontEnd::GetCurrentScan(CloudData::CLOUD_PTR & current_scan_ptr)
{
    display_filter_ptr_->Filter(current_scan_ptr_, current_scan_ptr);
    return true;
}
bool FrontEnd::GetNewLocalMap(CloudData::CLOUD_PTR & local_map_ptr)
{
    if (has_new_local_map_) {
        display_filter_ptr_->Filter(local_map_ptr_, local_map_ptr);
        return true;
    }
    return false;
    
}

}  // namespace location