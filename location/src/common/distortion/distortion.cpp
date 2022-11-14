#include "common/distortion/distortion.hpp"

namespace location
{
DistortionAdjust::DistortionAdjust() {}
DistortionAdjust::~DistortionAdjust() {}

void DistortionAdjust::SetMotionInfo(const float & scan_period, const VelocityData & velocity_data)
{
  scan_period_ = scan_period;
  velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
  angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}

bool DistortionAdjust::AdjustCloud(CloudData::CLOUD_PTR & input_cloud_ptr, CloudData::CLOUD_PTR & output_cloud_ptr)
{
  CloudData::CLOUD_PTR origin_cloud_ptr(new CloudData::CLOUD(*input_cloud_ptr));
  output_cloud_ptr.reset();
  float orientation_space = 2.0 * M_PI;
  float delete_space = 5.0 * M_PI / 180.0;
  float start_orientation = atan2(origin_cloud_ptr->points.front().y, origin_cloud_ptr->points.front().x);

  Eigen::AngleAxisf t_v(start_orientation, Eigen::Vector3f::UnitZ());
  Eigen::Matrix3f rotate_matrix = t_v.matrix();
  Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
  transform_matrix.block<3, 3>(0, 0) = rotate_matrix;
  pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);

  velocity_ = rotate_matrix * velocity_;
  angular_rate_ = rotate_matrix * angular_rate_;

  for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) {
    float orientation = atan2(origin_cloud_ptr->points.at(point_index).y, origin_cloud_ptr->points.at(point_index).x);
    if (orientation < 0.0) {
      orientation += 2 * M_PI;
    }
    if (orientation < delete_space || 2 * M_PI - orientation < delete_space) {
      continue;
    }

    float real_time = orientation / orientation_space * scan_period_ - scan_period_ / 2.0;
    Eigen::Vector3f origin_point(
      origin_cloud_ptr->points.at(point_index).x,
      origin_cloud_ptr->points.at(point_index).y,
      origin_cloud_ptr->points.at(point_index).z);
    
    Eigen::Matrix3f current_matirx = UpdateMatrix(real_time);
    Eigen::Vector3f rotated_point = current_matirx * origin_point;
    Eigen::Vector3f adjust_point = rotated_point + velocity_ * real_time;
    CloudData::POINT point(adjust_point[0], adjust_point[1], adjust_point[2]);
    output_cloud_ptr->points.emplace_back(point);
  }

  pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
  return true;
}

Eigen::Matrix3f DistortionAdjust::UpdateMatrix(const float & real_time)
{
  Eigen::Vector3f angle = angular_rate_ * real_time;
  return Eigen::Matrix3f(
    Eigen::AngleAxisf(angle[0], Eigen::Vector3f::UnitX()) *
    Eigen::AngleAxisf(angle[1], Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(angle[2], Eigen::Vector3f::UnitZ()));
}

}