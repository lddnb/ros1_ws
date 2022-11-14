#pragma once
#include "pcl/common/transforms.h"
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/velocity_data.hpp"

namespace location
{
class DistortionAdjust
{
public:
  DistortionAdjust();
  ~DistortionAdjust();
  void SetMotionInfo(const float & scan_period, const VelocityData & velocity_data);
  bool AdjustCloud(CloudData::CLOUD_PTR & input_cloud_ptr, CloudData::CLOUD_PTR & output_cloud_ptr);

private:
  Eigen::Matrix3f UpdateMatrix(const float & real_time);

private:
  float scan_period_;
  Eigen::Vector3f velocity_;
  Eigen::Vector3f angular_rate_;
};
}  // namespace location