#include "sensor_data/pose_data.hpp"

namespace location
{
  PoseData::PoseData()
  {
    pose = Eigen::Matrix4f::Identity();
    time = 0.0;
  };
  PoseData::~PoseData() {};

  Eigen::Quaternionf PoseData::GetQuaternion()
  {
    return Eigen::Quaternionf(pose.block<3, 3>(0, 0));
  }
}