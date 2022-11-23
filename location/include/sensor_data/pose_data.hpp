#pragma once

#include <Eigen/Dense>

namespace location
{
class PoseData
{
public:
  PoseData();
  ~PoseData();
  Eigen::Quaternionf GetQuaternion();
public:
  Eigen::Matrix4f pose;
  double time;
};

}