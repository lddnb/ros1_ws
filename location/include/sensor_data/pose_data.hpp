#pragma once

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>

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