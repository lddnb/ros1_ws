#pragma once

#include "Eigen/Dense"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <deque>
#include <string>

namespace location
{
class GraphOptimizerInterface
{
public:
  GraphOptimizerInterface();
  virtual ~GraphOptimizerInterface();
  virtual bool Optimize() = 0;
  virtual bool GetOptimizedPose(std::deque<Eigen::Matrix4f> & optimized_pose) = 0;
  virtual bool GetNodeNum() = 0;
  virtual void SetEdgeRobustKernel(const std::string & robust_kernel_name, const double & robust_kernel_size) = 0;
  virtual void AddSe3Node(const Eigen::Isometry3d & pose, const bool & need_fix) = 0;
  virtual void AddSe3Edge(
    const int & vertex_index1,
    const int & vertex_index2,
    const Eigen::Isometry3d & relative_pose,
    const Eigen::VectorXd & noise) = 0;
  virtual void AddSe3PriorXYZEdge(
    const int & se3_vertex_index,
    const Eigen::Vector3d & xyz,
    const Eigen::VectorXd & noise) = 0;
  virtual void AddSe3PriorQuaternionEdge(
    const int & se3_vertex_index,
    const Eigen::Quaterniond & xyz,
    const Eigen::VectorXd & noise) = 0;

  void SetMaxIterationsNum(const int & max_iterations_num)
  {
    max_iterations_num_ = max_iterations_num;    
  }

protected:
  int max_iterations_num_ = 512;
};
}