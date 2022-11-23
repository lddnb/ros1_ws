#pragma once

#include "back_end/graph_optimizer_interface.hpp"
#include "back_end/edge_se3_pirorxyz.hpp"
#include "back_end/edge_se3_pirorquat.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <memory>
#include <string>

G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(pcg);
G2O_USE_OPTIMIZATION_LIBRARY(cholmod);
G2O_USE_OPTIMIZATION_LIBRARY(csparse);

namespace location
{
class G2oGraphOptimizer : public GraphOptimizerInterface
{
public: 
  G2oGraphOptimizer(const std::string & solver_type);
  ~G2oGraphOptimizer();
  bool Optimize();
  bool GetOptimizedPose(std::deque<Eigen::Matrix4f> & optimized_pose);
  bool GetNodeNum();
  void SetEdgeRobustKernel(const std::string & robust_kernel_name, const double & robust_kernel_size);
  void AddSe3Node(const Eigen::Isometry3d & pose, const bool & need_fix);
  void AddSe3Edge(
    const int & vertex_index1,
    const int & vertex_index2,
    const Eigen::Isometry3d & relative_pose,
    const Eigen::VectorXd & noise);
  void AddSe3PriorXYZEdge(
    const int & se3_vertex_index,
    const Eigen::Vector3d & xyz,
    const Eigen::VectorXd & noise);
  void AddSe3PriorQuaternionEdge(
    const int & se3_vertex_index,
    const Eigen::Quaterniond & xyz,
    const Eigen::VectorXd & noise);
private:
  Eigen::MatrixXd CalculateSe3EdgeInfomationMatrix(Eigen::VectorXd noise);
  Eigen::MatrixXd CalculateSe3PriorQuaternionEdgeInfomationMatrix(Eigen::VectorXd noise);
  Eigen::MatrixXd CalculateDiagMatrix(Eigen::VectorXd noise);
  void AddRobustKernel(g2o::OptimizableGraph::Edge * edge, const std::string & kernel_type, const double kernel_size);

private:
  g2o::RobustKernelFactory * robust_kernel_factory_;
  std::unique_ptr<g2o::SparseOptimizer> graph_ptr_;
  std::string robust_kernel_name_;
  double robust_kernel_size_;
  bool need_robust_kernel_ = false;
};
} // namespace location