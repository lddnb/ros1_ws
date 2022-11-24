#include "back_end/g2o_graph_optimizer.hpp"
#include "back_end/edge_se3_pirorquat.hpp"
#include "back_end/edge_se3_pirorxyz.hpp"
#include <memory>

namespace location
{
  G2oGraphOptimizer::G2oGraphOptimizer(const std::string & solver_type)
  {
    graph_ptr_ = std::make_unique<g2o::SparseOptimizer>();
    g2o::OptimizationAlgorithmFactory * slover_factory = g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty slover_property;
    g2o::OptimizationAlgorithm * solver = slover_factory->construct(solver_type, slover_property);
    graph_ptr_->setAlgorithm(solver);
    if (!graph_ptr_->solver()) {
      LOG(ERROR) << "G2O optimizer construct failed!";
    }
    robust_kernel_factory_ = g2o::RobustKernelFactory::instance();
  }
  G2oGraphOptimizer::~G2oGraphOptimizer() {}
  bool G2oGraphOptimizer::Optimize()
  {
    static int optimiza_count = 0;
    if (graph_ptr_->edges().size() < 1) {
      return false;
    }
    graph_ptr_->initializeOptimization();
    graph_ptr_->computeInitialGuess();
    graph_ptr_->computeActiveErrors();
    graph_ptr_->setVerbose(false);

    double chi2 = graph_ptr_->chi2();
    int iterations = graph_ptr_->optimize(max_iterations_num_);

    LOG(INFO) << "----- No. " << optimiza_count++ << "Optimization" << std::endl
              << "Vertex num: " << graph_ptr_->vertices().size() << ", Edge num: " << graph_ptr_->edges().size() << std::endl
              << "Iteration num: " << iterations << "/" << max_iterations_num_ << std::endl
              << "diff: " << chi2 << " --> " << graph_ptr_->chi2() << std::endl << std::endl;
    return true;
  }

  bool G2oGraphOptimizer::GetOptimizedPose(std::deque<Eigen::Matrix4f> & optimized_pose)
  {
    optimized_pose.clear();
    int vertex_num = graph_ptr_->vertices().size();
    for (int i = 0; i < vertex_num; ++i) {
      g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(i));
      Eigen::Isometry3d pose = v->estimate();
      optimized_pose.emplace_back(pose.matrix().cast<float>());
    }
    return true;
  }

  bool G2oGraphOptimizer::GetNodeNum()
  {
    return graph_ptr_->vertices().size();
  }

  void G2oGraphOptimizer::SetEdgeRobustKernel(const std::string & robust_kernel_name, const double & robust_kernel_size)
  {
    robust_kernel_name_ = robust_kernel_name;
    robust_kernel_size_ = robust_kernel_size;
    need_robust_kernel_ = true;
  }

  void G2oGraphOptimizer::AddSe3Node(const Eigen::Isometry3d & pose, const bool & need_fix)
  {
    g2o::VertexSE3 * v(new g2o::VertexSE3());
    v->setId(graph_ptr_->vertices().size());
    v->setEstimate(pose);
    if (need_fix) {
      v->setFixed(true);
    }
    graph_ptr_->addVertex(v);
  }
  void G2oGraphOptimizer::AddSe3Edge(
    const int & vertex_index1,
    const int & vertex_index2,
    const Eigen::Isometry3d & relative_pose,
    const Eigen::VectorXd & noise)
  {
    Eigen::MatrixXd information_matrix = CalculateSe3EdgeInfomationMatrix(noise);
    g2o::VertexSE3 * v1 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(vertex_index1));
    g2o::VertexSE3 * v2 = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(vertex_index2));
    g2o::EdgeSE3 * edge(new g2o::EdgeSE3());
    edge->setMeasurement(relative_pose);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v1;
    edge->vertices()[1] = v2;
    graph_ptr_->addEdge(edge);
    if (need_robust_kernel_) {
      AddRobustKernel(edge, robust_kernel_name_, robust_kernel_size_);
    }
  }

  void G2oGraphOptimizer::AddSe3PriorXYZEdge(
    const int & se3_vertex_index,
    const Eigen::Vector3d & xyz,
    const Eigen::VectorXd & noise)
  {
    Eigen::MatrixXd information_matrix = CalculateDiagMatrix(noise);
    g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorXYZ * edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v;
    graph_ptr_->addEdge(edge);
  }

  void G2oGraphOptimizer::AddSe3PriorQuaternionEdge(
    const int & se3_vertex_index,
    const Eigen::Quaterniond & quat,
    const Eigen::VectorXd & noise)
  {
    Eigen::MatrixXd information_matrix = CalculateSe3PriorQuaternionEdgeInfomationMatrix(noise);
    g2o::VertexSE3 * v = dynamic_cast<g2o::VertexSE3 *>(graph_ptr_->vertex(se3_vertex_index));
    g2o::EdgeSE3PriorQuat * edge(new g2o::EdgeSE3PriorQuat());
    edge->setMeasurement(quat);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v;
    graph_ptr_->addEdge(edge);
  }

  Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3EdgeInfomationMatrix(Eigen::VectorXd noise)
  {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
    information_matrix = CalculateDiagMatrix(noise);
    return information_matrix;
  }

  Eigen::MatrixXd G2oGraphOptimizer::CalculateSe3PriorQuaternionEdgeInfomationMatrix(Eigen::VectorXd noise)
  {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
    return information_matrix;
  }

  Eigen::MatrixXd G2oGraphOptimizer::CalculateDiagMatrix(Eigen::VectorXd noise)
  {
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(noise.rows(), noise.rows());
    for (int i = 0; i < noise.rows(); ++i) {
      information_matrix(i, i) /= noise(i);
    }
    return information_matrix;
  }
  void G2oGraphOptimizer::AddRobustKernel(g2o::OptimizableGraph::Edge * edge, const std::string & kernel_type, const double kernel_size)
  {
    if (kernel_type == "NONE") {
      return;
    }

    g2o::RobustKernel * kernel = robust_kernel_factory_->construct(kernel_type);
    if (kernel == nullptr) {
      LOG(WARNING) << "Invaild Robust Kernel Type : " << kernel_type;
    }
    kernel->setDelta(kernel_size);
    edge->setRobustKernel(kernel);
  }

}