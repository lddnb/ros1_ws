#include "back_end/back_end.hpp"
#include "back_end/g2o_graph_optimizer.hpp"
#include "proto/back_end.pb.h"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Transform.h>
#include <memory>

namespace location
{
proto::BackEndOptions CreateBackEndOptions(common::LuaParameterDictionary* const lua_parameter_dictionary) {
    proto::BackEndOptions options;
    options.set_graph_optimizer_type(lua_parameter_dictionary->GetString("graph_optimizer_type"));
    options.set_use_gnss(lua_parameter_dictionary->GetBool("use_gnss"));
    options.set_use_loop_close(lua_parameter_dictionary->GetBool("use_loop_close"));
    options.set_optimize_step_with_key_frame(lua_parameter_dictionary->GetInt("optimize_step_with_key_frame"));
    options.set_optimize_step_with_gnss(lua_parameter_dictionary->GetInt("optimize_step_with_gnss"));
    options.set_optimize_step_with_loop(lua_parameter_dictionary->GetInt("optimize_step_with_loop"));
    options.set_key_frame_distance(lua_parameter_dictionary->GetDouble("key_frame_distance"));
    options.set_odom_edge_noise(lua_parameter_dictionary->GetString("odom_edge_noise"));
    options.set_loop_edge_noise(lua_parameter_dictionary->GetString("loop_edge_noise"));
    options.set_gnss_edge_noise(lua_parameter_dictionary->GetString("gnss_edge_noise"));
    return options;
}

BackEnd::BackEnd(const proto::BackEndOptions & options) 
: options_(options)
{
  if (options_.graph_optimizer_type() == "g2o") {
    graph_optimizer_ptr_ = std::make_unique<G2oGraphOptimizer>("lm_var_cholmod");
  } else {
    LOG(ERROR) << "Config graph_optimizer_type wrong: " << options_.graph_optimizer_type();
  }
  GetNoiseByString(odom_edge_noise_, options_.odom_edge_noise());
  GetNoiseByString(close_loop_noise_, options_.loop_edge_noise());
  GetNoiseByString(gnss_noise_, options_.gnss_edge_noise());
}

BackEnd::~BackEnd() {}

void BackEnd::GetNoiseByString(Eigen::VectorXd & matrix, const std::string & str)
{
  std::string tmp;
  std::vector<double> vec;
  for (auto c : str) {
    if (c == ' ') {
      vec.emplace_back(stod(tmp));
      tmp.clear();
    } else {
      tmp.push_back(c);
    }
  }
  vec.emplace_back(stod(tmp));
  matrix.resize(vec.size());
  for (size_t i = 0; i < vec.size(); ++i) {
    matrix(i) = vec.at(i);
  }
}

bool BackEnd::Update(const CloudData & cloud_data, const PoseData & laser_odom, const PoseData & gnss_odom)
{
  ResetParams();
  if (MaybeNewKeyFrame(cloud_data, laser_odom, gnss_odom)) {
    AddNodeAndEdge(gnss_odom);
    MaybeOptimized();
  }
  return true;
}
// TODO: add loop
bool BackEnd::InsertLoopPose() 
{
  return true;
}

bool BackEnd::ForceOptimize()
{
  if (graph_optimizer_ptr_->Optimize()) {
    has_new_optimized_ = true;
  }
  return has_new_optimized_;
}

void BackEnd::GetOptimizedKeyFrames(std::deque<KeyFrame> & key_frames_deque)
{
  KeyFrame key_frame;
  graph_optimizer_ptr_->GetOptimizedPose(optimized_pose_);
  for (size_t i = 0; i < optimized_pose_.size(); ++i) {
    key_frame.pose = optimized_pose_.at(i);
    key_frame.index = i;
    key_frames_deque.emplace_back(key_frame);
  }
}

bool BackEnd::HasNewKeyFrame()
{
  return has_new_key_frame_;
}

bool BackEnd::HasNewOptimized()
{
  LOG(INFO) << "has_new_optimized_: " << has_new_optimized_;
  return has_new_optimized_;
}

void BackEnd::GetLatestKeyFrame(KeyFrame & key_frame)
{
  key_frame = current_key_frame_;
}

void BackEnd::GetLatestKeyGnss(KeyFrame & key_frame)
{
  key_frame = current_key_gnss_;
}

void BackEnd::InitParams()
{
  new_key_frame_cnt_ = 0;
  new_gnss_cnt_ = 0;
  new_loop_cnt_ = 0;
}

void BackEnd::ResetParams()
{
  has_new_key_frame_ = false;
  has_new_optimized_ = false;
}

bool BackEnd::AddNodeAndEdge(const PoseData & gnss_data)
{
  Eigen::Isometry3d Isometry;
  Isometry.matrix() = current_key_frame_.pose.cast<double>();
  if (!options_.use_gnss() && graph_optimizer_ptr_->GetNodeNum() == 0) {
    graph_optimizer_ptr_->AddSe3Node(Isometry, true);
  } else {
    graph_optimizer_ptr_->AddSe3Node(Isometry, false);
  }
  new_key_frame_cnt_++;

  static KeyFrame last_key_frame = current_key_frame_;
  int node_num = graph_optimizer_ptr_->GetNodeNum();
  if (node_num > 1) {
    Eigen::Matrix4f relative_pose = last_key_frame.pose.inverse() * current_key_frame_.pose;
    Isometry.matrix() = relative_pose.cast<double>();
    graph_optimizer_ptr_->AddSe3Edge(node_num - 2, node_num - 1, Isometry, odom_edge_noise_);
  }
  last_key_frame = current_key_frame_;

  if (options_.use_gnss()) {
    Eigen::Vector3d xyz(
      static_cast<double>(gnss_data.pose(0, 3)),
      static_cast<double>(gnss_data.pose(1, 3)),
      static_cast<double>(gnss_data.pose(2, 3)));
    graph_optimizer_ptr_->AddSe3PriorXYZEdge(node_num - 1, xyz, gnss_noise_);
    new_gnss_cnt_++;
  }
  return true;
}

bool BackEnd::MaybeNewKeyFrame(const CloudData & cloud_data, const PoseData & laser_odom, const PoseData & gnss_odom)
{
  static Eigen::Matrix4f last_key_pose = laser_odom.pose;
  if (key_frame_deuqe_.size() == 0) {
    has_new_key_frame_ = true;
    last_key_pose = laser_odom.pose;
  }

  if (fabs(laser_odom.pose(0,3) - last_key_pose(0,3)) + 
    fabs(laser_odom.pose(1,3) - last_key_pose(1,3)) +
    fabs(laser_odom.pose(2,3) - last_key_pose(2,3)) > options_.key_frame_distance()) {
      has_new_key_frame_ = true;
      last_key_pose = laser_odom.pose;
  }

  if (has_new_key_frame_) {
    KeyFrame key_frame;
    key_frame.time = laser_odom.time;
    key_frame.index = key_frame_deuqe_.size();
    key_frame.pose = laser_odom.pose;
    key_frame_deuqe_.emplace_back(key_frame);
    current_key_frame_ = key_frame;

    current_key_gnss_.time = gnss_odom.time;
    current_key_gnss_.index = key_frame.index;
    current_key_gnss_.pose = gnss_odom.pose;
  }

  return has_new_key_frame_;
}

bool BackEnd::MaybeOptimized()
{
  LOG(INFO) << "new_gnss_cnt_: " << new_gnss_cnt_;
  LOG(INFO) << "new_key_frame_cnt_: " << new_key_frame_cnt_;
  LOG(INFO) << "new_loop_cnt_: " << new_loop_cnt_;
  if (new_gnss_cnt_ >= options_.optimize_step_with_gnss() ||
    new_key_frame_cnt_ >= options_.optimize_step_with_key_frame() ||
    new_loop_cnt_ >= options_.optimize_step_with_loop()) 
  {
    new_gnss_cnt_ = 0;
    new_key_frame_cnt_ = 0;
    new_loop_cnt_ = 0;
    if (graph_optimizer_ptr_->Optimize()) {
      has_new_optimized_ = true;
    }
    LOG(INFO) << "MaybeOptimized: True";
    return true;
  } else {
    LOG(INFO) << "MaybeOptimized: False";
    return false;
  }
}
} // namespace location