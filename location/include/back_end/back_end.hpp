#pragma once
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/pose_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "back_end/graph_optimizer_interface.hpp"
#include "common/configuration_file_resolver.hpp"
#include "proto/back_end.pb.h"
#include <deque>
#include <memory>

namespace location
{
struct KeyFrame
{
  double time = 0.0;
  unsigned int index = 0;
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
};
proto::BackEndOptions CreateBackEndOptions(common::LuaParameterDictionary* const lua_parameter_dictionary);
class BackEnd
{
public:
  BackEnd(const proto::BackEndOptions & options);
  ~BackEnd();
  bool Update(const CloudData & cloud_data, const PoseData & laser_odom, const PoseData & gnss_odom);
  bool InsertLoopPose();
  bool ForceOptimize();

  void GetOptimizedKeyFrames(std::deque<KeyFrame> & key_frames_deque);
  bool HasNewKeyFrame();
  bool HasNewOptimized();
  void GetLatestKeyFrame(KeyFrame & key_frame);
  void GetLatestKeyGnss(KeyFrame & key_frame);

private:
  void InitParams();
  void ResetParams();
  bool AddNodeAndEdge(const PoseData & gnss_data);
  bool MaybeNewKeyFrame(const CloudData & cloud_data, const PoseData & laser_odom, const PoseData & gnss_odom);
  bool MaybeOptimized();

  void GetNoiseByString(Eigen::VectorXd & matrix, const std::string & str);

private:
  proto::BackEndOptions options_;
  float key_frame_distance_;
  bool has_new_key_frame_;
  bool has_new_optimized_;

  std::deque<KeyFrame> key_frame_deuqe_;
  std::deque<Eigen::Matrix4f> optimized_pose_;
  std::unique_ptr<GraphOptimizerInterface> graph_optimizer_ptr_;

  KeyFrame current_key_frame_;
  KeyFrame current_key_gnss_;

  int new_gnss_cnt_;
  int new_loop_cnt_;
  int new_key_frame_cnt_;

  Eigen::VectorXd odom_edge_noise_;
  Eigen::VectorXd close_loop_noise_;
  Eigen::VectorXd gnss_noise_;
};

}