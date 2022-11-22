#pragma once
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/pose_data.hpp"
#include "sensor_data/gnss_data.hpp"
#include "back_end/graph_optimizer_interface.hpp"
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
class BackEnd
{
public:
  BackEnd();
  ~BackEnd();
  bool Update(const CloudData & cloud_data, const PoseData & laser_odom, const PoseData & gnss_odom);
  bool InsertLoopPose();
  bool ForceOptimize();

  void GetOptimizedKeyFrames();
  bool HasNewKeyFrame();
  bool HasNewOptimized();
  void GetLatestKeyFrame(KeyFrame & key_frame);
  void GetLatestKeyGnss(KeyFrame & key_frame);

private:
  void InitParams();
  void ResetParams();
  bool AddNodeAndEdge(const PoseData & gnss_data);
  bool MaybeNewKeyFrame();
  bool MaybeOptimized();

private:
  float key_frame_distance_;
  bool has_new_key_frame_;
  bool has_new_optimized_;

  std::deque<KeyFrame> key_frame_deuqe_;
  std::deque<Eigen::Matrix4f> optimized_pose;
  std::unique_ptr<GraphOptimizerInterface> graph_optimizer_ptr_;

  int new_gnss_cnt_;
  int new_loop_cnt_;
  int new_key_frame_cnt_;
};

}