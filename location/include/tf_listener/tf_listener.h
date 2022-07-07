#pragma once

#include <tf/transform_listener.h> 
#include <ros/ros.h>
#include <Eigen/Dense>

namespace location {
class TFListener {
    public:
    TFListener(ros::NodeHandle nh, std::string base_frame_id, std::string child_frame_id);

    bool LookupData(Eigen::Matrix4f& transform_matrix);

    private:
    bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

    private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
};
}