#pragma once

#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>

namespace location {
class TFListener {
    public:
    TFListener(ros::NodeHandle nh, std::string base_frame_id, std::string child_frame_id);

    bool LookupData(Eigen::Matrix4f& transform_matrix);

    private:
    bool TransformToMatrix(const geometry_msgs::TransformStamped& transform, Eigen::Matrix4f & transform_matrix);

    private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;
};
}