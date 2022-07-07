# pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

namespace location {
class OdometryPublisher {
    public:
    OdometryPublisher(ros::NodeHandle& nh,
                      std::string topic_name,
                      std::string base_frame_id,
                      std::string child_frame_id,
                      int buff_size);
    
    void Publish(Eigen::Matrix4f& transform_matrix);

    private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    nav_msgs::Odometry odometry_;
};

}
