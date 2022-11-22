#pragma once 
#include <ros/ros.h>
#include <deque>
#include <geometry_msgs/TwistStamped.h>

#include "sensor_data/velocity_data.hpp"

namespace location {
class VelocitySubscriber {
    public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    void ParseData(std::deque<VelocityData>& deque_velocity_data);

    private:
    void msg_callback(const geometry_msgs::TwistStamped::ConstPtr& twist_msg_ptr);

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<VelocityData> new_velocity_data_;

    std::mutex buff_mutex_; 
};

}