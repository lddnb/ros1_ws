#pragma once

#include <ros/ros.h>
#include <deque>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "sensor_data/cloud_data.hpp"

namespace location {
class CloudSubscriber {
    public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    void ParseData(std::deque<CloudData>& deque_cloud_data);

    int GetCloudNum() const {return new_cloud_data_.size();}

    private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<CloudData> new_cloud_data_;

    std::mutex buff_mutex_; 
};
}