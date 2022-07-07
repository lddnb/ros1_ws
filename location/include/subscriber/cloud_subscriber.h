#pragma once

#include <ros/ros.h>
#include <deque>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "sensor_data/cloud_data.h"

namespace location {
class CloudSubscriber {
    public:
    CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    void ParseData(std::deque<CloudData>& deque_cloud_data);

    private:
    void msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<CloudData> new_cloud_data_;
};
}