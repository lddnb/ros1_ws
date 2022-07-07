#pragma once 

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "sensor_data/gnss_data.h"

namespace location {
class GnssSubscriber {
    public:
    GnssSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    void ParseData(std::deque<GnssData>& deque_gnss_data);

    private:
    void msg_callback(const sensor_msgs::NavSatFix::ConstPtr& nav_sat_fix_ptr);

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<GnssData> new_gnss_data_;
};

}