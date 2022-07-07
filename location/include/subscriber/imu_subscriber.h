#pragma once

#include <ros/ros.h>
#include <deque>
#include <sensor_msgs/Imu.h>

#include "sensor_data/imu_data.h"

namespace location {
class IMUSubscriber {
    public:
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);

    void ParseData(std::deque<ImuData>& deque_imu_data);


    private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<ImuData> new_imu_data_;
};

}