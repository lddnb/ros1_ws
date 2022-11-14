#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_data/cloud_data.h"

namespace location {
class CloudPublisher{
    public:
    CloudPublisher(ros::NodeHandle& nh,
                   std::string topic_name,
                   long long buff_size,
                   std::string frame_id);
    
    void Publish(CloudData::CLOUD_PTR cloud_ptr_input);

    private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};

}



