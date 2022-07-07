#include "subscriber/cloud_subscriber.h"

namespace location {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::ParseData(std::deque<CloudData>& deque_cloud_data) {
    if (new_cloud_data_.size() > 0) {
        deque_cloud_data.insert(deque_cloud_data.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *cloud_data.cloud_ptr);

    new_cloud_data_.emplace_back(cloud_data);
}

}