#include "subscriber/gnss_subscriber.h"

namespace location {
GnssSubscriber::GnssSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &GnssSubscriber::msg_callback, this);
}    

void GnssSubscriber::ParseData(std::deque<GnssData>& deque_gnss_data) {
    if (new_gnss_data_.size() > 0) {
        deque_gnss_data.insert(deque_gnss_data.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
    }
}

void GnssSubscriber::msg_callback(const sensor_msgs::NavSatFix::ConstPtr& nav_sat_fix_ptr) {
    GnssData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->altitude;
    gnss_data.status = nav_sat_fix_ptr->status.status;
    gnss_data.service = nav_sat_fix_ptr->status.service;

    new_gnss_data_.emplace_back(gnss_data);
}

}