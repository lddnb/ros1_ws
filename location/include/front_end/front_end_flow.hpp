# pragma once

#include <ros/ros.h>

#include "global_definition/global_definition.h"
#include "subscriber/cloud_subscriber.h"
#include "subscriber/gnss_subscriber.h"
#include "subscriber/imu_subscriber.h"
#include "subscriber/velocity_subscriber.h"
#include "publisher/cloud_publisher.h"
#include "publisher/odometry_publisher.h"
#include "tf_listener/tf_listener.h"
#include "front_end/front_end.hpp"

namespace location {
class FrontEndFlow
{
public:
    FrontEndFlow(const ros::NodeHandle & nh, const proto::FrontEndOptions & options);

    bool Run();
    bool PublishGlobalMap();

private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool PublishData();
    bool UpdateGNSSOdometry();
    bool UpdateLaserOdometry();

private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GnssSubscriber> gnss_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_tf_ptr_;
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    std::shared_ptr<FrontEnd> front_end_ptr_;

    std::deque<CloudData> cloud_data_buff_;
    std::deque<ImuData> imu_data_buff_;
    std::deque<GnssData> gnss_data_buff_;
    std::deque<VelocityData> velocity_data_buff_;

    Eigen::Matrix4f lidar_to_imu_;
    CloudData current_cloud_data_;
    ImuData current_imu_data_;
    VelocityData current_velocity_data_;
    GnssData current_gnss_data_;

    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR global_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;
    Eigen::Matrix4f gnss_odometry_;
    Eigen::Matrix4f laser_odometry_;
}

} // namespace location