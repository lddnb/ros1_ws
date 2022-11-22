# pragma once

#include <ros/ros.h>

#include "global_definition/global_definition.h"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/velocity_subscriber.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "tf_listener/tf_listener.hpp"
#include "front_end/front_end.hpp"
#include "common/distortion/distortion.hpp"

namespace location {
class FrontEndFlow
{
public:
    FrontEndFlow(ros::NodeHandle & nh, const proto::FrontEndOptions & options);

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
    bool TransformData();

private:
    std::unique_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::unique_ptr<IMUSubscriber> imu_sub_ptr_;
    std::unique_ptr<GnssSubscriber> gnss_sub_ptr_;
    std::unique_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::unique_ptr<TFListener> lidar_to_imu_tf_ptr_;
    std::unique_ptr<CloudPublisher> cloud_pub_ptr_;
    std::unique_ptr<CloudPublisher> local_map_pub_ptr_;
    std::unique_ptr<CloudPublisher> global_map_pub_ptr_;
    std::unique_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::unique_ptr<OdometryPublisher> gnss_pub_ptr_;
    std::unique_ptr<FrontEnd> front_end_ptr_;
    std::unique_ptr<DistortionAdjust> distortion_adjust_ptr_;

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
};

} // namespace location