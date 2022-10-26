#include "front_end/front_end_flow.hpp"

namespace location {

FrontEndFlow::FrontEndFlow(const ros::NodeHandle & nh, const proto::FrontEndOptions & options)
{
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    gnss_sub_ptr_ = std::make_shared<GnssSubscriber>(nh, "/kitti/oxts/gps/fix", 100000);
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 100000);
    lidar_to_imu_tf_ptr_ = std::make_shared<TFListener>(nh, "imu_link", "velo_link");
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "local_map", 100, "map");
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "global_map", 100, "map");
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 100);
    
    front_end_ptr_ = std::make_shared<FrontEnd>(options);

    lidar_to_imu_ = Eigen::Matrix4f::Identity();
    local_map_ptr_.reset(new CloudData::CLOUD());
    global_map_ptr_.reset(new CloudData::CLOUD());
    current_scan_ptr_.reset(new CloudData::CLOUD());
    gnss_odometry_ = Eigen::Matrix4f::Identity();
    laser_odometry_ = Eigen::Matrix4f::Identity();
}


} // namespace location



