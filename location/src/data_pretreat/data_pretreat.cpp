#include "data_pretreat/data_pretreat.hpp"

namespace location {

FrontEndFlow::FrontEndFlow(ros::NodeHandle & nh, const proto::FrontEndOptions & options)
{
    cloud_sub_ptr_ = std::make_unique<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    imu_sub_ptr_ = std::make_unique<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    gnss_sub_ptr_ = std::make_unique<GnssSubscriber>(nh, "/kitti/oxts/gps/fix", 100000);
    velocity_sub_ptr_ = std::make_unique<VelocitySubscriber>(nh, "/kitti/oxts/gps/vel", 100000);
    lidar_to_imu_tf_ptr_ = std::make_unique<TFListener>(nh, "imu_link", "velo_link");
    cloud_pub_ptr_ = std::make_unique<CloudPublisher>(nh, "current_scan", 100, "map");
    local_map_pub_ptr_ = std::make_unique<CloudPublisher>(nh, "local_map", 100, "map");
    global_map_pub_ptr_ = std::make_unique<CloudPublisher>(nh, "global_map", 100, "map");
    laser_odom_pub_ptr_ = std::make_unique<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);
    gnss_pub_ptr_ = std::make_unique<OdometryPublisher>(nh, "gnss", "map", "lidar", 100);
    distortion_adjust_ptr_ = std::make_unique<DistortionAdjust>();
    
    front_end_ptr_ = std::make_unique<FrontEnd>(options);

    lidar_to_imu_ = Eigen::Matrix4f::Identity();
    local_map_ptr_.reset(new CloudData::CLOUD());
    global_map_ptr_.reset(new CloudData::CLOUD());
    current_scan_ptr_.reset(new CloudData::CLOUD());
    gnss_odometry_ = Eigen::Matrix4f::Identity();
    laser_odometry_ = Eigen::Matrix4f::Identity();
}

bool FrontEndFlow::Run()
{
    if (!ReadData()) {
        return false;
    }
    if (!InitCalibration()) {
        return false;
    }
    if (!InitGNSS()) {
        return false;
    }

    while (HasData()) {
        if (!ValidData()) {
            return false;
        }
        TransformData();
        UpdateGNSSOdometry();
        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }
    return true;
}

bool FrontEndFlow::ReadData()
{
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    
    static std::deque<ImuData> unsync_imu_;
    static std::deque<VelocityData> unsync_velo_;
    static std::deque<GnssData> unsync_gnss_;

    imu_sub_ptr_->ParseData(unsync_imu_);
    velocity_sub_ptr_->ParseData(unsync_velo_);
    gnss_sub_ptr_->ParseData(unsync_gnss_);

    if (cloud_data_buff_.size() == 0) {
        return false;
    }

    double cloud_time = cloud_data_buff_.front().time;
    bool valid_imu = ImuData::SyncData(unsync_imu_, imu_data_buff_, cloud_time);
    bool valid_velo = VelocityData::SyncData(unsync_velo_, velocity_data_buff_, cloud_time);
    bool valid_gnss =  GnssData::SyncData(unsync_gnss_, gnss_data_buff_, cloud_time);

    static bool sensor_init = false;

    if (!sensor_init) {
        if (!valid_imu || !valid_velo || !valid_gnss) {
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_init = true;
    }

    return true;
}

bool FrontEndFlow::InitCalibration()
{
    static bool calibration_received = false;
    if (!calibration_received && lidar_to_imu_tf_ptr_->LookupData(lidar_to_imu_)) {
        calibration_received = true;
    }
    return calibration_received;
}

bool FrontEndFlow::InitGNSS()
{
    static bool gnss_init = false;
    if (!gnss_init) {
        GnssData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_init = true;
    }
    return gnss_init;
}

bool FrontEndFlow::HasData()
{
    if (cloud_data_buff_.size() == 0 ||
        imu_data_buff_.size() == 0 ||
        velocity_data_buff_.size() == 0 ||
        gnss_data_buff_.size() == 0) 
    {
        return false;
    } else {
        return true;
    }
}

bool FrontEndFlow::ValidData()
{
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double delta_time = current_cloud_data_.time - current_imu_data_.time;
    if (delta_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    } else if (delta_time > 0.05) {
        imu_data_buff_.pop_front();
        velocity_data_buff_.pop_front();
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();
    return true;
}

bool FrontEndFlow::TransformData()
{
  current_velocity_data_.TransformCoordinate(lidar_to_imu_);
  distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
  distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);
  return true;
}

bool FrontEndFlow::UpdateGNSSOdometry()
{
    gnss_odometry_ = Eigen::Matrix4f::Identity();
    current_gnss_data_.UpdateXYZ();
    gnss_odometry_(0, 3) = current_gnss_data_.local_E;
    gnss_odometry_(2, 3) = current_gnss_data_.local_U;
    gnss_odometry_(1, 3) = current_gnss_data_.local_N;
    gnss_odometry_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();
    gnss_odometry_ *= lidar_to_imu_;
    return true;
}

bool FrontEndFlow::UpdateLaserOdometry()
{
    static bool front_end_pose_init = false;
    if (!front_end_pose_init) {
        front_end_pose_init = true;
        front_end_ptr_->SetInitPose(gnss_odometry_);
    }
    laser_odometry_ = Eigen::Matrix4f::Identity();
    front_end_ptr_->UpdateLaserOdom(current_cloud_data_, laser_odometry_);
    
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles (Eigen::Affine3f(laser_odometry_), x, y, z, roll, pitch, yaw);
    LOG(INFO) << "x = " << x << ", y = " << y << ", z = " << z;
    LOG(INFO) << "roll = " << roll << ", pitch = " << pitch << ", yaw = " << yaw;
    return true;
}

bool FrontEndFlow::PublishData()
{
    gnss_pub_ptr_->Publish(gnss_odometry_);
    laser_odom_pub_ptr_->Publish(laser_odometry_);

    front_end_ptr_->GetCurrentScan(current_scan_ptr_);
    cloud_pub_ptr_->Publish(current_scan_ptr_);

    if (front_end_ptr_->GetNewLocalMap(local_map_ptr_)) {
        local_map_pub_ptr_->Publish(local_map_ptr_);
    }
    return true;
}


} // namespace location
