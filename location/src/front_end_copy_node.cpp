#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <glog/logging.h>
#include <memory>

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "tf_listener/tf_listener.hpp"
#include "front_end/front_end.hpp"

#include <sstream>
#include <iostream>

using namespace location;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/ldd/catkin_ws/log";
    FLAGS_alsologtostderr = 1;

    LOG(INFO) << "glog success" << std::endl;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<GnssSubscriber> gnss_sub_ptr = std::make_shared<GnssSubscriber>(nh, "/kitti/oxts/gps/fix", 100000);
    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "gnss", "map", "lidar", 100);
    // std::shared_ptr<TFListener> lidar_to_imu_tf_ptr = std::make_shared<TFListener>(nh, "imu_link", "velo_link");

    std::deque<CloudData> cloud_data_buff;
    std::deque<ImuData> imu_data_buff;
    std::deque<GnssData> gnss_data_buff;

    std::string configuration_directory = "/home/ldd/catkin_ws/src/ros1_ws/location/configuration_files";
    std::string configuration_basename = "front_end_node.lua";
    auto file_resolver = std::make_unique<common::ConfigurationFileResolver>(std::vector<std::string>{configuration_directory});
    const std::string code =
        file_resolver->GetFileContentOrDie(configuration_basename);
    common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));
    proto::FrontEndOptions options = CreateFrontEndOptions(&lua_parameter_dictionary);
    LOG(INFO) << "option local_frame_num = " << options.local_frame_num();
    auto front_end_ptr = std::make_shared<FrontEnd>(options);

    int num = 0;
    
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        num++;
        cloud_sub_ptr->ParseData(cloud_data_buff);

        // static std::deque<ImuData> unsynced_imu_;
        // static std::deque<GnssData> unsynced_gnss_;

        // imu_sub_ptr->ParseData(unsynced_imu_);
        // gnss_sub_ptr->ParseData(unsynced_gnss_);

        // if (cloud_data_buff.size() == 0) {continue;}

        // double cloud_time = cloud_data_buff.front().time;
        // bool valid_imu = ImuData::SyncData(unsynced_imu_, imu_data_buff, cloud_time);
        // bool valid_gnss = GnssData::SyncData(unsynced_gnss_, gnss_data_buff, cloud_time);

        // static bool sensor_inited = false;
        // if (!sensor_inited)
        // {
        //     if (!valid_imu || !valid_gnss)
        //     {
        //         cloud_data_buff.pop_front();
        //         continue;
        //     }
        //     sensor_inited = true;
        // }

        // static bool gnss_inited = false;
        // if (!gnss_inited)
        // {
        //     GnssData gnss_data = gnss_data_buff.front();
        //     gnss_data.InitOriginPosition();
        //     gnss_inited = true;
        // }

        // if (cloud_data_buff.size() == 0)
        //     continue;
        // if (imu_data_buff.size() == 0)
        //     continue;
        // if (gnss_data_buff.size() == 0)
        //     continue;

        // while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
        while (cloud_data_buff.size()) {
            CloudData cloud_data = cloud_data_buff.front();
            // ImuData imu_data = imu_data_buff.front();
            // GnssData gnss_data = gnss_data_buff.front();

            // double d_time = cloud_data.time - imu_data.time;
            // if (d_time < -0.05)
            // {
            //     cloud_data_buff.pop_front();
            // }
            // else if (d_time > 0.05)
            // {
            //     imu_data_buff.pop_front();
            //     gnss_data_buff.pop_front();
            // }
            // else
            // {
                cloud_data_buff.pop_front();
                
                if (num % 3 == 0) {
                    num = 0;
                } else {
                    continue;
                }

                // imu_data_buff.pop_front();
                // gnss_data_buff.pop_front();

                Eigen::Matrix4f odometry_matrix = Eigen::Matrix4f::Identity();

                front_end_ptr->UpdateLaserOdom(cloud_data, odometry_matrix);
                float x, y, z, roll, pitch, yaw;
                pcl::getTranslationAndEulerAngles(Eigen::Affine3f(odometry_matrix), x, y, z, roll, pitch, yaw);
                LOG(INFO) << "x = " << x << ", y = " << y << ", z = " << z;
                LOG(INFO) << "roll = " << roll << ", pitch = " << pitch << ", yaw = " << yaw;

                CloudData cloud_data_transformed;
                pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data_transformed.cloud_ptr, odometry_matrix);

                // PCL体素滤波
                // CloudData filter_cloud;
                // pcl::VoxelGrid<pcl::PointXYZ> filter_;
                // filter_.setLeafSize(0.25f, 0.25f, 0.25f);
                // filter_.setInputCloud(cloud_data_transformed.cloud_ptr);
                // filter_.filter(*filter_cloud.cloud_ptr);

                cloud_pub_ptr->Publish(cloud_data_transformed.cloud_ptr);
                odom_pub_ptr->Publish(odometry_matrix);
            // }
        }
        rate.sleep();
    }
}