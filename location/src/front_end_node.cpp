#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <glog/logging.h>
#include <memory>

#include "subscriber/cloud_subscriber.h"
#include "publisher/cloud_publisher.h"
#include "publisher/odometry_publisher.h"
#include "tf_listener/tf_listener.h"
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
    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);

    std::deque<CloudData> cloud_data_buff;

    std::string configuration_directory = "/home/ldd/catkin_ws/src/ros1_ws/location/configuration_files";
    std::string configuration_basename = "front_end_node.lua";
    auto file_resolver = std::make_unique<common::ConfigurationFileResolver>(std::vector<std::string>{configuration_directory});
    const std::string code =
        file_resolver->GetFileContentOrDie(configuration_basename);
    common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));
    proto::FrontEndOptions options = CreateFrontEndOptions(&lua_parameter_dictionary);
    LOG(INFO) << "option local_frame_num = " << options.local_frame_num();
    auto front_end_ptr = std::make_shared<FrontEnd>(options);
    
    ros::Rate rate(10);
    while (ros::ok) {
        ros::spinOnce();
        LOG(INFO) << "cloud size " << cloud_sub_ptr->GetCloudNum();
        cloud_sub_ptr->ParseData(cloud_data_buff);
        LOG(INFO) << "cloud_data_buff.size() = " << cloud_data_buff.size();
        if (cloud_data_buff.size() > 0) {
            CloudData cloud_data = cloud_data_buff.front();
            cloud_data_buff.pop_front();

            Eigen::Matrix4f odometry_matrix;

            front_end_ptr->UpdateLaserOdom(cloud_data, odometry_matrix);
            LOG(INFO) << "UpdateLaserOdom success!";

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
        }
        LOG(INFO) << "spinonce success";
        rate.sleep();
    }
}