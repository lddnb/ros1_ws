#include <pcl/common/transforms.h>
#include <glog/logging.h>
#include <memory>

#include "global_definition/global_definition.h"
#include "subscriber/cloud_subscriber.h"
#include "subscriber/gnss_subscriber.h"
#include "subscriber/imu_subscriber.h"
#include "publisher/cloud_publisher.h"
#include "publisher/odometry_publisher.h"
#include "tf_listener/tf_listener.h"

#include "common/configuration_file_resolver.hpp"
#include "proto/test.pb.h"
#include "proto/test2.pb.h"
#include "common/voxel_filter.h"

#include <sstream>
#include <iostream>

using namespace location;

proto::Test2 CreateTest2options(common::LuaParameterDictionary* const lua_parameter_dictionary) {
    proto::Test2 options;
    options.set_a(lua_parameter_dictionary->GetDouble("a"));
    options.set_b(lua_parameter_dictionary->GetDouble("b"));
    options.set_c(lua_parameter_dictionary->GetDouble("c"));
    return options;
}

proto::TestInputMessage CreateTestInputoptions(common::LuaParameterDictionary* const lua_parameter_dictionary) {
    proto::TestInputMessage options;
    *options.mutable_test1() = CreateTest2options(
        lua_parameter_dictionary->GetDictionary("test1").get());
    options.set_intput_a(lua_parameter_dictionary->GetDouble("intput_a"));
    options.set_intput_b(lua_parameter_dictionary->GetDouble("intput_b"));
    options.set_intput_c(lua_parameter_dictionary->GetDouble("intput_c"));
    return options;
}


std::vector<std::string> split(const std::string& s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = "/home/ldd/catkin_ws/log";
    FLAGS_alsologtostderr = 1;

    LOG(INFO) << "glog success" << std::endl;

    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 100000);
    std::shared_ptr<GnssSubscriber> gnss_sub_ptr = std::make_shared<GnssSubscriber>(nh, "/kitti/oxts/gps/fix", 100000);
    std::shared_ptr<TFListener> lidar_to_imu_tf_ptr = std::make_shared<TFListener>(nh, "imu_link", "velo_link");

    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", 100, "map");
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);

    std::deque<CloudData> cloud_data_buff;
    std::deque<ImuData> imu_data_buff;
    std::deque<GnssData> gnss_data_buff;
    
    Eigen::Matrix4f lidar_to_imu_tf = Eigen::Matrix4f::Identity();
    bool transform_received = false;
    bool gnss_origin_position_inited = false;

    std::string configuration_directory = "/home/ldd/catkin_ws/src/ros1_ws/location/configuration_files";
    std::string configuration_basename = "configuration.lua";
    //std::vector<std::string> configuration_directorys = split(configuration_directory, '/');
    auto file_resolver = std::make_unique<common::ConfigurationFileResolver>(std::vector<std::string>{configuration_directory});
    const std::string code =
        file_resolver->GetFileContentOrDie(configuration_basename);
    common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));
    proto::TestInputMessage options = CreateTestInputoptions(&lua_parameter_dictionary);
    LOG(INFO) << "option intput_a = " << options.intput_a();
    //! cout必须带endl才会在终端输出！！！
    //! 下面三句没有输出
    std::cout << "option intput_a = " << options.intput_a() << std::endl;
    std::cout << "option intput_b = " << options.intput_b() << std::endl;
    std::cout << "option intput_c = " << options.intput_c() << std::endl;
    std::cout << "option test1 a = " << options.test1().a();
    std::cout << "option test1 b = " << options.test1().b();
    std::cout << "option test1 c = " << options.test1().c();

    ros::Rate rate(100);
    while (ros::ok) {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff);
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        if (!transform_received) {
            if (lidar_to_imu_tf_ptr->LookupData(lidar_to_imu_tf)) {
                transform_received = true;
                LOG(INFO) << "lidar to imu transform matrix is:" << std::endl << lidar_to_imu_tf;
            }
        }
        else {
            while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
                CloudData cloud_data = cloud_data_buff.front();
                ImuData imu_data = imu_data_buff.front();
                GnssData gnss_data = gnss_data_buff.front();

                double d_time = cloud_data.time - imu_data.time;
                if (d_time < -0.05) {
                    cloud_data_buff.pop_front();
                }
                else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                }
                else {
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();

                    Eigen::Matrix4f odometry_matrix;

                    if (!gnss_origin_position_inited) {
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }
                    gnss_data.UpdateXYZ();
                    odometry_matrix(0, 3) = gnss_data.local_E;
                    odometry_matrix(1, 3) = gnss_data.local_N;
                    odometry_matrix(2, 3) = gnss_data.local_U;
                    odometry_matrix.block<3, 3>(0, 0) = imu_data.GetOrientationMatrix();
                    odometry_matrix *= lidar_to_imu_tf;
                    
                    CloudData cloud_data_transformed;
                    pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data_transformed.cloud_ptr, odometry_matrix);
                    //LOG(INFO) << odometry_matrix;

                    CloudData filter_cloud = common::VoxelFilter(cloud_data_transformed, 0.5);

                    cloud_pub_ptr->Publish(filter_cloud.cloud_ptr);
                    odom_pub_ptr->Publish(odometry_matrix);
                }
            }
        }
        rate.sleep();
    }
}