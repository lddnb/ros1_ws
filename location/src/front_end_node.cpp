#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <glog/logging.h>
#include <memory>

#include "subscriber/cloud_subscriber.h"
#include "subscriber/gnss_subscriber.h"
#include "subscriber/imu_subscriber.h"
#include "publisher/cloud_publisher.h"
#include "publisher/odometry_publisher.h"
#include "tf_listener/tf_listener.h"
#include "front_end/front_end_flow.hpp"

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

    std::string configuration_directory = "/home/ldd/catkin_ws/src/ros1_ws/location/configuration_files";
    std::string configuration_basename = "front_end_node.lua";
    auto file_resolver = std::make_unique<common::ConfigurationFileResolver>(std::vector<std::string>{configuration_directory});
    const std::string code =
        file_resolver->GetFileContentOrDie(configuration_basename);
    common::LuaParameterDictionary lua_parameter_dictionary(code, std::move(file_resolver));
    proto::FrontEndOptions options = CreateFrontEndOptions(&lua_parameter_dictionary);
    LOG(INFO) << "option local_frame_num = " << options.local_frame_num();
    auto front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh, options);

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        front_end_flow_ptr->Run();
        rate.sleep();
    }
    return 0;
}