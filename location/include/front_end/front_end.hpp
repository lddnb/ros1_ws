#include <deque>
#include <memory>
#include <string>
#include <Eigen/Core>

#include "common/voxel_filter.h"
#include "sensor_data/cloud_data.h"
#include "front_end/ndt.hpp"
#include "proto/front_end.pb.h"
#include "common/configuration_file_resolver.hpp"

namespace location {

proto::FrontEndOptions CreateFrontEndOptions(common::LuaParameterDictionary* const lua_parameter_dictionary);

class FrontEnd
{
public:
    struct Frame { 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };
public:
    FrontEnd(const proto::FrontEndOptions & options);
    void UpdateLaserOdom(const CloudData & cloud_data, Eigen::Matrix4f & laser_odom);
    void SetInitPose(Eigen::Matrix4f & init_pose);

private:
    // todo:根据options重载两个fliter
    bool Initfliter(const std::string & filter_name, const double & resolution);

    bool InitRegistration();
    bool UpdateLocalMap(const Frame & new_key_frame);

private:
    Eigen::Matrix4f init_pose_;
    std::shared_ptr<NDT> registration_ptr_;
    std::shared_ptr<common::Filter> frame_filter_;
    std::shared_ptr<common::Filter> local_map_filter_;

    std::deque<Frame> local_map_frame_;
    CloudData::CLOUD_PTR local_map_ptr_;

    Eigen::Matrix4f step_pose;
    Eigen::Matrix4f last_pose;
    Eigen::Matrix4f predict_pose;
    Eigen::Matrix4f last_key_frame_pose;

    proto::FrontEndOptions options_;
};

}