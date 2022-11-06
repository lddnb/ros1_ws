#include <deque>
#include <memory>
#include <string>
#include <Eigen/Core>

#include "sensor_data/cloud_data.h"
#include "front_end/ndt.hpp"
#include "proto/front_end.pb.h"
#include "common/configuration_file_resolver.hpp"
#include "common/cloud_filter/voxel_filter.hpp"

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
    bool GetCurrentScan(CloudData::CLOUD_PTR & current_scan_ptr);
    bool GetNewLocalMap(CloudData::CLOUD_PTR & local_map_ptr);

private:
    // todo:根据options重载两个fliter
    bool Initfliter(const std::string & filter_name, const double & resolution);

    bool InitRegistration();
    bool UpdateLocalMap(const Frame & new_key_frame);

private:
    Eigen::Matrix4f init_pose_;
    std::unique_ptr<NDT> registration_ptr_;
    std::unique_ptr<common::CloudFilterInterface> frame_filter_;
    std::unique_ptr<common::CloudFilterInterface> local_map_filter_;
    std::unique_ptr<common::CloudFilterInterface> display_filter_ptr_;

    std::deque<Frame> local_map_frame_;
    CloudData::CLOUD_PTR local_map_ptr_;
    CloudData::CLOUD_PTR current_scan_ptr_;

    Eigen::Matrix4f step_pose;
    Eigen::Matrix4f last_pose;
    Eigen::Matrix4f predict_pose;
    Eigen::Matrix4f last_key_frame_pose;

    proto::FrontEndOptions options_;

    bool has_new_local_map_;
};

}