#pragma once
#include <sensor_data/cloud_data.hpp>
#include <pcl/registration/ndt.h>
#include <Eigen/Core>
#include "proto/front_end.pb.h"

namespace location {
class NDT
{
public:
    NDT(const proto::FrontEndOptions & options);
    ~NDT();
    void ScanMatch(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr & current_cloud,
      const Eigen::Matrix4f & init_guess,
      CloudData::CLOUD_PTR & result_cloud,
      Eigen::Matrix4f & transform);
    void SetTargetCloud(CloudData::CLOUD_PTR target_cloud);
private:
    CloudData::CLOUD_PTR target_cloud_;
    pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr ndt_ptr_;
};

}