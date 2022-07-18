#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace location {
class CloudData {
public:
    using POINT = pcl::PointXYZ;
    using CLOUD = pcl::PointCloud<POINT>;
    using CLOUD_PTR = CLOUD::Ptr;

    //TODO:CLOUD_PTR() 
    CloudData():cloud_ptr(new CLOUD()) {}
    CloudData(std::vector<POINT> points) {
        cloud_ptr->clear();
        for (const auto & i : points) {
            cloud_ptr->emplace_back(i);
        }
    }

public:
    double time = 0.0;
    CLOUD_PTR cloud_ptr;
};
}