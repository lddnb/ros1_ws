#include "common/cloud_filter/voxel_filter.hpp"
#include "glog/logging.h"

namespace location
{
namespace common
{
VoxelFilter::VoxelFilter(const double & leaf_size)
{
    SetFilterParam(leaf_size, leaf_size, leaf_size);
}

VoxelFilter::VoxelFilter(const double & leaf_size_x, const double & leaf_size_y, const double & leaf_size_z)
{
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::SetFilterParam(const double & leaf_size_x, const double & leaf_size_y, const double & leaf_size_z)
{
    Voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    LOG(INFO) << "Voxel Filter Param: " << leaf_size_x << ", " << leaf_size_y << ", " << leaf_size_z;
    return true;
}

bool VoxelFilter::Filter(const CloudData::CLOUD_PTR & input_cloud_ptr, CloudData::CLOUD_PTR & output_cloud_ptr)
{
    Voxel_filter_.setInputCloud(input_cloud_ptr);
    Voxel_filter_.filter(*output_cloud_ptr);
    return true;
}
}  // namespace common
}  // namespace location
