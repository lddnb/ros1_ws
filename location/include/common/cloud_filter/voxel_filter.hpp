#pragma once

#include "common/cloud_filter/cloud_fliter_interface.hpp"
#include "pcl/filters/voxel_grid.h"

namespace location
{
namespace common
{
class VoxelFilter : public CloudFilterInterface 
{
public:
    explicit VoxelFilter(const double & leaf_size);
    VoxelFilter(const double & leaf_size_x, const double & leaf_size_y, const double & leaf_size_z);

    bool Filter(const CloudData::CLOUD_PTR & input_cloud_ptr, CloudData::CLOUD_PTR & output_cloud_ptr) override;
private:
    bool SetFilterParam(const double & leaf_size_x, const double & leaf_size_y, const double & leaf_size_z);

private:
    pcl::VoxelGrid<CloudData::POINT> Voxel_filter_;
};
}  // namespace common
}  // namespace location
