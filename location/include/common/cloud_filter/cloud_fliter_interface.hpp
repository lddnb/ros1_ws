# pragma once
# include <sensor_data/cloud_data.h>

namespace location
{
namespace common
{
class CloudFilterInterface
{
public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData::CLOUD_PTR & input_cloud_ptr, CloudData::CLOUD_PTR & output_cloud_ptr) = 0;
};

}  // namespace common
}  // namespace location

