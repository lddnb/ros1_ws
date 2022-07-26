#include <sensor_data/cloud_data.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Core>

class NDT
{
public:
    NDT();
    ~NDT();
    void match(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr & reference_cloud,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr & current_cloud,
      const Eigen::Matrix4f & init_guess,
      Eigen::Matrix4f & transform);
private:
    


};