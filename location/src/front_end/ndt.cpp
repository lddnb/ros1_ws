#include <front_end/ndt.hpp>
#include <glog/logging.h>

namespace location {

NDT::NDT(const proto::FrontEndOptions & options)
: ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>())
{
    //ndt_ptr_ = boost::make_shared<pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>>();
    //为More-Thuente线搜索设置最大步长
    ndt_ptr_->setStepSize(options.step_size());
    //为终止条件设置最小转换差异
    ndt_ptr_->setTransformationEpsilon(options.trans_eps());
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt_ptr_->setResolution(options.res());
    //设置匹配迭代的最大次数
    ndt_ptr_->setMaximumIterations(options.max_iter());
    LOG(INFO) << "ndt params: step_size = " <<  options.step_size()
              << " TransformationEpsilon = " << options.trans_eps()
              << " Resolution = " << options.res()
              << " MaximumIterations " << options.max_iter();
}

NDT::~NDT() {}

void NDT::ScanMatch(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & current_cloud,
    const Eigen::Matrix4f & init_guess,
    CloudData::CLOUD_PTR & result_cloud,
    Eigen::Matrix4f & transform)
{
    auto start_time = std::chrono::system_clock::now();
    // LOG(INFO) << "current cloud point num = " << current_cloud->points.size();
    // LOG(INFO) << "target cloud point num = " << target_cloud_->points.size();

    ndt_ptr_->setInputCloud(current_cloud);
    
    //计算需要的刚体变换以便将输入的点云匹配到目标点云
    ndt_ptr_->align(*result_cloud, init_guess);
    // LOG(INFO) << " score: " << ndt_ptr_->getFitnessScore();
    // << "Normal Distributions Transform has converged:" << ndt_ptr_->hasConverged()

    transform = ndt_ptr_->getFinalTransformation();
    auto end_time = std::chrono::system_clock::now();
    // LOG(INFO) << "NDT cost time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count()
    //           << "ms";
}

void NDT::SetTargetCloud(CloudData::CLOUD_PTR target_cloud)
{
    target_cloud_ = target_cloud;
    //设置点云配准目标
    ndt_ptr_->setInputTarget(target_cloud_);
}

}