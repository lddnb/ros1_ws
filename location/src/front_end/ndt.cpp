#include <front_end/ndt.hpp>

void NDT::match(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & reference_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & current_cloud,
    const Eigen::Matrix4f & init_guess,
    Eigen::Matrix4f & transform)
{
    //初始化正态分布变换（NDT）
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    //设置依赖尺度NDT参数
    //为终止条件设置最小转换差异
    ndt.setTransformationEpsilon(0.01);
    //为More-Thuente线搜索设置最大步长
    ndt.setStepSize(0.1);
    //设置NDT网格结构的分辨率（VoxelGridCovariance）
    ndt.setResolution(50);
    //设置匹配迭代的最大次数
    ndt.setMaximumIterations(10);
    // 设置要配准的点云
    ndt.setInputCloud(current_cloud);
    //设置点云配准目标
    ndt.setInputTarget(reference_cloud);
    //设置使用机器人测距法得到的初始对准估计结果
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
    //计算需要的刚体变换以便将输入的点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);
    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged()
              << " score: " << ndt.getFitnessScore() << std::endl;
    //使用创建的变换对未过滤的输入点云进行变换
    //pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
}