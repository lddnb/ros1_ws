/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "common/voxel_filter.h"
#include "Eigen/Core"

#include <cmath>
#include <random>
#include <utility>
#include <unordered_map>

namespace common {
namespace {
using VoxelKeyType = uint64_t;

// 计算该点处于的voxel的序号
VoxelKeyType GetVoxelCellIndex(const pcl::PointXYZ& point,
                               const float resolution) {
  const uint64_t x = std::lround(point.x / resolution);
  const uint64_t y = std::lround(point.y / resolution);
  const uint64_t z = std::lround(point.z / resolution);
  return (x << 42) + (y << 21) + z;
}

// 进行体素滤波, 标记体素滤波后的点
std::vector<bool> RandomizedVoxelFilterIndices(
    const std::vector<pcl::PointXYZ>& point_cloud, const float resolution) {
  // According to https://en.wikipedia.org/wiki/Reservoir_sampling
  std::minstd_rand0 generator;
  // std::pair<int, int>的第一个元素保存该voxel内部的点的个数, 第二个元素保存该voxel中选择的那一个点的序号
  std::unordered_map<VoxelKeyType, std::pair<int, int>>
      voxel_count_and_point_index;
  // 遍历所有的点, 计算
  for (size_t i = 0; i < point_cloud.size(); i++) {
    // 获取VoxelKeyType对应的value的引用
    auto& voxel = voxel_count_and_point_index[GetVoxelCellIndex(
      point_cloud[i], resolution)];
    voxel.first++;
    // 如果这个体素格子只有1个点, 那这个体素格子里的点的索引就是i
    if (voxel.first == 1) {
      voxel.second = i;
    } 
    else {
      // 生成随机数的范围是 [1, voxel.first]
      std::uniform_int_distribution<> distribution(1, voxel.first);
      // 生成的随机数与个数相等, 就让这个点代表这个体素格子
      if (distribution(generator) == voxel.first) {
        voxel.second = i;
      }
    }
  }
  // 为体素滤波之后的点做标记
  std::vector<bool> points_used(point_cloud.size(), false);
  for (const auto& voxel_and_index : voxel_count_and_point_index) {
    points_used[voxel_and_index.second.second] = true;
  }
  return points_used;
}

}  // namespace


// 进行体素滤波
CloudData VoxelFilter(const CloudData & point_cloud, const float & resolution) {
  // 得到标记后的点
  std::vector<pcl::PointXYZ> point;
  for (const auto & i : point_cloud.cloud_ptr->points) {
    point.emplace_back(i);
  }
  const std::vector<bool> points_used = RandomizedVoxelFilterIndices(
      point, resolution);

  // 生成滤波后的点云
  CloudData result;
  for (size_t i = 0; i < point_cloud.cloud_ptr->points.size(); i++) {
    if (points_used[i]) {
      result.cloud_ptr->points.emplace_back(point_cloud.cloud_ptr->points[i]);
    }
  }

  return result;
}

}  // namespace common
