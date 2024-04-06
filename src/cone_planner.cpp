// Copyright 2024 Andrzej Rafalski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cone_planner/cone_planner.hpp"

#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/normalization.hpp>
#include <tf2/utils.h>

#include <vector>

namespace cone_planner
{
using geometry_msgs::msg::Pose;
using tier4_autoware_utils::createQuaternionFromYaw;
using tier4_autoware_utils::normalizeRadian;

int discretizeAngle(const double theta, const int theta_size)
{
  const double one_angle_range = 2.0 * M_PI / theta_size;
  return static_cast<int>(std::rint(normalizeRadian(theta, 0.0) / one_angle_range)) % theta_size;
}

IndexXYT pose2index(
  const OccupancyGrid& costmap,
  const Pose & pose_local,
  const int theta_size)
{
  const int index_x = pose_local.position.x / costmap.info.resolution;
  const int index_y = pose_local.position.y / costmap.info.resolution;
  const int index_theta = discretizeAngle(tf2::getYaw(pose_local.orientation), theta_size);
  return {index_x, index_y, index_theta};
}

Pose index2pose(
  const OccupancyGrid& costmap,
  const IndexXYT& index,
  const int theta_size)
{
  Pose pose_local;

  pose_local.position.x = index.x * costmap.info.resolution;
  pose_local.position.y = index.y * costmap.info.resolution;

  const double one_angle_range = 2.0 * M_PI / theta_size;
  const double yaw = index.theta * one_angle_range;
  pose_local.orientation = createQuaternionFromYaw(yaw);

  return pose_local;
}

void ConePlanner::set_map(const OccupancyGrid& costmap) {
  costmap_ = costmap;
  const auto height = costmap_.info.height;
  const auto width = costmap_.info.width;

  std::vector<std::vector<bool>> is_obstacle_table;
  is_obstacle_table.resize(height);
  for (uint32_t i = 0; i < height; i++) {
    is_obstacle_table.at(i).resize(width);
    for (uint32_t j = 0; j < width; j++) {
      const int cost = costmap_.data[i * width + j];

      if (cost < 0 || planner_param_.obstacle_threshold <= cost) {
        is_obstacle_table[i][j] = true;
      }
    }
  }
  is_obstacle_table_ = is_obstacle_table;

  if (!is_collision_table_initialized_) {
    for (int i = 0; i < planner_param_.theta_size; i++) {
      std::vector<IndexXY> indexes_2d;
      std::vector<IndexXY> vertex_indexes_2d;

      compute_collision_indexes(i, indexes_2d, vertex_indexes_2d);

      coll_indexes_table_.push_back(indexes_2d);
      vertex_indexes_table_.push_back(vertex_indexes_2d);
    }

    is_collision_table_initialized_ = true;
  }
}

void ConePlanner::compute_collision_indexes(
  int theta_index,
  std::vector<IndexXY>& indexes_2d,
  std::vector<IndexXY>& vertex_indexes_2d
) const
{
  IndexXYT base_index{0, 0, theta_index};
  const auto& vehicle_shape = collision_vehicle_shape_;

  // Define the robot as rectangle
  const double back = -1.0 * vehicle_shape.base2back;
  const double front = vehicle_shape.length - vehicle_shape.base2back;
  const double right = -1.0 * vehicle_shape.width / 2.0;
  const double left = vehicle_shape.width / 2.0;

  const auto base_pose = index2pose(costmap_, base_index, planner_param_.theta_size);
  const auto base_theta = tf2::getYaw(base_pose.orientation);

  // Convert each point to index and check if the node is Obstacle
  const auto addIndex2d = [&](const double x,
                              const double y,
                              std::vector<IndexXY>& indexes_cache) {
    // Calculate offset in rotated frame
    const double offset_x = std::cos(base_theta) * x - std::sin(base_theta) * y;
    const double offset_y = std::sin(base_theta) * x + std::cos(base_theta) * y;

    Pose pose_local;
    pose_local.position.x = base_pose.position.x + offset_x;
    pose_local.position.y = base_pose.position.y + offset_y;

    const auto index = pose2index(costmap_, pose_local, planner_param_.theta_size);
    const auto index_2d = IndexXY{index.x, index.y};
    indexes_cache.push_back(index_2d);
  };

  for (double x = back; x <= front; x += costmap_.info.resolution / 2) {
    for (double y = right; y <= left; y += costmap_.info.resolution / 2) {
      addIndex2d(x, y, indexes_2d);
    }
    addIndex2d(x, left, indexes_2d);
  }
  for (double y = right; y <= left; y += costmap_.info.resolution / 2) {
    addIndex2d(front, y, indexes_2d);
  }
  addIndex2d(front, left, indexes_2d);

  const auto compareIndex2d = [](const IndexXY & left, const IndexXY & right) {
    if (left.x != right.x) {
      return (left.x < right.x);
    } else {
      return (left.y < right.y);
    }
  };

  const auto equalIndex2d = [](const IndexXY & left, const IndexXY & right) {
    return ((left.x == right.x) && (left.y == right.y));
  };

  // remove duplicate indexes
  std::sort(indexes_2d.begin(), indexes_2d.end(), compareIndex2d);
  indexes_2d.erase(
    std::unique(indexes_2d.begin(), indexes_2d.end(), equalIndex2d), indexes_2d.end());

  addIndex2d(front, left, vertex_indexes_2d);
  addIndex2d(front, right, vertex_indexes_2d);
  addIndex2d(back, right, vertex_indexes_2d);
  addIndex2d(back, left, vertex_indexes_2d);
}

}  // namespace cone_planner
