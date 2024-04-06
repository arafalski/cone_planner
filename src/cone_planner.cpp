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

namespace
{

rrtstar_core::Pose poseMsgToPose(const geometry_msgs::msg::Pose& pose_msg)
{
  return rrtstar_core::Pose{
    pose_msg.position.x, pose_msg.position.y, tf2::getYaw(pose_msg.orientation)};
}

}  // namespace

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

Pose transformPose(const Pose& pose, const TransformStamped& transform)
{
  geometry_msgs::msg::Pose transformed_pose;
  tf2::doTransform(pose, transformed_pose, transform);

  return transformed_pose;
}

Pose global2local(const OccupancyGrid& costmap, const Pose& pose_global)
{
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin.inverse());

  return transformPose(pose_global, transform);
}

Pose local2global(const OccupancyGrid& costmap, const Pose& pose_local)
{
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  geometry_msgs::msg::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin);

  return transformPose(pose_local, transform);
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

bool ConePlanner::make_plan(const Pose& start_pose, const Pose& goal_pose)
{
  const rclcpp::Time begin = rclcpp::Clock(RCL_ROS_TIME).now();

  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);

  const auto is_obstacle_free = [&](const rrtstar_core::Pose & pose) {
    const int index_x = pose.x / costmap_.info.resolution;
    const int index_y = pose.y / costmap_.info.resolution;
    const int index_theta = discretizeAngle(pose.yaw, planner_param_.theta_size);
    return !detect_collision(IndexXYT{index_x, index_y, index_theta});
  };

  const rrtstar_core::Pose lo{0, 0, 0};
  const rrtstar_core::Pose hi{
    costmap_.info.resolution * costmap_.info.width,
    costmap_.info.resolution * costmap_.info.height,
    M_PI};
  const double radius = planner_param_.minimum_turning_radius;
  const auto cspace = rrtstar_core::CSpace(lo, hi, radius, is_obstacle_free);
  const auto x_start = poseMsgToPose(start_pose_);
  const auto x_goal = poseMsgToPose(goal_pose_);

  if (!is_obstacle_free(x_start)) {
    return false;
  }

  if (!is_obstacle_free(x_goal)) {
    return false;
  }

  const bool is_informed = true;
  const double collision_check_resolution = planner_param_.rrt_margin * 2;
  auto algo = rrtstar_core::RRTStar(
    x_start,
    x_goal,
    planner_param_.rrt_neighbor_radius,
    collision_check_resolution,
    is_informed,
    cspace);
  while (true) {
    const rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    const double msec = (now - begin).seconds() * 1000.0;

    if (msec > planner_param_.time_limit) {
      // break regardless of solution find or not
      break;
    }

    if (algo.isSolutionFound()) {
      if (!planner_param_.rrt_enable_update) {
        break;
      } else {
        if (msec > planner_param_.rrt_max_planning_time) {
          break;
        }
      }
    }

    algo.extend();
  }

  if (!algo.isSolutionFound()) {
    return false;
  }
  const auto waypoints = algo.sampleSolutionWaypoints();
  setRRTPath(waypoints);
  return true;
}

bool ConePlanner::detect_collision(const IndexXYT& base_index) const
{
  if (coll_indexes_table_.empty()) {
    std::cerr << "[abstract_algorithm] setMap has not yet been done." << std::endl;
    return false;
  }

  const auto& vertex_indexes_2d = vertex_indexes_table_[base_index.theta];
  for (const auto& vertex_index_2d : vertex_indexes_2d) {
    IndexXYT vertex_index{vertex_index_2d.x, vertex_index_2d.y, 0};
    // must slide to current base position
    vertex_index.x += base_index.x;
    vertex_index.y += base_index.y;
    if (is_out_of_range(vertex_index)) {
      return true;
    }
  }

  const auto& coll_indexes_2d = coll_indexes_table_[base_index.theta];
  for (const auto& coll_index_2d : coll_indexes_2d) {
    int idx_theta = 0;  // whatever. Yaw is nothing to do with collision detection between grids.
    IndexXYT coll_index{coll_index_2d.x, coll_index_2d.y, idx_theta};
    // must slide to current base position
    coll_index.x += base_index.x;
    coll_index.y += base_index.y;

    if (is_obs(coll_index)) {
      return true;
    }
  }

  return false;
}

void ConePlanner::setRRTPath(const std::vector<rrtstar_core::Pose>& waypoints)
{
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  header.frame_id = costmap_.header.frame_id;

  waypoints_.header = header;
  waypoints_.waypoints.clear();

  for (size_t i = 0; i < waypoints.size(); ++i) {
    auto & pt = waypoints.at(i);
    geometry_msgs::msg::Pose pose_local;
    pose_local.position.x = pt.x;
    pose_local.position.y = pt.y;
    pose_local.position.z = goal_pose_.position.z;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, pt.yaw);
    tf2::convert(quat, pose_local.orientation);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose = local2global(costmap_, pose_local);
    pose.header = header;
    PlannerWaypoint pw;
    if (0 == i) {
      const auto & pt_now = waypoints.at(i);
      const auto & pt_next = waypoints.at(i + 1);
      const double inner_product =
        cos(pt_now.yaw) * (pt_next.x - pt_now.x) + sin(pt_now.yaw) * (pt_next.y - pt_now.y);
      pw.is_back = (inner_product < 0.0);
    } else {
      const auto & pt_pre = waypoints.at(i - 1);
      const auto & pt_now = waypoints.at(i);
      const double inner_product =
        cos(pt_pre.yaw) * (pt_now.x - pt_pre.x) + sin(pt_pre.yaw) * (pt_now.y - pt_pre.y);
      pw.is_back = !(inner_product > 0.0);
    }
    pw.pose = pose;
    waypoints_.waypoints.push_back(pw);
  }
}

}  // namespace cone_planner
