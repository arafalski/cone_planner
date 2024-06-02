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

#ifndef CONE_PLANNER__CONE_PLANNER_NODE_HPP_
#define CONE_PLANNER__CONE_PLANNER_NODE_HPP_

#include <deque>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "cone_planner/cone_planner.hpp"

namespace cone_planner
{
using ConePlannerPtr = std::unique_ptr<cone_planner::ConePlanner>;
using autoware_auto_planning_msgs::msg::Trajectory;
using nav_msgs::msg::OccupancyGrid;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;

class CONE_PLANNER_PUBLIC ConePlannerNode : public rclcpp::Node
{
public:
  explicit ConePlannerNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<Trajectory>::SharedPtr planned_trajectory_pub_;

  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  ConePlannerParam get_planner_param();

  void on_odometry(const Odometry::SharedPtr msg);

  void onTimer();
  void reset();
  void planTrajectory(const PoseStamped& goal_pose);
  PoseStamped get_closest_pose();
  void update_target_index();
  bool is_plan_required();

  TransformStamped get_transform(const std::string& from, const std::string& to);

  double waypoints_velocity_;
  double th_arrived_distance_m_;
  double th_stopped_time_sec_;
  double th_stopped_velocity_mps_;
  double th_course_out_distance_m_;
  double c_space_margin_;
  bool replan_when_obstacle_found_;
  bool replan_when_course_out_;
  VehicleShape vehicle_shape_;

  Trajectory::SharedPtr trajectory_;
  OccupancyGrid::SharedPtr occupancy_grid_;
  PoseStamped::SharedPtr pose_;
  Odometry::SharedPtr odom_;
  std::deque<Odometry::SharedPtr> odom_buffer_;

  Trajectory planned_trajectory_;
  Trajectory partial_planned_trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_;
  size_t target_index_;
  bool is_completed_ = false;

  ConePlannerPtr cone_planner_{nullptr};
};
}  // namespace cone_planner

#endif  // CONE_PLANNER__CONE_PLANNER_NODE_HPP_
