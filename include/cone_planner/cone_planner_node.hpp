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

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "autoware_auto_planning_msgs/msg/trajectory.hpp"

#include "cone_planner/cone_planner.hpp"

namespace cone_planner
{
using ConePlannerPtr = std::unique_ptr<cone_planner::ConePlanner>;
using autoware_auto_planning_msgs::msg::Trajectory;
using nav_msgs::msg::OccupancyGrid;

class CONE_PLANNER_PUBLIC ConePlannerNode : public rclcpp::Node
{
public:
  explicit ConePlannerNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<Trajectory>::SharedPtr planned_trajectory_pub_;

  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();
  void reset();
  void planTrajectory();

  Trajectory::SharedPtr trajectory_;
  OccupancyGrid::SharedPtr occupancy_grid_;
  Trajectory planned_trajectory_;
  bool is_completed_ = false;

  ConePlannerPtr cone_planner_{nullptr};
};
}  // namespace cone_planner

#endif  // CONE_PLANNER__CONE_PLANNER_NODE_HPP_
