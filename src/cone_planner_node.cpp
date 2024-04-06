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

#include "cone_planner/cone_planner_node.hpp"

#include <motion_utils/trajectory/trajectory.hpp>

namespace cone_planner
{

ConePlannerNode::ConePlannerNode(const rclcpp::NodeOptions & options)
: Node("cone_planner", options)
{
  planned_trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", rclcpp::QoS{1});

  trajectory_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid", rclcpp::QoS{1},
    [this](const OccupancyGrid::SharedPtr msg) { occupancy_grid_ = msg; });

  pose_sub_ = create_subscription<PoseStamped>(
    "~/input/pose", rclcpp::QoS{1}, [this](const PoseStamped::SharedPtr msg) { pose_ = msg; });

  cone_planner_ = std::make_unique<ConePlanner>();

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&ConePlannerNode::onTimer, this));

  RCLCPP_INFO_ONCE(get_logger(), "Initialized node\n");
}

void ConePlannerNode::onTimer()
{
  if (!trajectory_ || !occupancy_grid_ || !pose_) {
    RCLCPP_INFO(get_logger(), "Data not ready\n");
    return;
  }

  reset();

  [[maybe_unused]]const auto goal_point = get_closest_point();
  planTrajectory();

  planned_trajectory_pub_->publish(planned_trajectory_);
}

void ConePlannerNode::reset()
{
  planned_trajectory_ = Trajectory();
  is_completed_ = false;
}

void ConePlannerNode::planTrajectory()
{
  if (!occupancy_grid_ || !pose_) {
    return;
  }
}

TrajectoryPoint ConePlannerNode::get_closest_point()
{
  const auto closest_idx =
    motion_utils::findNearestIndex(trajectory_->points, pose_->pose.position);
  return trajectory_->points.at(closest_idx);
}

}  // namespace cone_planner

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cone_planner::ConePlannerNode)
