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
#include <vehicle_info_util/vehicle_info_util.hpp>

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

  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
  }

  {
    const auto conePlannerParam = get_planner_param();
    cone_planner_ = std::make_unique<ConePlanner>(conePlannerParam, vehicle_shape_);
  }

  using namespace std::literals::chrono_literals;
  timer_ =
    rclcpp::create_timer(this, get_clock(), 100ms, std::bind(&ConePlannerNode::onTimer, this));

  RCLCPP_INFO_ONCE(get_logger(), "Initialized node\n");
}

ConePlannerParam ConePlannerNode::get_planner_param() {
  ConePlannerParam param{};

  param.theta_size = declare_parameter<int>("theta_size");

  param.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  param.rrt_margin = declare_parameter<double>("rrt_margin");

  return param;
}

void ConePlannerNode::onTimer()
{
  if (!trajectory_ || !occupancy_grid_ || !pose_) {
    RCLCPP_INFO(get_logger(), "Data not ready\n");
    return;
  }

  if (is_completed_) {
    return;
  }

  if (pose_->header.frame_id == "") {
    return;
  }

  [[maybe_unused]]const auto goal_point = get_closest_point();

  reset();
  planTrajectory();

  planned_trajectory_pub_->publish(planned_trajectory_);
}

TrajectoryPoint ConePlannerNode::get_closest_point()
{
  const auto closest_idx =
    motion_utils::findNearestIndex(trajectory_->points, pose_->pose.position);
  return trajectory_->points.at(closest_idx);
}

void ConePlannerNode::reset()
{
  planned_trajectory_ = Trajectory();
  is_completed_ = false;
}

void ConePlannerNode::planTrajectory()
{
  if (!occupancy_grid_) {
    return;
  }

  cone_planner_->set_map(*occupancy_grid_);
}

}  // namespace cone_planner

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cone_planner::ConePlannerNode)
