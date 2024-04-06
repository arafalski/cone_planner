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

#include <geometry_msgs/msg/pose.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

namespace
{
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;

Pose transform_pose(const Pose& pose, const TransformStamped& transform)
{
  PoseStamped transformed_pose;
  PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

} // namespace

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

    const auto vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");
    VehicleShape extended_vehicle_shape = vehicle_shape_;
    extended_vehicle_shape.length += vehicle_shape_margin_m;
    extended_vehicle_shape.width += vehicle_shape_margin_m;
    extended_vehicle_shape.base2back += vehicle_shape_margin_m / 2;

    cone_planner_ = std::make_unique<ConePlanner>(conePlannerParam, extended_vehicle_shape);
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

  const auto goal_pose = get_closest_pose();

  reset();
  planTrajectory(goal_pose);

  planned_trajectory_pub_->publish(planned_trajectory_);
}

PoseStamped ConePlannerNode::get_closest_pose()
{
  const auto closest_idx =
    motion_utils::findNearestIndex(trajectory_->points, pose_->pose.position);
  const auto closest_pose = trajectory_->points.at(closest_idx).pose;

  PoseStamped closest_pose_stamped{};
  closest_pose_stamped.header = trajectory_->header;
  closest_pose_stamped.pose = closest_pose;

  return closest_pose_stamped;
}

void ConePlannerNode::reset()
{
  planned_trajectory_ = Trajectory();
  is_completed_ = false;
}

void ConePlannerNode::planTrajectory(const PoseStamped& goal_pose)
{
  if (!occupancy_grid_ || !pose_) {
    return;
  }

  cone_planner_->set_map(*occupancy_grid_);

  const auto current_pose_in_costmap_frame = transform_pose(
    pose_->pose,
    get_transform(occupancy_grid_->header.frame_id,
                  pose_->header.frame_id));
  const auto goal_pose_in_costmap_frame = transform_pose(
    goal_pose.pose,
    get_transform(occupancy_grid_->header.frame_id,
                  goal_pose.header.frame_id));
}

TransformStamped ConePlannerNode::get_transform(
  const std::string& from,
  const std::string& to)
{
  TransformStamped tf;
  try {
    tf =
      tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}

}  // namespace cone_planner

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cone_planner::ConePlannerNode)
