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
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

namespace
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using cone_planner::PlannerWaypoint;
using cone_planner::PlannerWaypoints;

PoseArray trajectory2PoseArray(const Trajectory& trajectory)
{
  PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto& point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

Pose transform_pose(const Pose& pose, const TransformStamped& transform)
{
  PoseStamped transformed_pose;
  PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);

  return transformed_pose.pose;
}

Trajectory create_trajectory(
  const PoseStamped& current_pose,
  const PlannerWaypoints& planner_waypoints,
  double velocity)
{
  Trajectory trajectory;
  trajectory.header = planner_waypoints.header;

  for (const auto& awp : planner_waypoints.waypoints) {
    TrajectoryPoint point;

    point.pose = awp.pose.pose;

    point.pose.position.z = current_pose.pose.position.z;  // height = const
    point.longitudinal_velocity_mps = velocity / 3.6;      // velocity = const

    // switch sign by forward/backward
    point.longitudinal_velocity_mps = (awp.is_back ? -1 : 1) * point.longitudinal_velocity_mps;

    trajectory.points.push_back(point);
  }

  return trajectory;
}

std::vector<size_t> get_reversing_indices(const Trajectory& trajectory)
{
  std::vector<size_t> indices;

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    if (
      trajectory.points.at(i).longitudinal_velocity_mps *
        trajectory.points.at(i + 1).longitudinal_velocity_mps < 0) {
      indices.push_back(i);
    }
  }

  return indices;
}

size_t get_next_target_index(
  const size_t trajectory_size,
  const std::vector<size_t>& reversing_indices,
  const size_t current_target_index)
{
  if (!reversing_indices.empty()) {
    for (const auto reversing_index : reversing_indices) {
      if (reversing_index > current_target_index) {
        return reversing_index;
      }
    }
  }

  return trajectory_size - 1;
}

Trajectory get_partial_trajectory(
  const Trajectory& trajectory,
  const size_t start_index,
  const size_t end_index)
{
  Trajectory partial_trajectory;
  partial_trajectory.header = trajectory.header;
  partial_trajectory.header.stamp = rclcpp::Clock().now();

  partial_trajectory.points.reserve(trajectory.points.size());
  for (size_t i = start_index; i <= end_index; ++i) {
    partial_trajectory.points.push_back(trajectory.points.at(i));
  }

  // Modify velocity at start/end point
  if (partial_trajectory.points.size() >= 2) {
    partial_trajectory.points.front().longitudinal_velocity_mps =
      partial_trajectory.points.at(1).longitudinal_velocity_mps;
  }
  if (!partial_trajectory.points.empty()) {
    partial_trajectory.points.back().longitudinal_velocity_mps = 0;
  }

  return partial_trajectory;
}

Trajectory create_stop_trajectory(const PoseStamped& current_pose)
{
  PlannerWaypoints waypoints;
  PlannerWaypoint waypoint;

  waypoints.header.stamp = rclcpp::Clock().now();
  waypoints.header.frame_id = current_pose.header.frame_id;
  waypoint.pose.header = waypoints.header;
  waypoint.pose.pose = current_pose.pose;
  waypoint.is_back = false;
  waypoints.waypoints.push_back(waypoint);
  waypoints.waypoints.push_back(waypoint);

  return create_trajectory(current_pose, waypoints, 0.0);
}

Trajectory create_stop_trajectory(const Trajectory& trajectory)
{
  Trajectory stop_trajectory = trajectory;
  for (size_t i = 0; i < trajectory.points.size(); ++i) {
    stop_trajectory.points.at(i).longitudinal_velocity_mps = 0.0;
  }
  return stop_trajectory;
}

bool is_stopped(
  const std::deque<Odometry::SharedPtr>& odom_buffer,
  const double th_stopped_velocity_mps)
{
  for (const auto& odom : odom_buffer) {
    if (std::abs(odom->twist.twist.linear.x) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}

double calc_distance_2d(const Trajectory& trajectory, const Pose& pose)
{
  const auto idx = motion_utils::findNearestIndex(trajectory.points, pose.position);
  return tier4_autoware_utils::calcDistance2d(trajectory.points.at(idx), pose);
}

} // namespace

namespace cone_planner
{

ConePlannerNode::ConePlannerNode(const rclcpp::NodeOptions & options)
: Node("cone_planner", options)
{
  using std::placeholders::_1;

  planned_trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", rclcpp::QoS{1});

  trajectory_sub_ = create_subscription<Trajectory>(
    "~/input/trajectory", rclcpp::QoS{1},
    [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  auto occupancy_grid_qos = rclcpp::QoS{1};
  occupancy_grid_qos.best_effort();
  occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid", occupancy_grid_qos,
    [this](const OccupancyGrid::SharedPtr msg) { occupancy_grid_ = msg; });

  pose_sub_ = create_subscription<PoseStamped>(
    "~/input/pose", rclcpp::QoS{1}, [this](const PoseStamped::SharedPtr msg) { pose_ = msg; });

  odom_sub_ = create_subscription<Odometry>(
    "~/input/odom", rclcpp::QoS{100}, std::bind(&ConePlannerNode::on_odometry, this, _1));

  waypoints_velocity_ = declare_parameter<double>("waypoints_velocity");
  th_arrived_distance_m_ = declare_parameter<double>("th_arrived_distance_m");
  th_stopped_time_sec_ = declare_parameter<double>("th_stopped_time_sec");
  th_stopped_velocity_mps_ = declare_parameter<double>("th_stopped_velocity_mps");
  th_course_out_distance_m_ = declare_parameter<double>("th_course_out_distance_m");
  replan_when_obstacle_found_ = declare_parameter<bool>("replan_when_obstacle_found");
  replan_when_course_out_ = declare_parameter<bool>("replan_when_course_out");

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

  {
    const auto period_ns = rclcpp::Rate(declare_parameter<double>("update_rate")).period();
    timer_ = rclcpp::create_timer(this,
                                  get_clock(),
                                  period_ns,
                                  std::bind(&ConePlannerNode::onTimer, this));
  }

  RCLCPP_INFO_ONCE(get_logger(), "Initialized node\n");
}

void ConePlannerNode::on_odometry(const Odometry::SharedPtr msg) {
  odom_ = msg;

  odom_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);

    if (time_diff.seconds() < th_stopped_time_sec_) {
      break;
    }

    odom_buffer_.pop_front();
  }
}

ConePlannerParam ConePlannerNode::get_planner_param() {
  ConePlannerParam param{};

  param.time_limit = declare_parameter<double>("time_limit");
  param.minimum_turning_radius = declare_parameter<double>("minimum_turning_radius");

  param.theta_size = declare_parameter<int>("theta_size");

  param.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  param.rrt_enable_update = declare_parameter<bool>("rrt_enable_update");
  param.rrt_max_planning_time = declare_parameter<double>("rrt_max_planning_time");
  param.rrt_margin = declare_parameter<double>("rrt_margin");
  param.rrt_neighbor_radius = declare_parameter<double>("rrt_neighbor_radius");

  return param;
}

void ConePlannerNode::update_target_index()
{
  const auto is_near_target =
    tier4_autoware_utils::calcDistance2d(planned_trajectory_.points.at(target_index_), *pose_) < th_arrived_distance_m_;

  const auto stopped = is_stopped(odom_buffer_, th_stopped_velocity_mps_);

  if (is_near_target && stopped) {
    const auto new_target_index =
      get_next_target_index(planned_trajectory_.points.size(), reversing_indices_, target_index_);

    if (new_target_index == target_index_) {
      // Finished publishing all partial trajectories
      is_completed_ = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Freespace planning completed");
      // std_msgs::msg::Bool is_completed_msg;
      // is_completed_msg.data = is_completed_;

      // parking_state_pub_->publish(is_completed_msg);
    } else {
      // Switch to next partial trajectory
      prev_target_index_ = target_index_;
      target_index_ =
        get_next_target_index(planned_trajectory_.points.size(), reversing_indices_, target_index_);
    }
  }
}

bool ConePlannerNode::is_plan_required()
{
  if (planned_trajectory_.points.empty()) {
    return true;
  }

  const auto dist_to_goal = tier4_autoware_utils::calcDistance2d(planned_trajectory_.points.back(), *pose_);
  if (dist_to_goal < 3.0) {
    return true;
  }

  if (replan_when_obstacle_found_) {
    cone_planner_->set_map(*occupancy_grid_);

    const size_t nearest_index_partial =
      motion_utils::findNearestIndex(partial_planned_trajectory_.points, pose_->pose.position);
    const size_t end_index_partial = partial_planned_trajectory_.points.size() - 1;

    const auto forward_trajectory = get_partial_trajectory(partial_planned_trajectory_,
                                                           nearest_index_partial,
                                                           end_index_partial);
    const bool is_obstacle_found = cone_planner_->has_obstacle_on_trajectory(trajectory2PoseArray(forward_trajectory));
    if (is_obstacle_found) {
      RCLCPP_INFO(get_logger(), "Found obstacle");
      return true;
    }
  }

  if (replan_when_course_out_) {
    const bool is_course_out = calc_distance_2d(planned_trajectory_, pose_->pose) > th_course_out_distance_m_;
    if (is_course_out) {
      RCLCPP_INFO(get_logger(), "Course out");
      return true;
    }
  }

  return false;
}

void ConePlannerNode::onTimer()
{
  if (!trajectory_ || !occupancy_grid_ || !pose_ || !odom_) {
    RCLCPP_INFO(get_logger(), "Data not ready\n");
    return;
  }

  // if (is_completed_) {
  //   return;
  // }

  // TODO: Get from Odometry
  if (pose_->header.frame_id == "") {
    return;
  }

  if (is_plan_required()) {
    const auto stop_trajectory = partial_planned_trajectory_.points.empty()
                                   ? create_stop_trajectory(*pose_)
                                   : create_stop_trajectory(partial_planned_trajectory_);
    planned_trajectory_pub_->publish(stop_trajectory);
    RCLCPP_INFO(get_logger(), "Started replanning");

    reset();
    const auto goal_pose = get_closest_pose();
    planTrajectory(goal_pose);
  }

  // Stop
  if (planned_trajectory_.points.size() <= 1) {
    return;
  }

  // Update partial trajectory
  update_target_index();
  partial_planned_trajectory_ = get_partial_trajectory(planned_trajectory_, prev_target_index_, target_index_);

  planned_trajectory_pub_->publish(partial_planned_trajectory_);
}

PoseStamped ConePlannerNode::get_closest_pose()
{
  const auto closest_idx =
    (motion_utils::findNearestIndex(trajectory_->points, pose_->pose.position) + 25) % trajectory_->points.size();
  const auto closest_pose = trajectory_->points.at(closest_idx).pose;

  PoseStamped closest_pose_stamped{};
  closest_pose_stamped.header = trajectory_->header;
  closest_pose_stamped.pose = closest_pose;

  return closest_pose_stamped;
}

void ConePlannerNode::reset()
{
  planned_trajectory_ = Trajectory();
  partial_planned_trajectory_ = Trajectory();
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

  // execute planning
  const rclcpp::Time start = get_clock()->now();
  const bool result = cone_planner_->make_plan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
  const rclcpp::Time end = get_clock()->now();

  RCLCPP_INFO(get_logger(), "Freespace planning: %f [s]", (end - start).seconds());

  if (result) {
    RCLCPP_INFO(get_logger(), "Found goal!");
    planned_trajectory_ = create_trajectory(*pose_,
                                            cone_planner_->get_waypoints(),
                                            waypoints_velocity_);
    reversing_indices_ = get_reversing_indices(planned_trajectory_);
    prev_target_index_ = 0;
    target_index_ = get_next_target_index(planned_trajectory_.points.size(),
                                          reversing_indices_,
                                          prev_target_index_);
  } else {
    RCLCPP_INFO(get_logger(), "Can't find goal...");
    reset();
  }
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
