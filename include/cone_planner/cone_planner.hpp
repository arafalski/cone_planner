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

#ifndef CONE_PLANNER__CONE_PLANNER_HPP_
#define CONE_PLANNER__CONE_PLANNER_HPP_

#include "cone_planner/visibility_control.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>
#include <tf2/utils.h>

#include "freespace_planning_algorithms/rrtstar_core.hpp"

#include <vector>

namespace cone_planner
{
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::OccupancyGrid;

struct IndexXY
{
  int x;
  int y;
};

struct IndexXYT
{
  int x;
  int y;
  int theta;
};

struct VehicleShape
{
  double length;     // X [m]
  double width;      // Y [m]
  double base2back;  // base_link to rear [m]

  VehicleShape() = default;

  VehicleShape(double length, double width, double base2back)
    : length(length), width(width), base2back(base2back) {}

  explicit VehicleShape(
    const vehicle_info_util::VehicleInfo& vehicle_info,
    const double margin = 0.0)
    : length(vehicle_info.vehicle_length_m + margin),
    width(vehicle_info.vehicle_width_m + margin),
    base2back(vehicle_info.rear_overhang_m + margin / 2.0) {}
};

struct ConePlannerParam
{
  // base configs
  double time_limit;  // planning time limit [msec]

  // robot configs
  double minimum_turning_radius;  // [m]

  // search configs
  int theta_size; // discretized angle table size [-]

  //costmap configs
  int obstacle_threshold; // obstacle threshold on grid [-]

  // planner configs
  double rrt_margin; // [m]
  double rrt_neighbor_radius; // neighbor radius [m]
  bool rrt_enable_update; // update solution even after feasible solution found with given time budget
  double rrt_max_planning_time; // if enable_update is true, update is done before time elapsed [msec]
};

struct PlannerWaypoint
{
  geometry_msgs::msg::PoseStamped pose;
  bool is_back = false;
};

struct PlannerWaypoints
{
  std_msgs::msg::Header header;
  std::vector<PlannerWaypoint> waypoints;

  double compute_length() const;
};

class CONE_PLANNER_PUBLIC ConePlanner
{
public:
  ConePlanner(const ConePlannerParam& planner_param,
              const VehicleShape& original_vehicle_shape)
    : planner_param_{planner_param},
      original_vehicle_shape_{original_vehicle_shape},
      collision_vehicle_shape_{original_vehicle_shape.length + 2 * planner_param.rrt_margin,
                               original_vehicle_shape.width + 2 * planner_param.rrt_margin,
                               original_vehicle_shape.base2back + planner_param.rrt_margin} {}

  void set_map(const OccupancyGrid& costmap);
  bool make_plan(const Pose& start_pose, const Pose& goal_pose, double c_space_margin);
  const PlannerWaypoints& get_waypoints() const { return waypoints_; }
  bool has_obstacle_on_trajectory(const PoseArray& trajectory) const;

private:
  void compute_collision_indexes(
    int theta_index,
    std::vector<IndexXY> & indexes_2d,
    std::vector<IndexXY> & vertex_indexes_2d
  ) const;
  bool detect_collision(const IndexXYT& base_index) const;
  void setRRTPath(const std::vector<rrtstar_core::Pose>& waypoints);
  inline bool is_out_of_range(const IndexXYT& index) const
  {
    if (index.x < 0 || static_cast<int>(costmap_.info.width) <= index.x) {
      return true;
    }
    if (index.y < 0 || static_cast<int>(costmap_.info.height) <= index.y) {
      return true;
    }
    return false;
  }
  inline bool is_obs(const IndexXYT& index) const
  {
    // NOTE: Accessing by .at() instead makes 1.2 times slower here.
    // Also, boundary check is already done in isOutOfRange before calling this function.
    // So, basically .at() is not necessary.
    return is_obstacle_table_[index.y][index.x];
  }

  ConePlannerParam planner_param_{};
  const VehicleShape original_vehicle_shape_{};
  const VehicleShape collision_vehicle_shape_{};

  OccupancyGrid costmap_{};
  std::vector<std::vector<IndexXY>> coll_indexes_table_{};
  std::vector<std::vector<IndexXY>> vertex_indexes_table_{};
  std::vector<std::vector<bool>> is_obstacle_table_{};

  Pose start_pose_;
  Pose goal_pose_;

  bool is_collision_table_initialized_{};

  PlannerWaypoints waypoints_;
};

}  // namespace cone_planner

#endif  // CONE_PLANNER__CONE_PLANNER_HPP_
