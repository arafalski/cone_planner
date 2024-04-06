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

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>

namespace cone_planner
{
using nav_msgs::msg::OccupancyGrid;

struct ConePlannerParam
{
  int obstacle_threshold;  // obstacle threshold on grid [-]
};

class CONE_PLANNER_PUBLIC ConePlanner
{
public:
  ConePlanner(const ConePlannerParam& planner_param)
    : planner_param_{planner_param} {}

  void set_map(const OccupancyGrid& costmap);

private:
  ConePlannerParam planner_param_{};

  OccupancyGrid costmap_{};
  std::vector<std::vector<bool>> is_obstacle_table_{};
};

}  // namespace cone_planner

#endif  // CONE_PLANNER__CONE_PLANNER_HPP_
