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

namespace cone_planner
{

ConePlannerNode::ConePlannerNode(const rclcpp::NodeOptions & options)
:  Node("cone_planner", options)
{
  occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
    "~/input/occupancy_grid",
    rclcpp::QoS{1},
    std::bind(&ConePlannerNode::onOccupancyGrid, this, std::placeholders::_1)
  );

  cone_planner_ = std::make_unique<cone_planner::ConePlanner>();
  RCLCPP_INFO_ONCE(this->get_logger(), "Initialized node\n");
}

void ConePlannerNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  occupancy_grid_ = msg;
  RCLCPP_INFO_ONCE(this->get_logger(), "Received occupancy grid\n");
}

}  // namespace cone_planner

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(cone_planner::ConePlannerNode)
