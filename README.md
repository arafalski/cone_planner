# cone_planner
<!-- Required -->
<!-- Package description -->

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=On --packages-up-to cone_planner
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch cone_planner cone_planner.launch.py vehicle_param_file:=PATH_TO_VEHICLE_PARM_FILE
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `~/input/trajectory` | autoware_auto_planning_msgs::msg::Trajectory | Reference trajectory |
| `~/input/occupancy_grid` | nav_msgs::msg::OccupancyGrid | Occupancy grid map |
| `~/input/pose` | geometry_msgs::msg::PoseStamped | Pose of the car |
| `~/input/odom` | nav_msgs::msg::Odometry | Odometry used for checking stop condition |

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `~/output/trajectory` | autoware_auto_planning_msgs::msg::Trajectory | Planned trajectory |

### Parameters

| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| `update_rate` | float  | Update rate of checking if replanning is needed |
| `waypoints_velocity` | float  | Reference velocity at planned trajectory points |
| `vehicle_shape_margin_m` | float  | Car perimeter margin used for planning |
| `th_arrived_distance_m` | float  | Distance between car and goal point to consider that goal has been achieved |
| `th_stopped_time_sec` | float  | Time after stopping to consider that the car stopped successfully |
| `th_stopped_velocity_mps` | float  | Velocity threshold to consider that the car stopped |
| `th_course_out_distance_m` | float  | Distance between car and planned trajectory to consider above which replanning is triggered |
| `c_space_margin_m` | float  | C space margin |
| `replan_when_obstacle_found` | bool  | True if replanning should be triggered when new obstacle was found |
| `replan_when_course_out` | bool  | True if replanning should be triggered when the car is out of the planned trajectory |
| `time_limit` | float  | Max until finding path |
| `minimum_turning_radius` | float  | Min car turning radius |
| `theta_size` | int  | Discretization resolution of the heading angle range |
| `obstacle_threshold` | int  | Cost value threshold to be considered an obstacle |
| `rrt_enable_update` | bool  | True if planning should be continued after finding path to the goal pose |
| `rrt_max_planning_time` | float  | Max time of planning |
| `rrt_margin` | float | Additional car perimeter margin used in planner |
| `rrt_neighbor_radius` | float | Radius in which checking if shorter path exist through other neighbor is performed |
