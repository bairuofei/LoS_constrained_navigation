## Quick Start

### Dependencies
- Ubuntu 20.04
- ROS Noetic

### Demo for Visible Region & Frontiers
Terminal 1
```bash
## git clone navigation stack
git clone git@github.com:bairuofei/realm_navigation_stack.git
git submodule update --init --recursive
cd realm_navigation_stack/
git switch ras_demo
catkin build
source devel/setup.bash

# launch simulation environment
roslaunch vehicle_simulator two_robot.launch

# launch mapping module for better visualization (mandatory for exploration tasks)
roslaunch multi_slam_realm multi_connect_karto.launch
```
Terminal 2
```bash
## git clone LoS-connectivity maintenance module
git clone git@github.com:bairuofei/LoS_constrained_navigation.git
cd LoS_constrained_navigation/catkin_ws_connectivity
git switch demo
catkin build
source devel/setup.bash

# launch exploration task and connectivity controller
roslaunch connectivity_exploration two_robot.launch
```

Terminal 3
```bash
# manually publish /robot_1/cmd_vel to control robot_1
# 1. Add Pulgins/Rbot_Tools/Robot_Steering to load a control panel
# 2. Specify the published topic as "/robot1/cmd_vel"
# 3. Use the panel to control
rqt
```


## Configurations

#### (1) Specify simulation environment
> Specified in the launch file of vehicle_simulator package
- map name (also required by connectivity_controller)
- robot number (also required by mapper, connectivity_controller)

#### (2) Set task specification
> Specified in the yaml file
- For exploration, set robots' initial positions 
- For navigation, set both robots' and targets' positions

## Tasks
#### (1) Multi-Robot Navigation under LoS contraints
> This task requires pre-defined initial and target positions for robots. You should specify these information in `catkin_ws_connectivity/src/connectivity_exploration/param/tasks/env_{environment_name}{suffix}.yaml`. The `environment_name` is set when launching the gazebo simulator; and the `suffix` is set in `connectivity_exploration/launch/run_four_navigation.launch` to allow various settintgs under the same environment.

>We extensively use the environment `wide_grid`, because it represents the most challenging environment with cluttered obstacles that frequently block line-of-sight between robots.

```bash
## Under ros project: realm_navigation_stack
# Launch gazebo simulator & far_planner
# cd /home/ruofei/code/cpp/autonomous_exploration_development_environment
roslaunch vehicle_simulator four_robot.launch

## Under ros project: catkin_ws_connectivity
# Launch laser scan filter & connectivity controller
roslaunch connectivity_exploration run_four_navigation.launch

roslaunch connectivity_exploration publish_stop.launch
```


#### (2) Multi-Robot Exploration under LoS constraints

> This task requires an enclosed environment. Available environments include `{larger_garage, map4, map7, simple_forest}`. This should be specified in `four_robot.launch`.

> You can quickly check different enviroments only by directly launching `roslaunch p2os_urdf test_gazebo.launch`.

```bash
## Under ros project: realm_navigation_stack
# Launch gazebo simulator & far_planner
# cd /home/ruofei/code/cpp/autonomous_exploration_development_environment
roslaunch vehicle_simulator four_robot.launch

# Create occupancy map & outputs frontier points
roslaunch multi_slam_realm multi_connect_karto.launch

## Under ros project: catkin_ws_connectivity
# Launch laser scan filter & connectivity controller
roslaunch connectivity_exploration run_four_exploration.launch

roslaunch connectivity_exploration publish_stop.launch
```


## Acknowledgements
- [Autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)
- [FAR Planner](https://github.com/MichaelFYang/far_planner)
- [Multi_slam_karto](https://github.com/SunZezhou/multi_slam_karto)
- [Active_graph_slam](https://github.com/JulioPlaced/active_graph_slam)