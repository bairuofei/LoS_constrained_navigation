<div align ="center">

<!-- <img src="./assets/logo.png" width="20%"> -->
<h3> ICRA 2025: Realm: Real-time Line-of-Sight Maintenance in Multi-Robot Navigation with Unknown Obstacles </h3>

Ruofei Bai<sup>1,2</sup>, Shenghai Yuan<sup>1</sup>, Kun Li<sup>3</sup>, Hongliang Guo<sup>4</sup>, Wei-Yun Yau<sup>2</sup>, Lihua Xie<sup>1</sup>

<sup>1</sup> Nanyang Technological University,
<sup>2</sup> Institute for Infocomm Research (I2R), Agency for Science, Technology and Research (A*STAR)
<sup>3</sup> School of Automation, Chongqing University
<sup>4</sup> College of Computer Science, Sichuan University



<!-- <a href="https://ieeexplore.ieee.org/abstract/document/10802691"><img alt="Paper" src="https://img.shields.io/badge/Paper-IEEE%20Xplore-pink"/></a> -->
<a href="https://arxiv.org/abs/2502.15162"><img alt="Paper" src="https://img.shields.io/badge/Paper-arXiv-8A2BE2"/></a>
<!-- <a href='https://drive.google.com/drive/folders/1UmZ3vA1cOunB-2wgz8T1fJDebhb-gmax?usp=sharing'><img src='https://img.shields.io/badge/Dataset-UMAD-green' alt='Code&Datasets'></a>
<a href="https://www.youtube.com/watch?v=xORb4H-AyNw"><img alt="Video" src="https://img.shields.io/badge/Video-Youtube-red"/></a>
<a href="https://github.com/IMRL/UMAD/blob/main/Doc/UMAD-Poster.pdf"><img alt="Poster" src="https://img.shields.io/badge/Poster-blue"/></a> -->

</div>


## News
- [2025/09/02] We have open-sourced a more advanced version extened from the ICRA paper, which is currently under review. It supports several new features:
    - Flexible topology optimization for improved navigation efficiency;
    - Reliable line-of-sight distance evaluation compared with previous metrics;
    - Diverse environments for testing and reproducing the results in our paper;
    - Convenient task specification, result recording, and comparison.

- Our paper has been selected as a <span style="color:red">**Best Paper Award Finalist of ICRA 2025**</span>!


## Demo Video

Short video to intorduce our work:

[![](assets/video_cover.png)](https://www.bilibili.com/video/BV1UB7fzVEeu/?spm_id_from=333.337.search-card.all.click&vd_source=2d11232d984feb225a544f200a5b226e)





Please wait for the simulation gif to load...

![Four-robot navigation](assets/demo.gif)


## Quick Start

### Dependencies
- Ubuntu 20.04
- ROS Noetic

### Launch Simulations
Terminal 1
```bash
## git clone navigation stack
git clone git@github.com:bairuofei/realm_navigation_stack.git
git submodule update --init --recursive
cd realm_navigation_stack/
catkin build
source devel/setup.bash

# launch simulation environment
roslaunch vehicle_simulator four_robot.launch

# launch mapping module for better visualization (mandatory for exploration tasks)
roslaunch multi_slam_realm multi_connect_karto.launch
```
Terminal 2
```bash
## git clone LoS-connectivity maintenance module
git clone git@github.com:bairuofei/LoS_constrained_navigation.git
cd LoS_constrained_navigation/catkin_ws_connectivity
catkin build
source devel/setup.bash

# launch exploration task and connectivity controller
roslaunch connectivity_exploration run_four_exploration.launch

# start execution
roslaunch connectivity_exploration publish_stop.launch
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
- [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment)
- [FAR Planner](https://github.com/MichaelFYang/far_planner)
- [multi_slam_karto](https://github.com/SunZezhou/multi_slam_karto)
- [active_graph_slam](https://github.com/JulioPlaced/active_graph_slam)