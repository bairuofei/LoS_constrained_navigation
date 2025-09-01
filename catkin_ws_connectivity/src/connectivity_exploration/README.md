## Quick Start

### Branches
- `experiment` 测试MST与没有MST的版本。graph Laplacian没有用实际masked掉的吗，可能出现某些边无法用lambda2约束
- `exp_mst_enhance` 与`experiment`版本相同，但在使用MST时，使用了masked graph Laplacian
- `exp_fix_topology` 采用fixed topology, 其余与`exp_mst_enhance`相同
- `main` ICRA-2025投稿版本
- `dwa` Use dwa as path planner, rather than Far planner. Only appliable to almost free space navigation.
- `nonlinear_opt` Use nonlinear optimization to solve the exact LoS-distance using Gurobi


### 1. Initial configuration

#### (1) Simulation environment
> Specified in the launch file of vehicle_simulation package
- map name (also required by connectivity_controller)
- robot number (also required by mapper, connectivity_controller)

#### (2) Task specification
> Specified in the yaml file
- robots and targets initial positions
    - For exploration, only robot positions are required
    - For navigation, both robot and target positions are required


### 2. Navigation
#### (1) Four robot navigation
```bash
## Under ros project: autonomous_exploration_development_environment
# Run gazebo simulation, and far_planner
# cd /home/ruofei/code/cpp/autonomous_exploration_development_environment
roslaunch vehicle_simulator four_robot.launch

# Create occupancy map, and outputs frontier points
roslaunch multi_slam_realm multi_connect_karto.launch

## Under ros project: catkin_ws_connectivity
# Run laser scan filter, and connectivity controller
roslaunch connectivity_exploration run_four_exploration.launch

roslaunch connectivity_exploration publish_stop.launch
```

#### (2) Six robot navigatioon
```bash
roslaunch vehicle_simulator six_navigation.launch
roslaunch multi_slam_karto multi_connect_karto.launch
roslaunch connectivity_exploration run_six_navigation.launch
```

#### (3) IOT experiment with three robots
```bash
roslaunch vehicle_simulator three_navigation.launch # using iot4 environment
roslaunch multi_slam_karto multi_connect_karto.launch
roslaunch connectivity_exploration run_three_navigation.launch
```

### 3. Exploration 
#### (1) Four robot exploration
```bash
## Under ros project: autonomous_exploration_development_environment
# Run gazebo simulation, and far_planner
# cd /home/ruofei/code/cpp/autonomous_exploration_development_environment
roslaunch vehicle_simulator four_robot.launch
# Create occupancy map, and outputs frontier points
roslaunch multi_slam_realm multi_connect_karto.launch

## Under ros project: catkin_ws_connectivity
# Run laser scan filter, and connectivity controller
roslaunch connectivity_exploration run_four_exploration.launch

roslaunch connectivity_exploration publish_stop.launch
```

### 4. Visibility-aware planning
```
1. Use "experiment" branch for connectivity_exploration;
2. Set visible_aware_planning in autonomous_navigation_develop_env; use map "test_prior_planning"
3. roslaunch vehicle_simulator run_two.launch
```

### 5. Use dwa planner, rather than FAR planner
```
1. Use "dwa" branch for connectivity_exploration; roslaunch connectivity_exploration run_three_exploration_forest.launch
2. cd "~/code/cpp/catkin_ws_dwa", roslaunch dwa_planner multi_dwa.launch
3. cd "~/code/cpp/autonomous_exploration_development_environment", roslaunch vehicle_simulator run_three_dwa.launch
4. cd "~/code/cpp/autonomous_exploration_development_environment", roslaunch multi_slam_realm multi_connect_karto.launch
```


### 6. Visualize 3D visible region
```
1. cd "~/code/dataset/kitti", rosbag play kitti_00.bag
2. cd ~/code/cpp/catkin_ws_connectivity; roslaunch connectivity_exploration visualize_3D_polyhedron.launch 

```


## Important Notes

- You need to publish "/robot_number" as global paramter when running vehicle simulator, i.e., `roslaunch vehicle_simulator four_robot.launch`.
The multi-robot SLAM module, connectivity controller will then query this parameter in ros master.


## Progress
1. 目前los-distance使用限定phi范围时的los-distance lower bound;
2. gradient方向使用安全角度为laser分辨率角度(one degree in our case)的virtual flipping; 另外，如果两个点都是背景点，则gradient设置为径向。

注意上面第二点可以拓展到更大安全角度的virtual flipping.

给定convexhull, 如何建立virtual flipping和los-distance computation?

1. 识别hull verties in visible cone.
2. check distance of these vertices to center. 距离比当前机器人更大的（表示在原空间更近），则标记为potential obs point，并计算virtual flipping radius; 反之则标记为背景点，记录下标，准备在第4步进行virtual flipping. 同时，记录所有落在visible cone中的点的下标。
3. 保留原始convexhull中，与visible cone重叠的face （这通过检查每个face的顶点是否又在visible cone中的），以使用非线性优化方法求解los-distance。 注意，los-distance依然使用原始的convexhull,所以不依赖于virtual flipping.
4. 进行virtual flipping之后，重新计算convexhull, 以求解梯度。



## Problems
1. Loop循环超出时间，导致connect_pairs没有被记录。同时，这可能导致lambda2 = 0,因为机器人最终没有更新自身的位置。

2. 机器人的位置跳变比较严重，尤其是即将失去LoS，或者接近collision的threshold的时候。



## Results Record


```txt
# Step = 0.05
2024_11_21_16_57  使用los-distance lb
2024_11_21_19_49  使用los-distance lb, 失败


# Step = 0.08
2024_11_21_18_37  使用dk*cos_theta
2024_11_21_18_50  使用los-distance lb

# Step = 0.05

2024_09_01_16_52 使用d_los = d_k*cos_theta_k, 包含了对map4的完整探索。
2024_09_04_19_57 使用d_los = d_k*cos_theta_k, 包含了对map7的完整探索。
2024_09_05_19_33 使用d_los = d_k*cos_theta_k, 出现了lambda_2 = 0的情况！
2024_09_05_20_57  使用d_los = d_k*cos_theta_k, 没出现lambda_2 = 0的情况
2024_09_08_14_58  使用d_los = d_k*cos_theta_k, 没出现lambda_2 = 0的情况
2024_09_08_16_40  使用d_los = d_k*cos_theta_k, 没出现lambda_2 = 0的情况


2024_09_05_15_15 使用d_los = d_k，包含了对map7的完整探索。与前面两个参数完全相同，但是会出现lambda_2 = 0的情况！
2024_09_05_15_30 使用d_los = d_k，包含了对map4的完整探索. 参数相同，没有出现lambda_2 = 0的情况。
2024_09_05_16_23 使用d_los = d_k，包含了对map7的完整探索. 参数相同，没有出现lambda_2 = 0的情况。
2024_09_05_19_04 使用d_los = d_k，包含了对map7的完整探索. 参数相同，出现了lambda_2 = 0的情况！
2024_09_09_11_15  使用d_los = d_k, 没出现lambda_2 = 0的情况


总结：上面是否出现lambda=2，只是衡量改变d_los定义的效果的一种方式。另一种方式是，查看d_los的数值分布


# 增大step = 0.07 (原来0.05)
2024_09_06_13_33  使用d_los = d_k，探索map7, 中途断连，未完成探索
2024_09_06_15_40  使用d_los = d_k，探索map7, 有断连情况

2024_09_06_14_44  使用d_los = d_k*cos_theta_k， 探索map7, 良好
2024_09_06_15_47  使用d_los = d_k*cos_theta_k， 探索map7, 断连


# 2024-09-09之前，只记录机器人0-1之间的LoS-distance
example_los_distance: (d_los_ji, d_los_ij, )
example_cos_theta_k: (d_k*, cos_theta*)

# 2024-09-09之后，记录所有机器人之间的LoS-distance
example_los_distance[key]： (d_los_ji, d_los_ij, softmin_d_los)
example_cos_theta_k[key]： (dji_k*, cos_theta_ji*, dij_k*, cos_theta_ij*)  如果没有统计，则这一项始终为-1， -1


## ICRA-2025 论文实验章节的图是2024_09_01_16_52 对应的数据绘制的。

```

## Code Structure
### Controller
- `controller.py`  只有robot_1接收nav_goal, 并施加exploration force,其它机器人被动移动保持connectivity. nav_goal直接由RVIZ发布，不需要运行FAR_Planner.
- `multi_controller.py` 所有的robot都接收由FAR_Planner发出的way_point(通过订阅自身对应的topic). Leading_robot选择为距离目标点最近的机器人，到达目标点的机器人需要stay一段时间. 
- `multi_exploration.py` 在`multi_controller.py`的基础上，订阅frontier points，并进行task assignment. Leading_robot的选择不考虑当前正在stay的机器人。


- `GeneralizedLaplacian.py`   LoS-distance仅使用d*的版本
- `GeneralizedLaplacianWithSensitivity.py`  LoS-distance使用d*cos_theta的版本. 其中，LoS-distance fusion提供了两种选项，（1）weighted sum (默认设置); (2) softmin函数

### Results visualization
- `visualize_results.py` 常用版本，只对比0-1的example losdiantace. 在09-09改动后，无法直接用来plot LoS distance 
- `visualize_results2.py` 可以用来在一次实验中，比较所有机器人的LoS distance的累计密度函数
- `compare_two_los_distance.py`  用来比较两次实验的LoS distance的累计密度函数


### Navigation with specified targets
far_planner接收`/robot_x/goal_point` topic 作为目标点。具体可见，controller如何把frontier作为target point发送给far_planner.

## [Optional] Publish positions to tello in real experiments
```py
# Define publisher to publish robots' positions to Tello
self.pub_tello_position = rospy.Publisher('/tello_positions', PointStamped, queue_size=10)

# Publish data through topic
def publish_tello_positions(self, robot_id: int, position: tuple):
    """ Note robot id start from 1"""
    point_msg = PointStamped()
    
    # 填充 header 数据
    point_msg.header = Header()
    point_msg.header.stamp = rospy.Time.now()  # 当前时间戳
    point_msg.header.frame_id = f"robot_{robot_id}"    # 坐标系 id
    
    # 填充 Point 数据
    point_msg.point.x = position[0]
    point_msg.point.y = position[1]
    point_msg.point.z = 3.0

    # 发布消息
    self.pub_tello_position.publish(point_msg)

# Call publisher
next_pose = (curr_pose[0] + delta_movement[r][0], curr_pose[1] + delta_movement[r][1])
self.publish_tello_positions(r+1, next_pose)
```