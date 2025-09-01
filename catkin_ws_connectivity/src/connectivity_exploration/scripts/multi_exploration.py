#!/usr/bin/env python3

import sys
import os
import rospkg
import rospy
import math
import numpy as np
import threading
import copy
from typing import List, Dict, Set, Tuple
import atexit
import time
import yaml
import gc  # debug
import faulthandler  # debug

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped, PointStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import tf.transformations as tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Bool
from std_msgs.msg import ColorRGBA

from connectivity_exploration.msg import PointArray

from GeneralizedLaplacianWithSensitivity import GeneralizedLaplacian
from Robot import Robot
from utils import weight_to_rgb, GREEN, RED, RESET, create_arrow_marker, create_circle_marker, my_print, Timer
from LeadingTimer import LeadingTimer
from Recorder import Recorder


class Controller:
    def __init__(self, num_robot: int, enable_record: bool, save_folder: str, enable_mst: bool = False, world_name: str = "empty", exp_suffix: str = ""):
        # 注册保存数据的函数到程序退出时调用
        atexit.register(self.save_data_when_exit)
        self.timer = Timer()
        self.enable_mst = enable_mst
        self.world_name = world_name
        self.exp_suffix = exp_suffix

        self.num_robot = num_robot
        self.robots: List[Robot] = []
        for i in range(self.num_robot):
            self.robots.append(Robot(name="robot_"+str(i+1)))

        self.flipped_convexhull = [[] for r in range(self.num_robot)]
        self.closest_laser_points = [[] for r in range(self.num_robot)]

        self.create_recorder(enable_record, save_folder)

        self.list_lock = threading.Lock()
        self.lead_timer = LeadingTimer(10)  # Check leading-robot stuck or not

        # Subscribe robots' position from Gazebo
        self.sub_modelStates = rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback_model_states)
        self.has_received_robots_initial_positions = False
        
        # Subscribe navigation goal published by RVIZ, this is specifically designed for robot 1
        self.sub_rviz_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback_nav_goal)
        
        # subscribe map
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        
        # Subscribe way_point from far_planner
        self.sub_waypoints = []
        for r in range(self.num_robot):
            topic_name = 'robot_'+str(r+1) + '/way_point'
            sub_waypoint = rospy.Subscriber(topic_name, PointStamped, self.callback_waypoints, callback_args=r)
            self.sub_waypoints.append(sub_waypoint)
        
        # Subscribe robots' visible region 
        self.sub_flipped_convexhull = []
        for r in range(self.num_robot):
            topic_name = 'robot_'+str(r+1) + '/vertices_convexhull'
            sub = rospy.Subscriber(topic_name, Float32MultiArray, self.callback_flipped_convexhull, callback_args=r)
            self.sub_flipped_convexhull.append(sub)
        
        # Subscribe frontier points as targets
        self.sub_frontier = rospy.Subscriber("/centroids_points", PointArray, self.callback_frontier)
        self.frontiers: List[tuple] = []
        self.block_frontiers: Set[tuple] = set()
        self.frontier_count_flag = 0
        
        # Subscribe stop command
        self.sub_stop = rospy.Subscriber('/stop', Bool, self.callback_stop)
        self.stopMove = True
        self.rejectStopMsg = False  # stopMove can be controlled by outside message publisher. This is to reject the stop message.

        # Publish robots' positions to Gazebo
        self.pub_model_state = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=5)
        
        # Publish connectivity edge to rviz
        self.pub_edge_marker = rospy.Publisher('/edge_marker', Marker, queue_size=10)
        self.pub_lambda2 = rospy.Publisher('/lambda2', Float32, queue_size=10)
        self.pub_nav_goal = rospy.Publisher('/goal_marker', Marker, queue_size=10)
        self.pub_targets = []
        for r in range(self.num_robot):
            pub = rospy.Publisher(f'/robot_{r+1}/goal_point', PointStamped, queue_size=10)
            self.pub_targets.append(pub)
        self.pub_derivative_arrow = rospy.Publisher('/derivative_arrow', MarkerArray, queue_size=10)
        self.pub_communication_range = rospy.Publisher('/communication_circle', MarkerArray, queue_size=10)
        self.pub_robot_pairs = rospy.Publisher('/robot_pairs', Float32MultiArray, queue_size=10)

        
        # Check whether the leading robot is defined or not
        self.has_leading_robot: bool = False
        self.leading_idx: int = -1

    def create_recorder(self, enable_record: bool, save_folder: str):
        """ save_folder only provides the folder, do not need to specify the file name. """
        self.enable_record = enable_record
        self.recorder = Recorder(self.num_robot, save_folder, enable_record)

    def save_data_when_exit(self):
        """ Only saving then enable_record is true. """
        if self.recorder.stop_time < 0:   # if navigation not stopped
            self.recorder.stop_record()
            print(f"{RED}Exploration unfinished after {self.recorder.get_running_time()} seconds!{RESET}")
        else:
            print(f"{RED}Exploration finished after {self.recorder.get_running_time()} seconds!{RESET}")
        
        print(f"{GREEN}world_name: {self.recorder.parameter['world_name']}{RESET}")
        print(f"{GREEN}use_dk: {self.recorder.parameter['use_dk']}, \t enable_mst: {self.recorder.parameter['enable_mst']}{RESET}")
        print(f"{GREEN}flip_R: {self.recorder.parameter['flip_radius']}, \t lambda2_prefer: {self.recorder.parameter['lambda2_prefer']}{RESET}")
        print(f"{GREEN}control_rate: {self.recorder.parameter['control_rate']}, \t navigation success: {self.recorder.parameter['successful']}{RESET}")
        
        if self.enable_record:
            self.recorder.save_record()
            print(f"{GREEN} All recorded data has been saved! {RESET}")
        else:
            print(f"{GREEN} Exist without saving record! {RESET}")

    def set_move_step(self, step):
        self.step = step

    def set_control_rate(self, control_rate):
        self.control_rate = control_rate

    def set_scaling_factor(self, leader_scaling_factor, 
                            basic_scaling_factor, 
                            leader_crucial_connectivity_force,
                            leader_crucial_scaling_factor,
                            all_crucial_connectivity_force,
                            all_crucial_scaling_factor):
        self.leader_scaling_factor = leader_scaling_factor
        self.basic_scaling_factor = basic_scaling_factor
        self.leader_crucial_connectivity_force = leader_crucial_connectivity_force
        self.leader_crucial_scaling_factor = leader_crucial_scaling_factor
        self.all_crucial_connectivity_force = all_crucial_connectivity_force
        self.all_crucial_scaling_factor = all_crucial_scaling_factor

    def set_robots_target_tolerance(self, target_tolerance: float):
        """ Set target tolerance in robot navigation. """
        for r in self.robots:
            r.set_target_tolerance(target_tolerance)
        
    def get_robot_positions(self) -> List[tuple]:
        # Get x, y, yaw of robot positions
        robot_positions = []
        for robot in self.robots:
            robot_positions.append(robot.get_pose())
        return robot_positions

    def get_flipped_convexhull(self) -> np.ndarray:
        with self.list_lock:
            copy_flipped_convexhull = copy.deepcopy(self.flipped_convexhull)
        return copy_flipped_convexhull
    
    def check_completeness_of_convexhull(self, flipped_convexhull: List[list]) -> bool:
        has_receive_all_convexhull = True
        for i in range(len(flipped_convexhull)):
            if len(flipped_convexhull[i]) == 0:
                has_receive_all_convexhull = False
                break
        return has_receive_all_convexhull

    def callback_flipped_convexhull(self, data: Float32MultiArray, robot_id: int):
        """_summary_

        Args:
            data (Float32MultiArray): the array of vertices of flipped convexhull (not the original space)
                                      the last two elements are (x, y) of the closest laser points
            robot_id (int): _description_
        """
        array = list(data.data)
        # rospy.loginfo(make_str_green(f"Receive convexHull size {len(array)} from robot {robot_id+1}"))
        # 使用单个锁保护对 shared_lists 的访问
        with self.list_lock:
            # 更新共享列表
            self.flipped_convexhull[robot_id] = array[:-2]
            self.closest_laser_points[robot_id] = array[-2:]
        return
    
    def callback_waypoints(self, data: PointStamped, robot_id: int):
        """ Subscribe waypoint from far_planner. """
        self.robots[robot_id].set_waypoint((data.point.x, data.point.y))
    
    def callback_nav_goal(self, msg):
        """ Navigation goal published by rviz tool """
        rospy.loginfo("Received 2D Navigation Goal:")
        self.robots[0].set_target((msg.pose.position.x, msg.pose.position.y))
        self.robots[0].set_waypoint((msg.pose.position.x, msg.pose.position.y))

        # Publish marker of the robot
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = msg.header.frame_id  # 确保这个frame_id和PoseStamped中的一致

        marker.ns = "goal_marker"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose = msg.pose
        marker.scale.x = 0.5  # Marker的尺寸
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 0.58  # Marker的颜色
        marker.color.g = 0.0
        marker.color.b = 0.83
        marker.color.a = 1.0  # 透明度
        # 发布Marker
        self.pub_nav_goal.publish(marker)

    def map_callback(self, map_msg: OccupancyGrid):
        # 从OccupancyGrid消息中提取地图数据并存储在类属性中
        map_data = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))
        map_resolution = map_msg.info.resolution
        map_origin = (map_msg.info.origin.position.x, map_msg.info.origin.position.y)
        self.recorder.update_map(map_data, map_resolution, map_origin)

    def callback_model_states(self, data: ModelStates):
        if self.has_received_robots_initial_positions:
            return
        robot_to_indices = {}
        for i, model_name in enumerate(data.name):
            if "robot" in model_name:  
                robot_to_indices[model_name] = i
        for k in range(self.num_robot):
            model_idx = robot_to_indices["robot_" + str(k+1)]
            robot_x, robot_y = data.pose[model_idx].position.x, data.pose[model_idx].position.y
            orientation = data.pose[model_idx].orientation
            quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
            euler = tf.euler_from_quaternion(quaternion)
            yaw = euler[2]  # 欧拉角中的yaw
            self.robots[k].set_pose((robot_x, robot_y, yaw))
        self.has_received_robots_initial_positions = True


    def callback_stop(self, data: Bool):
        if self.rejectStopMsg:
            return
        if data.data:
            self.stopMove = True
        else:
            self.stopMove = False

    def callback_frontier(self, data: PointArray):
        # rospy.loginfo("Received %d frontiers:" % len(data.points))
        new_frontiers: List[tuple] = []
        for i, point in enumerate(data.points):
            this_frontier = (round(point.x, 1), round(point.y, 1))
            if this_frontier not in self.block_frontiers:
                new_frontiers.append((point.x, point.y, point.z))
        self.frontiers = new_frontiers

    def clear_all_targets(self):
        """ Clear all robots' targets before a new frontier assignment. """
        for r in range(self.num_robot):
            self.robots[r].clear_target()

    def assign_frontiers(self, robot_positions: List[tuple], timer_robots_stay: Dict[int, int]) -> int:
        """ Re-assign all robots' targets once called, and return the closest robot as leading robot. 
        Return -1 of not frontier exists.
        """
        frontiers = copy.deepcopy(self.frontiers)
        if len(frontiers) == 0:
            return -1
        dist_robot_target = []
        for r in range(self.num_robot):
            for k in range(len(frontiers)):
                robot_pos = robot_positions[r]
                frontier = frontiers[k]
                dist = math.sqrt((robot_pos[0] - frontier[0])**2 + (robot_pos[1] - frontier[1])**2)
                dist_robot_target.append((dist, r, k))
        dist_robot_target.sort()
        assigned_robots = set()
        assigned_frontiers = set()
        leading_idx = -1
        for i in range(len(dist_robot_target)):
            _, r_idx, f_idx = dist_robot_target[i]
            if r_idx not in assigned_robots and f_idx not in assigned_frontiers:
                assigned_robots.add(r_idx)
                assigned_frontiers.add(f_idx)
                self.robots[r_idx].set_target((frontiers[f_idx][0], frontiers[f_idx][1]), force_reset = True)
                self.robots[r_idx].set_waypoint((frontiers[f_idx][0], frontiers[f_idx][1]))
                self.publish_target_to_far_planner(r_idx, (frontiers[f_idx][0], frontiers[f_idx][1]))
                if leading_idx == -1 and r_idx not in timer_robots_stay:  # Do not assign leading robots to robot that are currently stay!
                    leading_idx = r_idx
            if len(assigned_robots) == self.num_robot:
                break
        return leading_idx

    def assign_close_frontiers(self, robot_positions: List[tuple], timer_robots_stay: Dict[int, int]) -> int:
        """ Re-assign all robots' targets once called, and return the closest robot as leading robot. 
        Return -1 of not frontier exists.
        """
        frontiers = copy.deepcopy(self.frontiers)
        if len(frontiers) == 0:
            return -1
        dist_robot_target = []
        # Get robots center
        center = [0, 0]
        for r in range(self.num_robot):
            center[0] += robot_positions[r][0]
            center[1] += robot_positions[r][1]
        center[0] /= self.num_robot
        center[1] /= self.num_robot

        # Find closest frontier to center
        closest_k = -1
        dist = float("inf")
        for k in range(len(frontiers)):
            frontier = frontiers[k]
            dist_center = math.sqrt((center[0] - frontier[0])**2 + (center[1] - frontier[1])**2)
            if dist_center < dist:
                dist = dist_center
                closest_k = k
        target_frontier = frontiers[closest_k]
        
        for r in range(self.num_robot):
            for k in range(len(frontiers)):
                robot_pos = robot_positions[r]
                frontier = frontiers[k]
                dist = math.sqrt((robot_pos[0] - frontier[0])**2 + (robot_pos[1] - frontier[1])**2)
                # dist to center 
                dist_target_frontier = math.sqrt((target_frontier[0] - frontier[0])**2 + (target_frontier[1] - frontier[1])**2)
                dist_robot_target.append((dist_target_frontier, dist, r, k))
        dist_robot_target.sort()
        assigned_robots = set()
        assigned_frontiers = set()
        leading_idx = -1
        for i in range(len(dist_robot_target)):
            _, _, r_idx, f_idx = dist_robot_target[i]
            if r_idx not in assigned_robots and f_idx not in assigned_frontiers:
                assigned_robots.add(r_idx)
                assigned_frontiers.add(f_idx)
                self.robots[r_idx].set_target((frontiers[f_idx][0], frontiers[f_idx][1]), force_reset = True)
                self.robots[r_idx].set_waypoint((frontiers[f_idx][0], frontiers[f_idx][1]))
                self.publish_target_to_far_planner(r_idx, (frontiers[f_idx][0], frontiers[f_idx][1]))
                if leading_idx == -1 and r_idx not in timer_robots_stay:  # Do not assign leading robots to robot that are currently stay!
                    leading_idx = r_idx
            if len(assigned_robots) == self.num_robot:
                break
        return leading_idx
    
    def publish_target_to_far_planner(self, r: int, target: tuple):
        """ Publish target to robot r's far_planner, in map frame. """
        target_point = PointStamped()
        target_point.header = Header()
        target_point.header.stamp = rospy.Time.now()
        target_point.header.frame_id = "map"  # 可以根据实际需要设置坐标系
        # 填充Point信息
        target_point.point.x = target[0]
        target_point.point.y = target[1]
        target_point.point.z = 0
        self.pub_targets[r].publish(target_point)

    def publish_force_marker(self, connect_force: np.ndarray, explore_force: np.ndarray):
        id = 1
        marker_array = MarkerArray()
        robot_positions = self.get_robot_positions()
        for r in range(self.num_robot):
            marker = create_arrow_marker(id, robot_positions[r], connect_force[r], color=(1, 0, 0))
            id += 1
            marker_array.markers.append(marker)
            marker = create_arrow_marker(id, robot_positions[r], explore_force[r], color=(0, 1, 0))
            id += 1
            marker_array.markers.append(marker)
        self.pub_derivative_arrow.publish(marker_array)
        return
    
    def publish_communication_range_marker(self, radius: float):
        id = 1
        marker_array = MarkerArray()
        robot_positions = self.get_robot_positions()
        for r in range(self.num_robot):
            marker = create_circle_marker(id, robot_positions[r], radius, color=(0, 0, 1))
            id += 1
            marker_array.markers.append(marker)
        self.pub_communication_range.publish(marker_array)
        return

    def publish_connectivity_marker(self, generalized_laplacian: np.ndarray, minimum_edges: Set[tuple] = set()):
        robot_positions = self.get_robot_positions()
        connected_pairs = set()
        for r in range(self.num_robot):
            for j in range(r+1, self.num_robot):
                if generalized_laplacian[r, j] != 0:
                    if (j, r) not in connected_pairs:
                        connected_pairs.add((r, j))
        self.recorder.add_connect_pairs(connected_pairs)

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lines"
        marker.id = 1
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # 设置线条的颜色和大小
        marker.scale.x = 0.1  # 线条的宽度
        marker.color.a = 1.0  # 透明度

        # 设置 marker 的 pose
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # 添加点到线条
        for i, j in connected_pairs:
            this_pose = Point()
            this_pose.x = robot_positions[i][0]
            this_pose.y = robot_positions[i][1]
            this_pose.z = 0
            that_pose = Point()
            that_pose.x = robot_positions[j][0]
            that_pose.y = robot_positions[j][1]
            that_pose.z = 0
            marker.points.append(this_pose)
            marker.points.append(that_pose)

            # 设置线条的颜色
            weight = generalized_laplacian[i, j]
            r, g, b = weight_to_rgb(abs(weight))
            start_color = ColorRGBA()
            # start_color = marker.color  # 默认透明度
            if (i, j) not in minimum_edges and (j, i) not in minimum_edges:
                start_color.r = 0.0
                start_color.g = 0.0
                start_color.b = 1.0
                start_color.a = 0.5
            else:
                start_color.r = r
                start_color.g = g
                start_color.b = b
                start_color.a = 1
            marker.colors.append(start_color)  # 起点颜色
            marker.colors.append(start_color)  # 终点颜色保持一致

        self.pub_edge_marker.publish(marker)
        
    def get_closest_laser_points(self) -> np.ndarray:
        with self.list_lock:
            closest_laser_points = copy.deepcopy(self.closest_laser_points)
        return closest_laser_points
        
    def set_robot_initial_positions(self):
        """ Set robot initial positions."""
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("connectivity_exploration")
        yaml_path = os.path.join(package_path, "param", "tasks", f"env_{self.world_name}{self.exp_suffix}.yaml")

        with open(yaml_path, "r") as file:
            data = yaml.safe_load(file)
        for r in data["robot_list"]:
            self_position = data[f"robot_{r}"]["self_position"]
            self.robots[r].set_pose(tuple(self_position))
            self.set_robot_modelState(r, tuple(self_position))
    
    def publish_connect_pairs(self, generalized_laplacian: np.ndarray):
        connected_pairs = set()
        for r in range(self.num_robot):
            for j in range(r+1, self.num_robot):
                if generalized_laplacian[r, j] != 0:
                    if (j, r) not in connected_pairs:
                        connected_pairs.add((r, j))
        robot_pair = Float32MultiArray()
        for x, y in connected_pairs:
            robot_pair.data.append(x)
            robot_pair.data.append(y)
        self.pub_robot_pairs.publish(robot_pair)
    
    def cal_exploration_force(self, r: int, timer_robots_stay: Dict[int, int], u_connect: np.ndarray, max_connect_potential: float, is_lead_robot_efficient: bool) -> Tuple[bool, np.ndarray]:
        """ Return: (robotNeedStay, force). 
        If robotNeedStay, u_combined should be set to zero. """
        robotNeedStay = True
        u_explore = np.zeros(2)
        if r in timer_robots_stay:   # the robots that stay at their target
            timer_robots_stay[r] -= 1
            if timer_robots_stay[r] <= 0:
                del timer_robots_stay[r]
            return robotNeedStay, u_explore
        
        if self.robots[r].has_a_target():
            if not is_lead_robot_efficient and r != self.leading_idx:
                return not robotNeedStay, u_explore

            # Calculate exploration force
            diff_waypoint_x = self.robots[r].get_waypoint()[0] - self.robots[r].get_pose()[0]
            diff_waypoint_y = self.robots[r].get_waypoint()[1] - self.robots[r].get_pose()[1]
            diff_waypoint = np.array([diff_waypoint_x, diff_waypoint_y])
            dist_waypoint = np.linalg.norm(diff_waypoint)
            if dist_waypoint <= self.step:
                u_explore = diff_waypoint
            else:
                u_explore = diff_waypoint * self.step / dist_waypoint

            # Scaling exploration force
            this_scaling_factor = self.basic_scaling_factor
            if r == self.leading_idx:
                this_scaling_factor = self.leader_scaling_factor
                if max_connect_potential > self.leader_crucial_connectivity_force:
                    this_scaling_factor = self.leader_crucial_scaling_factor
            if max_connect_potential > self.all_crucial_connectivity_force:
                this_scaling_factor = self.all_crucial_scaling_factor
            u_explore *= this_scaling_factor

            # Add force alignment weight   
            norm_u_explore = np.linalg.norm(u_explore)
            u_connect = np.copy(u_connect[r])
            norm_u_connect = np.linalg.norm(u_connect)
            if r != self.leading_idx and norm_u_explore > 1e-4 and norm_u_connect > 1e-4:
                cos_theta = np.dot(u_explore / norm_u_explore, u_connect / norm_u_connect)
                weight_direction = 0.5 * (1 + cos_theta)
                u_explore *= weight_direction
            
            return not robotNeedStay, u_explore
        
        else:
            return not robotNeedStay, u_explore
        
    def set_robot_modelState(self, r: int, pose: tuple):
        """ Set robot's pose in Gazebo. """
        # Publish robot pose in Gazebo
        robot_state = ModelState()
        robot_state.model_name = self.robots[r].get_name()
        # 创建四元数
        vehicle_roll = 0.0
        vehicle_pitch = 0.0
        vehicle_yaw = pose[2]
        geo_quat = tf.quaternion_from_euler(vehicle_roll, vehicle_pitch, vehicle_yaw)
        robot_state.pose.orientation = Quaternion(*geo_quat)
        # 设置位置
        robot_state.pose.position.x = pose[0]
        robot_state.pose.position.y = pose[1]
        robot_state.pose.position.z = 0.0
        self.pub_model_state.publish(robot_state)

      

    def main_loop(self, general_lp: GeneralizedLaplacian):
        # record parameters
        self.recorder.record_param_enable_mst(self.enable_mst)
        self.recorder.record_param_control_rate(self.control_rate)
        self.recorder.record_param_use_dk(general_lp.use_dk)
        self.recorder.record_param_lambda2_prefer(general_lp.prefer_lambda2)
        self.recorder.record_param_flip_radius(general_lp.flip_radius)
        self.recorder.record_param_worldname(self.world_name)
        self.recorder.record_param_exp_suffix(self.exp_suffix)

        loop_rate = self.control_rate
        self.rate = rospy.Rate(loop_rate)  # 10 Hz
        loop_period = 1000 // loop_rate

        timer_robots_stay: Dict[int, int] = {} # Important global timer to urge robots stay at their targets
        stay_steps = 0.5 * loop_rate  # 3 seconds
        time_last_print = time.time()
        
        self.team_disconnected = False
        hasPrintTeamDisconnected = False
        
        while not rospy.is_shutdown():
            enable_print = False
            if time.time() - time_last_print > 0.5:
                enable_print = True
                time_last_print = time.time()
                
            if self.team_disconnected: # Team disconnected
                if not hasPrintTeamDisconnected:
                    my_print(f"{RED}Team disconnected! Stop testing! {RESET}", enable_print) 
                    hasPrintTeamDisconnected = True
                self.rate.sleep()
                continue

            if self.stopMove:  # Stop navigation
                if self.frontier_count_flag >= loop_rate*5:
                    my_print(f"{RED}Finish Navigation after {self.recorder.get_running_time()} seconds! Current: {rospy.get_time() - self.recorder.start_time} seconds.{RESET}", enable_print)
                self.rate.sleep()
                continue

            start_time = rospy.get_time()
            self.timer.tic()
            #------------------ 1. Get robots' positions ----------------------#
            robot_positions = self.get_robot_positions()
            info_str = "\n" + f"{GREEN}Robot positions:{RESET}\n"
            for r, pose in enumerate(robot_positions):
                info_str += f"\trobot {r+1}: ({round(pose[0], 4)}, {round(pose[1], 4)}) \n"
            # rospy.loginfo(info_str)

            #------------------ 2. Calculate distance matrix ----------------------#
            general_lp.update_robot_positions(robot_positions)
            distance_matrix = general_lp.get_distance_matrix()
            my_print(f"T_position_update: {self.timer.toc():.3f} seconds", enable_print) 

            #------------------ 3. Wait until all robot's convexhulls are recevied ----------------------#
            self.timer.tic()
            flipped_convexhull = self.get_flipped_convexhull()
            if not self.check_completeness_of_convexhull(flipped_convexhull):
                self.timer.toc()
                self.rate.sleep()
                continue

            #------------------ 4. Update each robot's closest obstacle point (in odom frame)! ------#
            closest_laser_points = self.get_closest_laser_points()
            general_lp.update_obstacle_points(closest_laser_points)  
            # print(f"{GREEN}Obstacle points:{RESET}:")
            # for r, point in enumerate(general_lp.obstacle_points):
            #     print(f"\trobot {r}:" + f"({round(point[0], 3)}, {round(point[1], 3)})")
            # general_lp.clear_obstacle_points()  # whether obstacle points are considered
            
            #------------------ 5. Calculate Laplacian matrix and lambda_2 ----------------------#
            general_lp.update_hulls_and_normal_vectors(flipped_convexhull)
            general_lp.interoplate_hull_faces(angle_step=2)
            general_lp.update_dists_to_hull_faces(flipped_convexhull, distance_matrix)
            my_print(f"T_convexhull_update: {self.timer.toc():.3f} seconds", enable_print) 
            
            self.timer.tic()
            general_lp.clear_weights_recorder()
            is_graph_connected, minimum_edges = general_lp.get_minimum_topology(distance_matrix, 
                                                            add_communication=True, 
                                                            add_collision=True,
                                                            add_los = True,
                                                            enable=self.enable_mst)        
            if self.enable_mst:  # print edge weight for debugging
                selected_weights = f"MST_weights({general_lp.num_graph_edges}): "
                for vStart, vEnd in minimum_edges:
                    if (vStart, vEnd) in general_lp.combined_weight_record:
                        selected_weights += f"[{vStart+1}, {vEnd+1}, {general_lp.combined_weight_record[(vStart, vEnd)]:.3f}]"
                    else:
                        selected_weights += f"[{vEnd+1}, {vStart+1}, {general_lp.combined_weight_record[(vEnd, vStart)]:.3f}]"
                my_print(selected_weights, enable_print)
            
            if not is_graph_connected:
                if self.stopMove:  
                    print(robot_positions)
                self.team_disconnected = True
                if not self.stopMove:
                    self.recorder.stop_record()
                self.rejectStopMsg = True
                self.stopMove = True
                self.rate.sleep()
                continue
            
            my_print(f"T_Laplacian_calculation (1): {self.timer.toc():.3f} seconds", enable_print) 
            self.timer.tic()
            
            generalized_laplacian, actual_laplacian = general_lp.get_masked_generalized_laplacian_matrix(distance_matrix, 
                                                            minimum_edges=minimum_edges)
            self.publish_connect_pairs(actual_laplacian)  # publish connected robot pairs
            actual_lambda2, _ = general_lp.get_second_eigenvalue_eigenvector(actual_laplacian)
            lambda2, eigen_vector2 = general_lp.get_second_eigenvalue_eigenvector(generalized_laplacian)
            my_print(f"T_Laplacian_calculation (2): {self.timer.toc():.3f} seconds", enable_print) 

            #------------------ 6. Calculate connectivity potential of each robot ----------------------#
            self.timer.tic()
            u_connect = np.zeros((self.num_robot, 2))
            dt = 1
            max_connect_potential = 0
            my_print(f"{GREEN}Derivative:{RESET}:", enable_print)
            for r in range(self.num_robot):
                # dx, dy = general_lp.get_gradient(r, lambda2, eigen_vector2, distance_matrix)
                dx, dy = general_lp.get_masked_gradient(r, actual_lambda2, eigen_vector2, distance_matrix, minimum_edges=minimum_edges)
                u_connect[r, :] = [dx*dt, dy*dt]
                force_connect_abs = np.linalg.norm(u_connect[r])
                max_connect_potential = max([abs(max_connect_potential), force_connect_abs])
                self.recorder.add_connect_force_original(force_connect_abs, r)
                my_print(f"\trobot {r+1}: ({round(dx*dt, 3):<7}, {round(dy*dt, 3):<7}) total: {round(force_connect_abs, 3):<7}", enable_print)

            #------------------ 7. Scale connectivity force for each robot ----------------------#
            if max_connect_potential > self.step:
                for r in range(self.num_robot):
                    abs_movement = np.linalg.norm(u_connect[r])
                    if abs_movement > self.step:
                        u_connect[r] = u_connect[r]/abs_movement * self.step
            
            my_print(f"{GREEN}Connectivity Force:{RESET}:", enable_print)
            for r in range(self.num_robot):
                force_connect_abs =np.linalg.norm(u_connect[r])
                self.recorder.add_connect_force(force_connect_abs, r)
                my_print(f"\trobot {r+1}: ({round(u_connect[r][0], 3):<7}, {round(u_connect[r][1], 3):<7}) total: {round(force_connect_abs, 3):<7}", enable_print)
            
            
            visual_connect_force = np.copy(u_connect)
            my_print(f"T_connectivity potential: {self.timer.toc():.3f} seconds", enable_print) 
            

            #------------------ 8. Check leading robot && assign targets to robots ----------------------#
            self.timer.tic()
            message_terminal = ""
            if self.has_leading_robot:
                # 8.1 Update current navigation status of robots
                for r in range(self.num_robot):
                    if r not in timer_robots_stay and self.robots[r].has_a_target():
                        if self.robots[r].has_reach_target():
                            timer_robots_stay[r] = stay_steps
                            self.robots[r].clear_target()
                        # This step is to avoid new waypoint is not generated due to unreachable of goal
                        if self.robots[r].has_reach_waypoint() and not self.robots[r].has_reach_target():
                            message_terminal += f"{RED}Robot {r+1} stucks halfway! {RESET} "
                            # print(f"{RED}Target of robot {r+1} in obstacles! {RESET}")
                            timer_robots_stay[r] = stay_steps // 2
                            self.robots[r].clear_target()
                # Two situations to trigger task assignment process
                # 1. leading robot reaches targets
                # 2. leading robot's movement is less than 0.3m within 10 seconds
                if self.leading_idx in timer_robots_stay or not self.lead_timer.is_leading_robot_moving(robot_positions[self.leading_idx]):
                    if self.leading_idx in timer_robots_stay:
                        message_terminal += f"Leading-robot {self.leading_idx+1} {RED}reach target!{RESET} "
                        # print(f"Robot {self.leading_idx+1} {RED}reach target!{RESET}")
                    else:  # Not reach target for too long, and without any further movement
                        # Add leading robot to stay list
                        timer_robots_stay[self.leading_idx] = stay_steps // 3
                        message_terminal += f"Leading-robot {self.leading_idx+1} {RED}stucks for a long time!{RESET}"
                        # print(f"Robot {self.leading_idx+1} {RED}cannot reach target after long time attempt!{RESET}")
                        this_target = self.robots[self.leading_idx].get_target()
                        self.block_frontiers.add((round(this_target[0], 1), round(this_target[1], 1)))
                    self.has_leading_robot = False
                    # Assign new target
                    self.clear_all_targets()
                    self.leading_idx = self.assign_close_frontiers(robot_positions, timer_robots_stay) 
                    if self.leading_idx >= 0:
                        self.has_leading_robot = True
                        self.lead_timer.start(robot_positions[self.leading_idx])
                # Else if not reach target, do not assign targets to other robots now
            else:
                # Assign new targets
                self.clear_all_targets()
                self.leading_idx = self.assign_close_frontiers(robot_positions, timer_robots_stay) 
                if self.leading_idx >= 0:
                    self.has_leading_robot = True
                    self.lead_timer.start(robot_positions[self.leading_idx])
            self.recorder.add_leading_idx(self.leading_idx)
            my_print(message_terminal, enable_print)
            my_print(f"{self.timer.toc():.3f} seconds for task assignment", enable_print) 
                    

            #------------------ 9. Calculate exploration force for each robot----------------------#
            is_lead_robot_efficient = True
            if self.has_leading_robot:
                is_lead_robot_efficient = self.lead_timer.check_move_efficiency(robot_positions[self.leading_idx], self.leading_idx)
            my_print(f"Leading robot moves fast? --- {is_lead_robot_efficient}", enable_print)
            
            visual_explore_force = np.zeros((self.num_robot, 2))
            my_print(f"{GREEN}Exploration Force:{RESET}:", enable_print)
            
            u_combined = np.copy(u_connect)
            for r in range(self.num_robot):
                robotNeedStay, u_explore = self.cal_exploration_force(r, timer_robots_stay, u_connect, max_connect_potential, is_lead_robot_efficient)
                my_print(f"\trobot {r+1}: ({round(u_explore[0], 3):<7}, {round(u_explore[1], 3):<7}) total: {round(np.linalg.norm(u_explore), 3):<7}", enable_print)
                self.recorder.add_explore_force(np.linalg.norm(u_explore), r)
                visual_explore_force[r] = u_explore
                
                if robotNeedStay:
                    u_combined[r] = np.zeros(2)
                    continue
                u_combined[r] += u_explore
                norm_u_combined = np.linalg.norm(u_combined[r])
                if norm_u_combined > self.step:
                    u_combined[r] = u_combined[r]/norm_u_combined * self.step
                
            self.publish_force_marker(visual_connect_force, visual_explore_force)
            self.publish_communication_range_marker(general_lp.d_comm_min)

            my_print(f"{GREEN}Leading robot: {RESET} {self.leading_idx+1} (start from 1)", enable_print)
            str_robots_with_targets = ""
            for r in range(self.num_robot):
                if self.robots[r].has_a_target():
                    str_robots_with_targets += str(r+1) + " "
            my_print(f"{GREEN}Robots with targets: {RESET} {str_robots_with_targets}", enable_print)
            str_robot_stay = ""
            for r in timer_robots_stay.keys():
                str_robot_stay += str(r+1) + f"({timer_robots_stay[r]})  "
            my_print(f"{GREEN}Robot stays: {RESET} {str_robot_stay}", enable_print)
            
            my_print(f"T_exploration_control: {self.timer.toc():.3f} seconds", enable_print) 

            #------------------ 11: Update robot positions ----------------------# 
            if not self.stopMove:  # Whether the movement of robots is disabled
                for r in range(self.num_robot):
                    curr_pose = self.robots[r].get_pose()
                    new_pose = (curr_pose[0] + u_combined[r][0], curr_pose[1] + u_combined[r][1], curr_pose[2])
                    # 为防止下一次循环时，机器人仍没根据modelState更新位置，因此保留此处的更新
                    self.robots[r].set_pose(new_pose)
                    self.set_robot_modelState(r, new_pose)
            
            # Publishing and recording
            self.recorder.add_lambda2(actual_lambda2)
            my_print(f"{GREEN}lambda2{RESET}: {round(actual_lambda2, 4)} ({round(lambda2, 4)})", enable_print)
            self.publish_connectivity_marker(actual_laplacian, minimum_edges=minimum_edges)
            self.pub_lambda2.publish(actual_lambda2)
            for r, pose in enumerate(robot_positions):
                self.recorder.add_positions(pose, r)

            loop_time = round((rospy.get_time() - start_time)*1000)
            self.recorder.add_control_frequency(loop_time)
            if loop_time > loop_period:
                my_print(f"{RED}Loop takes {loop_time:<5} / {loop_period:<5} ms. Losing desired rate!!!{RESET}", enable_print)
            else:
                my_print(f"Loop takes {loop_time:<5} / {loop_period:<5} ms.", enable_print)
            
            #------------------ 10: Check whether there are still frontiers ----------------------#
            my_print(f"{GREEN}#Frontier: {RESET} {len(self.frontiers):<6} {GREEN}#Block Frontier: {RESET} {len(self.block_frontiers):<6}", enable_print)
            if not self.stopMove:
                my_print(f"Navigation for {self.recorder.get_running_time()} seconds!", enable_print)
                if len(self.frontiers) == 0:
                    self.frontier_count_flag += 1
                    if self.frontier_count_flag >= loop_rate*5:  # 连续5秒以上没有新的frontier,则停止记录
                        self.recorder.record_param_successful()  # Record the success of navigation
                        self.recorder.stop_record()
                        self.rejectStopMsg = True
                        self.stopMove = True
                else:
                    self.frontier_count_flag = 0
            self.rate.sleep()


def main():
    rospy.init_node('connectivity_controller', anonymous=True)
    
    # 从参数服务器中获取 robot_number 参数
    robot_number = rospy.get_param('/robot_number', 0)  # 默认值为0

    ## Coefficients
    # Lambda function
    lambda_prefer = rospy.get_param('/lambda_prefer', 0.01)

    # Control rate
    control_rate = rospy.get_param('/control_rate', 30)

    # Communication radius
    k_gamma = rospy.get_param('/k_gamma', 1)  # coefficient of the communication radius constraints 
    d_comm_max = rospy.get_param('/d_comm_max', 30) # radius beyond which the communication fails 
    d_comm_min = rospy.get_param('/d_comm_min', 25) # radius beyond which the communication quality reduces

    # Prefered inter-robot distance
    k_beta = rospy.get_param('/k_beta', 1)  # coefficient of the preferred inter-robot constraints
    d_inter = rospy.get_param('/d_inter', 15)  # prefered inter-robot distance
    sigma = rospy.get_param('/sigma', 20) # how hard the prefered inter-robot distance constraints is
    
    # Collision avoidance with other robots
    k_alpha = rospy.get_param('/k_alpha', 1)
    d_coll_max = rospy.get_param('/d_coll_max', 13)
    d_coll_min = rospy.get_param('/d_coll_min', 10)

    # Collision avoidance with obstacle
    k_alpha_obs = rospy.get_param('/k_alpha_obs', 1)
    d_coll_obs_max = rospy.get_param('/d_coll_obs_max', 13)
    d_coll_obs_min = rospy.get_param('/d_coll_obs_min', 10)

    # LoS constraints
    k_omega = rospy.get_param('/k_omega', 1)
    d_los_max = rospy.get_param('/d_los_max', 10)
    d_los_min = rospy.get_param('/d_los_min', 0.5)
    alpha_relax = rospy.get_param('/alpha_relax', 1)
    beta_relax = rospy.get_param('/beta_relax', 1)
    flip_radius = rospy.get_param('/flip_radius', 500)

    # scaling_factor to balance connect and explore force
    leader_scaling_factor = rospy.get_param('/leader_scaling_factor', 1.2)
    basic_scaling_factor = rospy.get_param('/basic_scaling_factor', 0.6)   # default value of scaling factor for exploration force
    leader_crucial_connectivity_force = rospy.get_param('/leader_crucial_connectivity_force', 2)
    leader_crucial_scaling_factor = rospy.get_param('/leader_crucial_scaling_factor', 0.6)
    all_crucial_connectivity_force = rospy.get_param('/all_crucial_connectivity_force', 5)
    all_crucial_scaling_factor = rospy.get_param('/all_crucial_scaling_factor', 0.0)

    # Reach target tolerance
    target_tolerance = rospy.get_param('/target_tolerance', 1.2)


    # Final movement step
    step = rospy.get_param('/step', 0.2)

    # delta movement range
    use_dk = rospy.get_param('/use_dk', False)
    enable_mst = rospy.get_param('/enable_mst', False)

    # Folder to save data
    enable_record = rospy.get_param('/enable_record', False)
    save_folder = rospy.get_param('/save_folder', "~/connectivity_record")

    # Print control
    print_edge_weight = rospy.get_param('/print_edge_weight', True)
    
    world_name = rospy.get_param('/env_name', "empty")
    exp_suffix = rospy.get_param('/exp_suffix', "")  # the suffix to distinguish different experiments in the same environment
    
    # 创建 ModelStatesListener 实例
    robot_controller = Controller(robot_number, enable_record, save_folder, enable_mst=enable_mst, world_name=world_name, exp_suffix=exp_suffix) # Create recorder that save data
    robot_controller.set_move_step(step)
    robot_controller.set_control_rate(control_rate)
    robot_controller.set_scaling_factor(leader_scaling_factor, 
                                        basic_scaling_factor, 
                                        leader_crucial_connectivity_force,
                                        leader_crucial_scaling_factor,
                                        all_crucial_connectivity_force,
                                        all_crucial_scaling_factor)
    robot_controller.set_robots_target_tolerance(target_tolerance)

    general_lp = GeneralizedLaplacian(use_dk=use_dk)  # Set the discrete movement step
    general_lp.set_print_edge_weight(print_edge_weight)
    general_lp.set_prefer_lambda2(lambda_prefer)
    general_lp.set_communication_parameters(k_gamma, d_comm_max, d_comm_min)
    general_lp.set_prefered_distance_parameters(k_beta, d_inter, sigma)
    general_lp.set_collision_paramters(k_alpha, d_coll_max, d_coll_min)
    general_lp.set_collision_obstacle_paramters(k_alpha_obs, d_coll_obs_max, d_coll_obs_min)
    general_lp.set_los_paramters(k_omega, d_los_max, d_los_min, alpha_relax, beta_relax, flip_radius)
    
    rospy.sleep(2) # Sleep seconds for callback functions to work

    # Read targets from yaml file
    robot_controller.set_robot_initial_positions()
    robot_controller.main_loop(general_lp)
    
    # rospy.spin() 在这里不再需要，因为 main_loop 中已经有循环

if __name__ == '__main__':
    gc.collect()  # 强制触发垃圾回收
    faulthandler.enable()
    print("NumPy Debug: Running")

    np.seterr(all='raise')  # 让 NumPy 直接报错而不是崩溃
    try:
        main()
    except rospy.ROSInterruptException:
        pass
