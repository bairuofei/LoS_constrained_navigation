import time
from typing import List
import datetime
import numpy as np
import os
import rospy

from utils import save_data, GREEN, RED, RESET



class Recorder:
    def __init__(self, robot_number: int, save_folder: str, enable_record: bool = True):
        self.start_time = rospy.get_time()
        self.robot_number = robot_number
        self.record_lambda2 = []
        self.record_connect_original = [[] for _ in range(self.robot_number)]
        self.record_connect = [[] for _ in range(self.robot_number)]
        self.record_explore = [[] for _ in range(self.robot_number)]
        self.record_positions = [[] for _ in range(self.robot_number)]
        self.record_connect_pairs: List[set] = []
        self.record_control_frequency = []
        self.leading_idx = []
        self.example_los_distance: dict = {}
        self.example_cos_theta_k: dict = {}
        self.save_folder = save_folder
        self.map_updated = False
        if enable_record:
            self.disable_record = False  # Indicator to stop record
            self.enable_record = True
        else:
            self.disable_record = True
            self.enable_record = False
        self.stop_time = -1
        self.parameter = {"successful": False}

    def record_param_use_dk(self, use_dk: bool):
        self.parameter["use_dk"] = use_dk
    
    def record_param_lambda2_prefer(self, lambda2_prefer: float):
        self.parameter["lambda2_prefer"] = lambda2_prefer

    def record_param_enable_mst(self, enable_mst: bool):
        self.parameter["enable_mst"] = enable_mst
    
    def record_param_flip_radius(self, flip_radius: float):
        self.parameter["flip_radius"] = flip_radius
    
    def record_param_control_rate(self, control_rate: float):
        self.parameter["control_rate"] = control_rate

    def record_param_successful(self):
        self.parameter["successful"] = True

    def record_param_worldname(self, world_name: str):
        self.parameter["world_name"] = world_name
        
    def record_param_exp_suffix(self, exp_suffix: str):
        self.parameter["exp_suffix"] = exp_suffix

    
    def clear(self):
        self.record_lambda2 = []
        self.record_connect_original: List[list] = [[] for _ in range(self.robot_number)]
        self.record_connect: List[list] = [[] for _ in range(self.robot_number)]
        self.record_explore = [[] for _ in range(self.robot_number)]
        self.record_positions = [[] for _ in range(self.robot_number)]
        self.record_connect_pairs: List[set] = []
        self.leading_idx = []
        self.example_los_distance: dict = {}
        self.example_cos_theta_k: dict = {}

    def stop_record(self):
        self.stop_time = rospy.get_time()
        self.disable_record = True

    def get_running_time(self):
        if self.stop_time >= 0:
            return self.stop_time - self.start_time
        return rospy.get_time() - self.start_time
    
    def add_lambda2(self, lambda2: float):
        if not self.disable_record:
            self.record_lambda2.append(lambda2)

    def add_control_frequency(self, control_frequency: float):
        if not self.disable_record:
            self.record_control_frequency.append(control_frequency)

    def add_connect_force_original(self, connect_force_original: float, robot_idx: int):
        """ Record the norm for connectivity without scaling """
        if not self.disable_record:
            self.record_connect_original[robot_idx].append(connect_force_original)

    def add_connect_force(self, connect_force: float, robot_idx: int):
        """ Record the norm for connectivity after scaling """
        if not self.disable_record:
            self.record_connect[robot_idx].append(connect_force)
    
    def add_explore_force(self, explore_force: float, robot_idx: int):
        """ Record the norm for exploration """
        if not self.disable_record:
            self.record_explore[robot_idx].append(explore_force)

    def add_leading_idx(self, robot_idx):
        if not self.disable_record:
            self.leading_idx.append(robot_idx)

    def add_los_distance(self, los_distance: tuple, key):
        """(d_los_ij, d_los_ji, softmin_d_los)"""
        if not self.disable_record:
            if key in self.example_los_distance:
                self.example_los_distance[key].append(los_distance)
            else:
                self.example_los_distance[key] = [los_distance]
    
    def add_cos_theta_k(self, cos_theta_k: tuple, key):
        """(dji_k*, cos_theta_ji*, dij_k*, cos_theta_ij*)"""
        if not self.disable_record:
            if key in self.example_cos_theta_k:
                self.example_cos_theta_k[key].append(cos_theta_k)
            else:
                self.example_cos_theta_k[key] = [cos_theta_k]

    def add_positions(self, position: tuple, robot_idx: int):
        if not self.disable_record:
            self.record_positions[robot_idx].append(position)

    def add_connect_pairs(self, connect_pairs: set):
        """ Should be align with the time stamp of positions. """
        if not self.disable_record:
            self.record_connect_pairs.append(connect_pairs)

    def update_map(self, map_data: np.array, map_resolution: float, map_origin: tuple):
        if not self.disable_record:
            self.map_data = map_data
            self.map_resolution = map_resolution
            self.map_origin = map_origin
            self.map_updated = True

    def save_record(self):
        current_datetime = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M")
        save_dir = self.save_folder+ "/" + current_datetime
        save_dir += "_" + self.parameter["world_name"]
        save_dir += self.parameter["exp_suffix"]
        if self.parameter["use_dk"]:
            save_dir += "_Dk"
        else:
            save_dir += "_NoDk"
        if self.parameter["enable_mst"]:
            save_dir += "_Mst"
        else:
            save_dir += "_NoMst"
        if self.parameter["successful"]:
            save_dir += "_success"
        else:
            save_dir += "_fail"
        os.makedirs(save_dir, exist_ok=True)
        save_data(self.record_lambda2, save_dir + "/lambda2.pkl")
        save_data(self.record_connect, save_dir + "/connect.pkl")  # normalized connectivity force
        save_data(self.record_connect_original, save_dir + "/connect_original.pkl")
        save_data(self.record_explore, save_dir + "/explore.pkl") # normalized and scaled connectivity force
        save_data(self.leading_idx, save_dir + "/leading_idx.pkl")
        save_data(self.example_los_distance, save_dir + "/example_los_distance.pkl")
        save_data(self.example_cos_theta_k, save_dir + "/example_cos_theta_k.pkl")
        save_data(self.record_positions, save_dir + "/positions.pkl")
        save_data(self.record_connect_pairs, save_dir + "/connect_pairs.pkl")
        if self.map_updated:
            save_data(self.map_data, save_dir + "/map.pkl")
            save_data(self.map_resolution, save_dir + "/map_resolution.pkl")
            save_data(self.map_origin, save_dir + "/map_origin.pkl")
        save_data(self.record_control_frequency, save_dir + "/control_freq.pkl")
        save_data([self.get_running_time()], save_dir + "/run_time.pkl")
        save_data(self.parameter, save_dir + "/parameter.pkl")
        print(f"{GREEN}Stop recording after {round(self.get_running_time(), 2)} seconds.{RESET}")
        return