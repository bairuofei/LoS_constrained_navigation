import math
import numpy as np
import time
from numpy import linalg as LA
from typing import Tuple, List, Set

from utils import make_str_green, make_str_red, transform_to_local, transform_to_original, RED, GREEN, PURPLE, RESET,\
    save_data, get_outward_normal, spherical_flipping, interpolate_points, cartesian_to_polar, get_all_dist_and_gradient,\
    flip_vertices_of_hull_faces, construct_hull_with_small_faces
from GraphMST import GraphMST

class GeneralizedLaplacian:
    def __init__(self, use_dk=False) -> None:
        self.obstacle_points: List[tuple] = []
        self.constant_c = 3
        self.print_edge_weight = True
        self.save_before = True
        self.save_count = 0
        self.create_weights_recorder()
        self.los_debug_save = False
        self.dim = 2  # dimension of the robot's pose
        self.use_dk = use_dk
        return
    
    ## Set paramters
    def set_prefer_lambda2(self, lambda2):
        self.prefer_lambda2 = lambda2

    def set_communication_parameters(self, k_gamma, d_comm_max, d_comm_min):
        self.k_gamma = k_gamma
        self.d_comm_max = d_comm_max
        self.d_comm_min = d_comm_min

    def set_prefered_distance_parameters(self, k_beta, d_inter, sigma):
        self.k_beta = k_beta
        self.d_inter = d_inter
        self.sigma = sigma

    def set_collision_paramters(self, k_alpha, d_coll_max, d_coll_min):
        self.k_alpha = k_alpha
        self.d_coll_max = d_coll_max
        self.d_coll_min = d_coll_min

    def set_collision_obstacle_paramters(self, k_alpha_obs, d_coll_obs_max, d_coll_obs_min):
        self.k_alpha_obs = k_alpha_obs
        self.d_coll_obs_max = d_coll_obs_max
        self.d_coll_obs_min = d_coll_obs_min

    def set_los_paramters(self, k_omega, d_los_max, d_los_min, alpha_relax, beta_relax, flip_radius):
        self.k_omega = k_omega
        self.d_los_max = d_los_max
        self.d_los_min = d_los_min
        self.alpha_relax = alpha_relax
        self.beta_relax = beta_relax
        self.flip_radius = flip_radius

    def set_print_edge_weight(self, print_edge_weight: bool):
        self.print_edge_weight = print_edge_weight

    def my_print(self, worlds):
        """ Print control based on flag parameter"""
        if self.print_edge_weight:
            print(worlds)

    def create_weights_recorder(self): 
        self.gamma_record = {}   # communication weight
        self.alpha_record = {}   # collision avoidance weight
        self.omega_record = {}   # LoS weight
        self.combined_weight_record = {}  # combined weight
        self.los_dist_record = {}  # LoS distance
        self.exact_los_dist_record = {}  # exact LoS distance

    def clear_weights_recorder(self):
        self.gamma_record.clear()
        self.alpha_record.clear()
        self.omega_record.clear()   
        self.combined_weight_record.clear()
        self.los_dist_record.clear()
        self.exact_los_dist_record.clear()

    ## Update states in each control cycle
    def update_robot_positions(self, robot_positions: List[tuple]):
        """ Update robot positions."""
        self.robot_positions = robot_positions

    def update_hulls_and_normal_vectors(self, flipped_convexhull: List[list]):
        """Update robots' normal vectors of their convexhull in List[list] data structure.
        Args:
            flipped_convexhull (List[list]): the vertices that forms the flipped convexhull of each robot
        Returns:
            List[List[tuple]]: the reguralized normal vector of each edge that forms the convexhull
        """
        robot_number = len(flipped_convexhull)
        self.all_hull_normal_vectors = []
        self.all_hull_faces = []
        for r in range(robot_number):
            this_normal_vectors = []
            this_hull_vertices = []
            for i in range(0, len(flipped_convexhull[r]), 2):
                i_next = (i+2) % len(flipped_convexhull[r])
                point1 = (flipped_convexhull[r][i], flipped_convexhull[r][i+1])
                point2 = (flipped_convexhull[r][i_next], flipped_convexhull[r][i_next+1])
                this_hull_vertices.append([point1, point2])
                this_normal_vectors.append(get_outward_normal(point1, point2))
            self.all_hull_normal_vectors.append(np.array(this_normal_vectors))
            self.all_hull_faces.append(np.array(this_hull_vertices))
        return 
    
    def interoplate_hull_faces(self, angle_step=1):
        """ Interpolate the hull faces with small faces."""
        self.all_hull_small_faces = []
        self.all_approx_visible_region = []
        for r in range(len(self.all_hull_faces)):
            if not isinstance(self.all_hull_faces[r], np.ndarray):
                print(type(self.all_hull_faces[r]))
                raise TypeError("all_hull_faces should be a numpy array.")
            
            robot_hull_faces = self.all_hull_faces[r]
            # print(f"hull shapes: {robot_hull_faces.shape}")
            hull_small_faces = construct_hull_with_small_faces(robot_hull_faces, angle_step=angle_step)
            approx_visible_region = flip_vertices_of_hull_faces(hull_small_faces, self.flip_radius)
            self.all_hull_small_faces.append(hull_small_faces)
            self.all_approx_visible_region.append(approx_visible_region)
        return
    
    def update_dists_to_hull_faces(self, flipped_convexhull: np.ndarray, distance_matrix: np.ndarray) -> List[dict]:
        """ Get los_distance of each robot to all its neighboring (according to distance) robot's visible region."""
        # 1. for each robot, transform all other robots into its local frame;
        # 2. do shperical flipping
        # 3. get the los_distance list, and store
        number_robot = distance_matrix.shape[0]
        self.pairwise_maxDist_to_hull: List[dict] = []  # 保存其它机器人到this robot的convexhull的每条边的距离的最大值
        for r in range(number_robot):
            self.pairwise_maxDist_to_hull.append({})
            r_neighbors = self.find_neighbors_in_comm_range(r, distance_matrix)
            for j in r_neighbors:
                # Transform position of robot j into robot i's local frame
                pose_j_local = transform_to_local(self.robot_positions[j], self.robot_positions[r])
                flipped_rPose = np.array(spherical_flipping(pose_j_local[:self.dim], self.flip_radius))
                diff = flipped_rPose.reshape(1, -1) - self.all_hull_faces[r][:, 0]
                dists = np.sum(diff * self.all_hull_normal_vectors[r], axis=1)
                self.pairwise_maxDist_to_hull[r][j] = np.max(dists)
        return self.pairwise_maxDist_to_hull
    
    def update_obstacle_points(self, obstacle_points_local: List[list]):
        """ Transform local obstacle points into odom frame."""
        self.obstacle_points: List[tuple] = []
        for r in range(len(self.robot_positions)):
            obstacle_local = obstacle_points_local[r]
            yaw = self.robot_positions[r][2]
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            obstacle_odom_x = obstacle_local[0]*cos_yaw - obstacle_local[1]*sin_yaw + self.robot_positions[r][0]
            obstacle_odom_y = obstacle_local[0]*sin_yaw + obstacle_local[1]*cos_yaw + self.robot_positions[r][1]
            self.obstacle_points.append((obstacle_odom_x, obstacle_odom_y))

    def clear_obstacle_points(self):
        self.obstacle_points: List[tuple] = []

    ## Distance matrix
    def get_distance_matrix(self) -> np.ndarray:
        """ Return inter-robot distance matrix given all robots' current positions"""
        num_robots = len(self.robot_positions)
        distances = np.zeros((num_robots, num_robots))
        for i in range(num_robots):
            for j in range(i+1, num_robots):
                dist = math.sqrt((self.robot_positions[i][0] - self.robot_positions[j][0])**2 + \
                                 (self.robot_positions[i][1] - self.robot_positions[j][1])**2)
                distances[i, j] = dist
                distances[j, i] = dist
        return distances

    def find_neighbors_in_comm_range(self, id_this, distance_matrix: np.ndarray) -> list:
        """ Find this robot's neighbors within communication radius"""
        neighbor_ids = []
        for k in range(distance_matrix.shape[0]):
            if k == id_this:
                continue
            if distance_matrix[id_this, k] < self.d_comm_max:
                neighbor_ids.append(k)
        return neighbor_ids

    ## Distance derivative
    def get_partial_dij_qi(self, id_i: int, id_j: int, robot_positions: List[tuple], distance_matrix: np.ndarray) -> list:
        dij = distance_matrix[id_i, id_j]
        return [(robot_positions[id_i][0] - robot_positions[id_j][0])/dij, (robot_positions[id_i][1] - robot_positions[id_j][1])/dij]


    ## Constraints
    #---------------------------- 1. Communication constraints ----------------------------------#
    def get_gamma_ij(self, dij: float):
        """ Communication radius constraints"""
        if 0 <= dij <= self.d_comm_min:
            return self.k_gamma
        elif dij > self.d_comm_max:
            return 0
        else:
            return self.k_gamma/2*(1+math.cos((dij - self.d_comm_min)/(self.d_comm_max - self.d_comm_min)*math.pi))
        
    def get_partial_gamma_ij_d_ij(self, dij: float):
        """ Get derivativa of gamma_ij w.r.t. the d_ij"""
        if 0 <= dij <= self.d_comm_min or dij > self.d_comm_max:
            return 0
        else:
            return -self.k_gamma*math.pi/(2*(self.d_comm_max - self.d_comm_min))*math.sin((dij - self.d_comm_min)*math.pi/(self.d_comm_max-self.d_comm_min))
        
    def get_partial_gamma_ij_q_i(self, id_i: int, id_j: int, distance_matrix: np.ndarray) -> list:
        dij = distance_matrix[id_i, id_j]
        partial_gamma_ij_d_ij = self.get_partial_gamma_ij_d_ij(dij)
        partial_dij_qi = self.get_partial_dij_qi(id_i, id_j, self.robot_positions, distance_matrix)
        return [partial_gamma_ij_d_ij * partial_dij_qi[0], partial_gamma_ij_d_ij * partial_dij_qi[1]]


    #---------------------------- 2. Prefered inter-robot distance ----------------------------------#
    def get_beta_ij(self, dij: float):
        """ Prefered inter-robot constraints"""
        return self.k_beta*math.exp(-(dij - self.d_inter)**2/self.sigma)

    def get_partial_beta_ij_d_ij(self, dij: float):
        """ Derivative of prefered inter-robot distance constraints"""
        return -2*(dij - self.d_inter)*self.get_beta_ij(dij) / self.sigma

    #---------------------------- 3. Collision avoidance ----------------------------------------#
    def get_alpha_ij_star(self, dij: float):
        """ Get asymmetric definition of alpha_ij"""
        if dij <= self.d_coll_min:
            return 0
        elif self.d_coll_min < dij <= self.d_coll_max:
            return self.k_alpha/2 * (1 - math.cos(((dij - self.d_coll_min)/(self.d_coll_max - self.d_coll_min))*math.pi))
        else:
            return self.k_alpha
        
    def get_partial_alpha_ij_star_d_ij(self, dij: float):
        """ Get derivative of alpha_ij_star wrt d_ij"""
        if dij <= self.d_coll_min or dij > self.d_coll_max:
            return 0   
        return self.k_alpha*math.pi/(2*(self.d_coll_max - self.d_coll_min)) * math.sin((dij - self.d_coll_min)*math.pi/(self.d_coll_max - self.d_coll_min))

    def get_alpha_ij_star_obstacle(self, dij: float):
        """ Get asymmetric definition of alpha_ij"""
        if dij <= self.d_coll_obs_min:
            return 0
        elif self.d_coll_obs_min < dij <= self.d_coll_obs_max:
            return self.k_alpha_obs/2 * (1 - math.cos(((dij - self.d_coll_obs_min)/(self.d_coll_obs_max - self.d_coll_obs_min))*math.pi))
        else:
            return self.k_alpha_obs
        
    def get_partial_alpha_ij_star_d_ij_obstacle(self, dij: float):
        """ Get derivative of alpha_ij_star wrt d_ij"""
        if dij <= self.d_coll_obs_min or dij > self.d_coll_obs_max:
            return 0   
        return self.k_alpha_obs*math.pi/(2*(self.d_coll_obs_max - self.d_coll_obs_min)) * \
            math.sin((dij - self.d_coll_obs_min)*math.pi/(self.d_coll_obs_max - self.d_coll_obs_min))


    def get_alpha_ij(self, id_i: int, id_j: int, distance_matrix: np.ndarray):
        """ Edge weight for collision avoidance. Use symmetric definition of alpha_ij"""
        neighbors_i = self.find_neighbors_in_comm_range(id_i, distance_matrix)
        neighbors_j = self.find_neighbors_in_comm_range(id_j, distance_matrix)
        alpha_ij = 1
        for k in neighbors_i:
            d_ik = distance_matrix[id_i, k]
            alpha_ij *= self.get_alpha_ij_star(d_ik)
        for k in neighbors_j:
            if k == id_i:
                continue
            d_jk = distance_matrix[id_j, k]
            alpha_ij *= self.get_alpha_ij_star(d_jk)

        # Add obstalce points to avoide collision
        if len(self.obstacle_points) > 0:
            obstacle_i = self.obstacle_points[id_i]
            obstacle_j = self.obstacle_points[id_j]
            pose_i = self.robot_positions[id_i]
            pose_j = self.robot_positions[id_j]
            d_obs_i = math.sqrt((pose_i[0] - obstacle_i[0])**2 + (pose_i[1] - obstacle_i[1])**2)
            d_obs_j = math.sqrt((pose_j[0] - obstacle_j[0])**2 + (pose_j[1] - obstacle_j[1])**2)
            alpha_i_obs = self.get_alpha_ij_star_obstacle(d_obs_i)
            alpha_ij *= alpha_i_obs
            alpha_j_obs = self.get_alpha_ij_star_obstacle(d_obs_j)
            alpha_ij *= alpha_j_obs

        return alpha_ij

    def get_partial_alpha_ij_q_i(self, id_i: int, id_j: int, distance_matrix: np.ndarray) -> list:
        """ Note here the derivative is w.r.t. q_i, rather than d_ij
        Return: [partial_of_xi, partial_of_yi]
        """
        alpha_ij = self.get_alpha_ij(id_i, id_j, distance_matrix)
        if alpha_ij == 0:
            print("ERROR: the alpha_ij is zero, some robots collide with each other!!!")

        neighbors_i = self.find_neighbors_in_comm_range(id_i, distance_matrix)
        sum_neighbors = [0.0, 0.0]
        for k in neighbors_i:
            d_ik = distance_matrix[id_i, k]
            if self.get_alpha_ij_star(d_ik) == 0:  # FIXME: This means the two robot's has collided with each other
                return sum_neighbors
            else:
                coeff_k = 1/self.get_alpha_ij_star(d_ik) * self.get_partial_alpha_ij_star_d_ij(d_ik)
                sum_neighbors[0] += coeff_k * (self.robot_positions[id_i][0] - self.robot_positions[k][0]) / d_ik
                sum_neighbors[1] += coeff_k * (self.robot_positions[id_i][1] - self.robot_positions[k][1]) / d_ik

        # Add derivative caused by the obstacle points
        if len(self.obstacle_points) > 0:
            obstacle_i = self.obstacle_points[id_i]
            pose_i = self.robot_positions[id_i]
            d_obs_i = math.sqrt((pose_i[0] - obstacle_i[0])**2 + (pose_i[1] - obstacle_i[1])**2)
            alpha_ij_obs = self.get_alpha_ij_star_obstacle(d_obs_i)
            if alpha_ij_obs == 0:
                print(f"\033[91m Robot {id_i+1} collide with obstacle!!\033[0m")
                raise ValueError(f"\033[91m Robot {id_i+1} collide with obstacle!! pose: {pose_i}, obstacle: {obstacle_i}\033[0m")
            else:
                coeff_k = 1/alpha_ij_obs * self.get_partial_alpha_ij_star_d_ij_obstacle(d_obs_i)
                sum_neighbors[0] += coeff_k * (self.robot_positions[id_i][0] - obstacle_i[0]) / d_obs_i
                sum_neighbors[1] += coeff_k * (self.robot_positions[id_i][1] - obstacle_i[1]) / d_obs_i

        sum_neighbors[0] *= alpha_ij
        sum_neighbors[1] *= alpha_ij
        return sum_neighbors

    #---------------------------- 4. Line-of-Sight constraints ----------------------------------#
    def get_recorded_los_dist(self, i: int, j: int) -> tuple:
        """ Record los-distance from robot i to robot j's visible region.
        If returned distance is -1, then it means the two robots may not be within each other's LoS. 

        Returned value: (d_los_ij, d_exact_los_ij, d_k)
        """
        if i not in self.pairwise_maxDist_to_hull[j]:
            return (-1, -1)
        else:
            d_los_ij = self.los_dist_record[(i, j)]
            return (d_los_ij, self.pairwise_maxDist_to_hull[j][i])


    def get_los_distance(self, this_robot: int, center_robot: int):
        """ Get the initial asymmetric LoS distance from this robot to center_robot's convexhull. """
        if this_robot not in self.pairwise_maxDist_to_hull[center_robot] or \
            self.pairwise_maxDist_to_hull[center_robot][this_robot] < 0:
            return -1
        if self.use_dk:
            return self.pairwise_maxDist_to_hull[center_robot][this_robot]

        pose_local_frame = transform_to_local(self.robot_positions[this_robot], self.robot_positions[center_robot])
        relative_pose = np.array(pose_local_frame[:2])  # only consider x, y, as yaw is not important for 360 degree laser

        hull_small_faces = np.array(self.all_hull_small_faces[center_robot])
        hull_flipped_small_faces = flip_vertices_of_hull_faces(hull_small_faces, self.flip_radius)
        los_dists, gradients = get_all_dist_and_gradient(hull_flipped_small_faces, relative_pose)
        approx_min_dist = np.min(los_dists)
        return approx_min_dist



    def symmetrize_los_distance(self, d_los_ij, d_los_ji):
        """ The LoS-distance fusion method."""
        # Method 1: use softmin, leads to poor coorperation
        # return -math.log(math.exp(-self.beta_relax*d_los_ij) + math.exp(-self.beta_relax*d_los_ji)) / self.beta_relax
        # Method 2: use weighted sum
        return min(d_los_ij, d_los_ji)
        # return (d_los_ij + self.constant_c)*d_los_ji / (d_los_ij + d_los_ji + 2*self.constant_c) +\
        #       (d_los_ji + self.constant_c)*d_los_ij / (d_los_ij + d_los_ji + 2*self.constant_c) 
    
    def potential_symmetric_los_dist(self, dij) -> float:
        """ The potential function for los-distance. """
        if dij < self.d_los_min:   # Also includes the case that lose line-of-sight
            return 0
        elif self.d_los_min <= dij < self.d_los_max:
            return self.k_omega/2 * (1 - math.cos(((dij - self.d_los_min)/(self.d_los_max - self.d_los_min))*math.pi)) 
        else:
            return self.k_omega
        
    def get_omega_ij(self, i: int, j: int, print_los_distance: bool = False) -> float:
        """ Get edges weights for line-of-sight constraints.
        Note: self.los_distances should have been set!
        New here means we use weighted_sum to combine d_los_ij and d_los_ji.
        """
        if j not in self.pairwise_maxDist_to_hull[i] or i not in self.pairwise_maxDist_to_hull[j]:
            omega_ij = 0
            self.my_print(f"{PURPLE}Robot {i+1}--{j+1} not in Line-of-Sight.{RESET}")
        else:
            d_los_ij = self.get_los_distance(i, j)
            d_los_ji = self.get_los_distance(j, i)
            self.los_dist_record[(i, j)] = d_los_ij
            self.los_dist_record[(j, i)] = d_los_ji

            if d_los_ji <= 0 or d_los_ij <= 0:  # This step is very important, because the fusion function cannot work when d_los_ji < 0
                omega_ij = 0
                self.my_print(f"{PURPLE}Robot {i+1}--{j+1} not in Line-of-Sight. ({d_los_ij} & {d_los_ji}){RESET}")
            else:
                symmetric_los_distance = self.symmetrize_los_distance(d_los_ij, d_los_ji)
                omega_ij = self.potential_symmetric_los_dist(symmetric_los_distance)
                if print_los_distance:
                    print(make_str_green(f"Robot {i+1} to {j+1} LoS: ") + make_str_green(f"dij: ") +\
                        f"{round(d_los_ij, 3):<7}, " + make_str_green(f"dji: ") + f"{round(d_los_ji, 3):<7}, " + \
                            make_str_green(f"final_dij: ") + f"{round(symmetric_los_distance, 3):<7}")
        return omega_ij
    

    def get_derive_losDist_to_qji(self, this_robot: int,  center_robot: int) -> np.ndarray:
        """ The derivative of los-distance w.r.t. dji, using definition of d*cos_theta_k. """
        pose_local_frame = transform_to_local(self.robot_positions[this_robot], self.robot_positions[center_robot])
        relative_pose = np.array(pose_local_frame[:2])  # only consider x, y, as yaw is not important for 360 degree laser

        if self.use_dk:
            flipped_rPose = np.array(spherical_flipping(relative_pose, self.flip_radius))
            diff = flipped_rPose.reshape(1, -1) - self.all_hull_faces[center_robot][:, 0]
            dists = np.sum(diff * self.all_hull_normal_vectors[center_robot], axis=1)
            idx_max = np.argmax(dists)
            normal_vector = self.all_hull_normal_vectors[center_robot][idx_max]
            # The derivative of flipped_qi w.r.t. qi
            dji_flipped_qji = normal_vector.reshape(-1, 1)
            qji = relative_pose.reshape(-1, 1)
            norm_qji = np.linalg.norm(qji)
            dji_qji = dji_flipped_qji.T * (2 * self.flip_radius / norm_qji - 1)
            dji_qji += -2*self.flip_radius/norm_qji**3 * (dji_flipped_qji.T @ (qji @ qji.T))
            return dji_qji/np.linalg.norm(dji_qji)
        
        else:
            hull_flipped_small_faces = self.all_approx_visible_region[center_robot]
            los_dists, gradients = get_all_dist_and_gradient(hull_flipped_small_faces, relative_pose)
            idx_min = np.argmin(los_dists)
            # add radial gradient component
            los_dist = los_dists[idx_min]
            potential = self.potential_symmetric_los_dist(los_dist)
            radial_gradient = -relative_pose/np.linalg.norm(relative_pose)
            final_gradient = gradients[idx_min] + potential/self.k_alpha*radial_gradient
            return final_gradient/np.linalg.norm(final_gradient)

    
    def get_derivative_omega_ij_to_q_ji(self, id_thisRobot: int,  id_centerRobot: int) -> tuple:
        """ Get partial derivative of omega_ij w.r.t. q_i.
        Use the symmetric definition of omega_ij, from weighted sum.
        """
        if (id_thisRobot, id_centerRobot) in self.omega_record:
            saved_key = (id_thisRobot, id_centerRobot)
        else:
            saved_key = (id_centerRobot, id_thisRobot)
        omega_ij = self.omega_record[saved_key]
        if omega_ij == 0 or omega_ij == self.k_omega:
            return np.zeros(2)
        
        # First term
        d_los_ji, d_los_ij = self.los_dist_record[(id_centerRobot, id_thisRobot)], self.los_dist_record[(id_thisRobot, id_centerRobot)]
        symmetric_los_distance = self.symmetrize_los_distance(d_los_ij, d_los_ji)
        partial_wij_to_symmetric_los_dist = self.k_omega*math.pi / (2*(self.d_los_max - self.d_los_min)) * math.sin((symmetric_los_distance - self.d_los_min)*math.pi/(self.d_los_max - self.d_los_min))
        # TODO: here I reset the second term as 1
        partial_symmetric_los_dist_to_dji = 2*(d_los_ji+self.constant_c)**2 / (d_los_ij+d_los_ji+2*self.constant_c)**2
        partial_symmetric_los_dist_to_dji = 1
        # Third term: the derivative of d_ji w.r.t. q_ji
        partial_dji_qji = self.get_derive_losDist_to_qji(id_thisRobot, id_centerRobot) 
        
        partial_omega_ij_qji = partial_wij_to_symmetric_los_dist * partial_symmetric_los_dist_to_dji * partial_dji_qji
        partial_omega_ij_qji = partial_omega_ij_qji.reshape(-1)

        return (partial_omega_ij_qji[0], partial_omega_ij_qji[1])
    
    ## Generalized graph connectivity
    def get_minimum_topology(self, 
                             distance_matrix: np.ndarray, 
                             add_communication: bool, 
                             add_collision: bool, 
                             add_los: bool, 
                             enable: bool = True) -> Tuple[bool, Set[tuple]]:
        """ Get the minimum edges that keep connectivity. The edge weight recorders are also updated."""
        weighted_graph = GraphMST(distance_matrix.shape[0])
        graph_neighbors = [{} for _ in range(distance_matrix.shape[0])]
        all_edges = set()
        for i in range(distance_matrix.shape[0]):
            for j in range(i+1, distance_matrix.shape[0]):
                dij = distance_matrix[i][j]
                gamma_ij = self.get_gamma_ij(dij)  # communication radius weight
                alpha_ij = self.get_alpha_ij(i, j, distance_matrix)  # collision avoidance weight
                omega_ij = self.get_omega_ij(i, j, print_los_distance=False)   # line-of-sight constraints weight

                w_ij = 1
                if add_collision:
                    w_ij *= alpha_ij
                if add_communication:
                    w_ij *= gamma_ij
                if add_los:
                    w_ij *= omega_ij
                if w_ij != 0:  # Only selec topology from existing connectivity graph
                    # TODO: add dist to this weight
                    weighted_graph.addEdge(i, j, -gamma_ij * omega_ij + dij/self.d_comm_max)  
                    all_edges.add((i, j))
                    graph_neighbors[i][j] = w_ij
                    graph_neighbors[j][i] = w_ij

                # Save edge weigths for reuse
                self.gamma_record[(i, j)] = gamma_ij
                self.alpha_record[(i, j)] = alpha_ij
                self.omega_record[(i, j)] = omega_ij
                self.combined_weight_record[(i, j)] = w_ij
        # Check if the commuication graph is connected
        graph_connected = weighted_graph.is_graph_connected()
        self.num_graph_edges = len(weighted_graph.edges)
        if enable:
            # Enhance MST selection
            selected_edges = set(weighted_graph.KruskalMST())
            # 1. Count degree of each node in the selected edges
            degree_count = [0 for _ in range(distance_matrix.shape[0])]
            for i, j in selected_edges:
                degree_count[i] += 1
                degree_count[j] += 1
            # 2. Add additional edge if the degree of any weakly connected node is 1
            add_edges = set()
            for i, j in selected_edges:
                if i > j:
                    i, j = j, i
                if self.combined_weight_record[(i, j)] < 0.1:
                    if degree_count[i] == 1:
                        for neighbor in graph_neighbors[i]:
                            if neighbor != j and (i, neighbor) not in add_edges and (neighbor, i) not in add_edges:
                                add_edges.add((i, neighbor))
                    if degree_count[j] == 1:
                        for neighbor in graph_neighbors[j]:
                            if neighbor != i and (j, neighbor) not in add_edges and (neighbor, j) not in add_edges:
                                add_edges.add((j, neighbor))
            selected_edges.update(add_edges)
            return (graph_connected, selected_edges)
        return (graph_connected, all_edges)


    def get_masked_generalized_laplacian_matrix(self, distance_matrix: np.ndarray, 
                                         minimum_edges: Set[tuple]) -> Tuple[np.ndarray]:
        """ 
        Normal_vectors of convexhull, and los distance has been updated before calling this function. 
        """
        laplacian_matrix = np.zeros(distance_matrix.shape)
        actual_laplacian_matrix = np.zeros(distance_matrix.shape)
        for i in range(laplacian_matrix.shape[0]):
            for j in range(i+1, laplacian_matrix.shape[0]):
                if (i, j) in self.combined_weight_record:
                    saved_key = (i, j)
                else:
                    saved_key = (j, i)
                w_ij = self.combined_weight_record[saved_key]

                actual_laplacian_matrix[i][j] = -w_ij
                actual_laplacian_matrix[j][i] = -w_ij
                
                if (i, j) not in minimum_edges and (j, i) not in minimum_edges and w_ij > 0:
                    # Only maintain the minimum edges if other edges are well-connected
                    if self.alpha_record[saved_key] < 0.8:
                        w_ij = self.alpha_record[saved_key]   # only collision avoidance is maintained
                    else:
                        w_ij = 0

                self.my_print(make_str_green(f"Robot {i+1} to {j+1} w_ij: ") + f"{round(w_ij, 4)}")
                self.my_print("    gamma: " + f"{round(self.gamma_record[saved_key], 4):<6}   " + \
                      "alpha: "+ f"{round(self.alpha_record[saved_key], 4):<6}   " + \
                      "omega: " + f"{round(self.omega_record[saved_key], 4):<6}")
                
                laplacian_matrix[i][j] = -w_ij
                laplacian_matrix[j][i] = -w_ij
        for i in range(laplacian_matrix.shape[0]):
            laplacian_matrix[i][i] = -1 * np.sum(laplacian_matrix[i, :])
            actual_laplacian_matrix[i][i] = -1 * np.sum(actual_laplacian_matrix[i, :])
        return laplacian_matrix, actual_laplacian_matrix
            
    def get_second_eigenvalue_eigenvector(self, laplacian_matrix: np.ndarray) -> Tuple[float, list]:
        eigenvalues, eigenvectors = LA.eig(laplacian_matrix)
        if np.iscomplex(eigenvalues.any()):
            print("Error: Complex Root")
        eigenvalue_sort = []
        for i, value in enumerate(eigenvalues):
            eigenvalue_sort.append((value, i))
        eigenvalue_sort.sort()
        
        lambda2_idx = eigenvalue_sort[1][1]
        return (np.real(eigenvalues[lambda2_idx]), np.real(list(eigenvectors[:, lambda2_idx])))


    ## Generalized potential force
    def get_f_lambda(self, lambda2: float):
        """ Potential function for lambda_2"""
        return 2/(lambda2 - self.prefer_lambda2)
    
    def get_derivative_f_lambda(self, lambda2: float):
        return 2/(lambda2 - self.prefer_lambda2)**2
    
    def get_masked_gradient(self, id_this: int, lambda2: float, eigen_vector2: np.ndarray, distance_matrix: np.ndarray, minimum_edges: Set[tuple]) -> Tuple[float]:
        left_term = self.get_derivative_f_lambda(lambda2)
        sum_term = np.zeros(2)

        neighbors_i = self.find_neighbors_in_comm_range(id_this, distance_matrix)
        for j in neighbors_i:
            dij = distance_matrix[id_this, j]
            right_square_term = (np.real(eigen_vector2[id_this]) - np.real(eigen_vector2[j]))**2

            if (id_this, j) in self.combined_weight_record:
                saved_key = (id_this, j)
            else:
                saved_key = (j, id_this)

            # Edge weights
            alpha_ij = self.alpha_record[saved_key]  # collsion avoidance
            gamma_ij = self.gamma_record[saved_key]  # communication radius
            omega_ij = self.omega_record[saved_key]  # LoS constraints
            
            # if id_this == 1:
            #     print(f"Robot {id_this+1}-{j+1}: alpha: {alpha_ij}, gamma: {gamma_ij}, omega: {omega_ij}")

            derivative1 = np.array(self.get_partial_alpha_ij_q_i(id_this, j, distance_matrix))
            derivative1 *= gamma_ij * omega_ij

            derivative2 = np.array(self.get_partial_gamma_ij_q_i(id_this, j, distance_matrix))
            derivative2 *= alpha_ij * omega_ij
            
            if omega_ij == 0:
                derivative3 = np.zeros(2)
            else:
                derivative3_local = self.get_derivative_omega_ij_to_q_ji(id_this, j)
                # Transform this derivative back to robot r's local frame
                derivative3 = np.array(transform_to_original(self.robot_positions[j], self.robot_positions[id_this], derivative3_local))
            derivative3 *= alpha_ij * gamma_ij

            # if id_this == 1:
            #     print(f"Robot {id_this+1}-{j+1}: derivative1: {derivative1}, derivative2: {derivative2}, derivative3: {derivative3}")
            #     print(f"Position: robot {id_this+1}: {self.robot_positions[id_this]}, robot {j+1}: {self.robot_positions[j]}")

            # More derivatives can be added here...
            if (id_this, j) in minimum_edges or (j, id_this) in minimum_edges:
                sum_term += (derivative1 + derivative2 + derivative3) * right_square_term
            else:
                sum_term += derivative1 * right_square_term
        
        sum_term *= left_term
        return sum_term[0], sum_term[1] 
    

if __name__ == "__main__":
    general_lp = GeneralizedLaplacian()
    pose1 = (0, 0, 0)
    pose2 = (2, 2, 2*math.pi/3)
    vector = (1, 0)
    vector_original = transform_to_original(pose2, pose1, vector)
    print(vector_original)