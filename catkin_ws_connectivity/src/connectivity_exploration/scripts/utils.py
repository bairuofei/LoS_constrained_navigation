import math
import rospy
import pickle
import time

from visualization_msgs.msg import Marker
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Point

import numpy as np

# Define the color code for green
RED = '\033[91m'
GREEN = '\033[92m'
PURPLE = '\033[95m'
# Define the color code for resetting the color to default
RESET = '\033[0m'


class Timer:
    def __init__(self):
        self.time_stamps = []
    
    def tic(self):
        self.time_stamps.append(time.time())

    def toc(self):
        if len(self.time_stamps) == 0:
            return 0
        return time.time() - self.time_stamps.pop()


def save_data(data, file_path):
    """ Save data to file with suffix .pkl"""
    file = open(file_path, 'wb')
    pickle.dump(data, file)
    file.close()
    return True

def read_data(file_path):
    with open(file_path, 'rb') as file:
        data =pickle.load(file)
        return data
    

def make_str_green(message: str) -> str:
    return f"{GREEN}{message}{RESET}"

def make_str_red(message: str) -> str:
    return f"{RED}{message}{RESET}"


## LoS constraints
def transform_to_local(pose1: tuple, pose2: tuple) -> tuple:
    """ Tranform pose1 to pose2's local frame"""
    x1, y1, yaw1 = pose1
    x2, y2, yaw2 = pose2

    # 计算 pose1 相对于 pose2 的坐标
    dx = x1 - x2
    dy = y1 - y2

    # 旋转角度为 -yaw2 (从全局坐标系到局部坐标系)
    theta = -yaw2
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    # 使用旋转矩阵转换坐标
    # [cos_theta  -sin_theta]  *  [dx
    # [sin_theta  cos_theta]       dy]

    local_x = cos_theta * dx - sin_theta * dy
    local_y = sin_theta * dx + cos_theta * dy
    local_yaw = yaw1 - yaw2

    # limited yaw to [-pi, pi]
    if local_yaw <= -math.pi:
        local_yaw += 2 * math.pi
    elif local_yaw >= math.pi:
        local_yaw -= 2 * math.pi

    return local_x, local_y, local_yaw

def get_outward_normal(point1: tuple, point2: tuple) -> tuple:
    """ Get the outward normal vector of the line that connects point1 and point2."""
    a = point2[1] - point1[1]
    b = -(point2[0] - point1[0])
    norm = math.sqrt(a**2 + b**2)
    cos_theta = point1[0] * a + point1[1] * b
    if cos_theta < 0:
        return (-a/norm, -b/norm)
    return (a/norm, b/norm)

def spherical_flipping(point, flip_radius: float) -> tuple:
    """ Note the point should be transoformed into local coordinate frame before flipping."""
    norm = math.sqrt(point[0]**2 + point[1]**2)
    return 2*flip_radius*point[0]/norm - point[0], 2*flip_radius*point[1]/norm - point[1]  


def transform_to_original(pose_local: tuple, pose_original: tuple, vector_local: list):
    """Transform vector local from pose_local frame to pose_original frame"""
    yaw1 = pose_local[2]
    yaw2 = pose_original[2]
    yaw_diff = yaw1 - yaw2
    cos_yaw = math.cos(yaw_diff)
    sin_yaw = math.sin(yaw_diff)

    derivative_x = cos_yaw * vector_local[0] - sin_yaw * vector_local[1]
    derivative_y = sin_yaw * vector_local[0] + cos_yaw * vector_local[1]
    return [derivative_x, derivative_y]


def weight_to_rgb(weight: float) -> tuple:
    """Red -- Yellow -- Green -- Blue, as weights approach 1
    """
    if weight > 1:
        weight = 1
    elif weight < 0:
        weight = 0
    if weight < 0.5:  # 从红色(1, 0, 0)到黄色(1, 1, 0)的渐变
        r = 1.0
        g = 2 * weight  # green 从 0 到 1
        b = 0.0
    else:  # 从黄色(1, 1, 0)到蓝色(0, 0, 1)的渐变
        r = 2 * (1 - weight)  # red 从 1 到 0
        g = 1.0
        b = 1 * (weight - 0.5)  # blue 从 0 到 1
    
    return r, g, b


#-------------------------- Create RVIZ marker -----------------------------------#
def create_arrow_marker(id: int, position: np.ndarray, force: np.ndarray, color: tuple):
    """ Create marker to be published to rviz to show the direction of derivatives for robots."""
    marker = Marker()
    marker.header.frame_id = "map"  # 使用地图坐标系
    marker.header.stamp = rospy.Time.now()
    marker.ns = "arrows"  # 命名空间
    marker.id = id  # 唯一标识符
    marker.type = Marker.ARROW  # 设置为箭头类型
    marker.action = Marker.ADD  # 添加到 rviz 中
    marker.pose.position.x = position[0]  # 设置位置
    marker.pose.position.y = position[1]
    marker.pose.position.z = 0

    if force[0] == 0:
        z_angle = math.pi
    else:
        z_angle = math.atan2(force[1], force[0])
    quaternion_arrow = quaternion_from_euler(0, 0, z_angle) 
    marker.pose.orientation.x = quaternion_arrow[0]
    marker.pose.orientation.y = quaternion_arrow[1]
    marker.pose.orientation.z = quaternion_arrow[2]
    marker.pose.orientation.w = quaternion_arrow[3]

    # 设置箭头的方向
    marker.scale.x = 20 * np.linalg.norm(force)  # 箭头的宽度
    marker.scale.y = 0.2  # 箭头的尾部宽度
    marker.scale.z = 0.2 # 500 *np.linalg.norm(force)  # 箭头的长度

    # 设置箭头的颜色
    marker.color.r = color[0]  # 颜色信息
    marker.color.g = color[1]  # 颜色信息
    marker.color.b = color[2]  # 颜色信息
    marker.color.a = 1.0  # 完全不透明

    return marker


def create_circle_marker(id: int, center: list, radius: float, color: tuple):
    """ Create marker to be published to rviz to show the direction of derivatives for robots."""
    marker = Marker()
    marker.header.frame_id = "map"  # 使用地图坐标系
    marker.header.stamp = rospy.Time.now()
    marker.ns = "circles"  # 命名空间
    marker.id = id  # 唯一标识符
    marker.type = Marker.LINE_STRIP # 设置为箭头类型
    marker.action = Marker.ADD  # 添加到 rviz 中

    marker.scale.x = 0.1  #
    marker.color.a = 0.1  # 透明度
    marker.color.r = color[0]  # 红色
    marker.color.g = color[1]  # 红色
    marker.color.b = color[2]  # 红色

    # 设置圆心位置
    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]
    marker.pose.orientation.w = 1.0  # 无旋转


    for i in np.arange(0, math.pi * 2, 0.1):
        p = Point()
        p.x = radius * math.cos(i)
        p.y = radius * math.sin(i)
        p.z = 0
        marker.points.append(p)

    return marker


# Debug los-distance
def interpolate_points(pt1, pt2, num_steps) -> np.ndarray:
    point1 = np.array(pt1)
    point2 = np.array(pt2)

    # 计算两点之间的欧几里得距离
    distance = np.linalg.norm(point2 - point1)

    # 计算方向向量并标准化
    direction = (point2 -point1) / distance
    step = round(distance / num_steps, 6)
    if step == 0:
        return np.array([point1, point2])

    inter_points = [np.array((point1[0] + direction[0] * step * i, point1[1] + direction[1] * step * i)) for i in range(num_steps)]
    inter_points.append(point2)
    return np.array(inter_points)


def cartesian_to_polar(x: float, y: float):
    """
    将二维点从笛卡尔坐标系转换为极坐标形式。
    
    参数:
        x: 点的x坐标
        y: 点的y坐标
    
    返回:
        (rho, theta): 极坐标表示，其中 rho 为半径，theta 为角度（弧度）
    """
    rho = np.sqrt(x**2 + y**2)
    new_x = x
    if abs(x) < 1e-6:
        new_x = 0
    try:
        theta = np.arctan2(y, new_x)
    except Exception as e:
        print(f"cartesian_to_polar ValueError: {e}")
        print(f"x: {x}, y: {y}")
        raise False
    return rho, theta


def construct_hull_with_small_faces(hull_faces: np.ndarray, angle_step) -> np.ndarray:
    if not isinstance(hull_faces, np.ndarray):
        raise TypeError("hull_faces should be a numpy array.")
    
    hull_small_faces = []
    for k in range(hull_faces.shape[0]):
        face = hull_faces[k]
        rho1, theta1 = cartesian_to_polar(face[0][0], face[0][1])
        rho2, theta2 = cartesian_to_polar(face[1][0], face[1][1])
        if not isinstance(theta1, float) or not isinstance(theta2, float):
            print(f"theta1: {theta1}, type: {type(theta1)}")
            print(f"theta2: {theta2}, type: {type(theta2)}")
        
        try:
            angle_diff = min(abs(theta1 - theta2), 2*np.pi - abs(theta1 - theta2))
        except Exception as e:
            print(f"ValueError: {e}")
            print(f"theta1: {theta1}, type: {type(theta1)}")
            print(f"theta2: {theta2}, type: {type(theta2)}")
            if isinstance(theta1, np.ndarray):
                print(f"theta1.shape: {theta1.shape}, theta1 content: {theta1}")
            if isinstance(theta2, np.ndarray):
                print(f"theta2.shape: {theta2.shape}, theta2 content: {theta2}")
            raise  # 重新抛出异常，以便进一步调试
        
        # angle_diff = min(abs(theta1 - theta2), 2*np.pi - abs(theta1 - theta2))
        num_steps = int(math.floor(angle_diff / (angle_step/180*np.pi)))
        if num_steps > 1:
            small_face_vertices = interpolate_points(face[0], face[1], num_steps)
            for j in range(small_face_vertices.shape[0]-1):
                hull_small_faces.append(np.array([small_face_vertices[j], small_face_vertices[j+1]]))
        else:
            hull_small_faces.append(face)
    return np.array(hull_small_faces)

def flip_vertices_of_hull_faces(hull_faces: np.ndarray, flip_radius: float) -> np.ndarray:
    return 2*flip_radius*hull_faces/np.linalg.norm(hull_faces, axis=2, keepdims=True) - hull_faces

def get_all_dist_and_gradient(faces: np.ndarray, robot_pose: np.ndarray):
    """ faces: (n, 2); robot_pose: (2,) """
    dim = len(faces[0][0])
    X0 = robot_pose[:dim].reshape(1, -1)  # (1, 2)
    A = faces[:, 0] - faces[:, 1]   # (n, 2)
    B = X0 - faces[:, 1]  # (n, 2)
    t = np.sum(A * B, axis=1) / np.sum(A * A, axis=1) # (n,)  q到线段上的投影

    mask_t_le_0 = t <= 0   # 把向量朝边投影，如果比边还长，或者在边的负方向，则表明该点在边外
    mask_t_ge_1 = t >= 1
    mask_t_between = (t > 0) & (t < 1)
    
    los_dist = np.zeros(len(faces))
    gradient = np.zeros((len(faces), dim))

    if np.any(mask_t_le_0):
        los_dist[mask_t_le_0] = np.linalg.norm(X0 - faces[mask_t_le_0, 1], axis=1)
        gradient[mask_t_le_0] = (1 / (2 * los_dist[mask_t_le_0]))[:, None] * 2 * (X0 - faces[mask_t_le_0, 1])

    if np.any(mask_t_ge_1):
        los_dist[mask_t_ge_1] = np.linalg.norm(X0 - faces[mask_t_ge_1, 0], axis=1)
        gradient[mask_t_ge_1] = (1 / (2 * los_dist[mask_t_ge_1]))[:, None] * 2 * (X0 - faces[mask_t_ge_1, 0])
    
    if np.any(mask_t_between):
        A_bet = A[mask_t_between]
        B_bet = B[mask_t_between]
        los_dist[mask_t_between] = np.sqrt(-np.sum(A_bet * B_bet, axis=1)**2/np.sum(A_bet * A_bet, axis=1) + np.sum(B_bet * B_bet, axis=1))
        # Note: derive the gradient following the above expression of los_dist, rather than using the equation in the paper
        gradient[mask_t_between] = (1 / (2 * los_dist[mask_t_between]))[:, None] * (-np.sum(A_bet * B_bet, axis=1)[:, None] / np.sum(A_bet * A_bet, axis=1)[:, None] * 2 * A_bet + 2 * B_bet)
    return los_dist, gradient


def my_print(message: str, enable_print: bool = True):
    if enable_print:
        print(message)