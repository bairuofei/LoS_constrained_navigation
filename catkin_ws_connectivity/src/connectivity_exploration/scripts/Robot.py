import math

class Robot:
    def __init__(self, x=0, y=0, yaw=0, name="robot", target_tolerance = 1.5) -> None:
        self.pose = (x, y, yaw)
        self.name = name
        self.has_target = False
        self.target = (0, 0)
        self.target_tolerance = target_tolerance
        pass

    def get_name(self) -> str:
        return self.name
    
    def set_pose(self, pose: tuple):
        self.pose = pose

    def get_pose(self) -> tuple:
        return self.pose
    
    def set_target_tolerance(self, target_tolerance):
        self.target_tolerance = target_tolerance

    def get_target_tolerance(self):
        return self.target_tolerance
    
    def set_target(self, target: tuple, force_reset: bool = False):
        """ Update target, and set has_target to true if this is a new target."""
        if force_reset:
            self.has_target = True
            self.target = target
            return
        if self.target != target:
            self.has_target = True  # Only set has_target to true when there is a new target
        self.target = target
    
    def get_target(self) -> tuple:
        return self.target
    
    def clear_target(self):
        """ Set has_target to false. """
        self.has_target = False

    def set_waypoint(self, waypoint: tuple):
        self.waypoint = waypoint

    def get_waypoint(self) -> tuple:
        """ Return intermediate waypoint toward target. """
        return self.waypoint
        
    def has_a_target(self) -> bool:
        """ Check whether this robot has target."""
        return self.has_target
    
    def has_reach_target(self) -> bool:
        if not self.has_target:
            return False
        if math.sqrt((self.pose[0] - self.target[0])**2 + (self.pose[1] - self.target[1])**2) < self.target_tolerance:  # 1.5 for exploration; 0.2 for iot demo
            return True
        return False
    
    def has_reach_waypoint(self) -> bool:
        """ This function is to fix the bug of far_planner, that it cannot handle target points in obstacles. """
        if math.sqrt((self.pose[0] - self.waypoint[0])**2 + (self.pose[1] - self.waypoint[1])**2) < 0.1:  # 0.1
            return True
        return False
        
    def get_dist_to_target(self) -> float:
        return math.sqrt((self.pose[0] - self.target[0])**2 + (self.pose[1] - self.target[1])**2)