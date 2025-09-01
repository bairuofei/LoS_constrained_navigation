import rospy
import math


class LeadingTimer:
    """ A timer to check whether a robot moves sufficient distance after a period of time.
    Used to avoid stuck of robots.
    """
    def __init__(self, stuck_time_threshold = 5, move_efficiency_threshold = 1):
        self.stuck_threshold = stuck_time_threshold
        self.move_efficiency_threshold = move_efficiency_threshold
        self.time = rospy.Time.now()
        self.position = (0, 0)
    
    def start(self, position: tuple):
        self.time = rospy.Time.now()
        self.position = position  # Set robot's starting position
        self.efficiency_start_time = self.time
        self.efficiency_position = position
        self.this_leading_robot_inefficient = False

    def check_move_efficiency(self, position: tuple, leading_robot_id: int):
        """ Check the move efficiency of leading robots"""
        if rospy.Time.now() - self.efficiency_start_time < rospy.Duration(self.move_efficiency_threshold):
            return True
        if math.sqrt((self.efficiency_position[0] - position[0])**2 + (self.efficiency_position[1] - position[1])**2) < 0.3:
            self.this_leading_robot_inefficient = True
            self.leading_robot = leading_robot_id
            return False
        else:
            self.efficiency_start_time = rospy.Time.now()
            self.efficiency_position = position
            return True

            # if self.this_leading_robot_inefficient and leading_robot_id == self.leading_robot:
            #     return False
            # else:  # leading robot变化，或者之前没有inefficient
            #     self.efficiency_start_time = rospy.Time.now()
            #     self.efficiency_position = position
            #     return True
    
    def is_leading_robot_moving(self, position: tuple):
        """ Check if leading robot stucked here for a predefined period. """
        if rospy.Time.now() - self.time < rospy.Duration(self.stuck_threshold):
            return True
        if math.sqrt((self.position[0] - position[0])**2 + (self.position[1] - position[1])**2) < 0.1:
            return False
        else:
            self.time = rospy.Time.now()
            self.position = position  # Reset current position every 10 seconds
            return True