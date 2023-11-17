import rospy
from geometry_msgs.msg import Pose

class OPS:
    def __init__(self):
        rospy.Subscriber("simple_map", list, self.simple_map_callback)
        rospy.Subscriber("estimated_pose", Pose, self.estimated_pose_callback) 
        rospy.Subscriber("known_objects", dict, self.known_objects_callback) # [("name", (x,y)),("name", (x,y))]
        rospy.Subscriber("target_object", str, self.target_object_callback)

        rospy.publisher("goal_position", Pose)

        self.simple_map = None
        self.estimated_pose = None
        self.known_objects = None
        self.target_object = None

    def simple_map_callback(self, simple_map):
        self.simple_map = simple_map
        self.calculate_long_term_goal()
        return None

    def estimated_pose_callback(self, estimated_pose):
        self.estimated_pose = estimated_pose
        self.calculate_long_term_goal()
        return None

    def known_objects_callback(self, known_objects):
        self.known_objects = known_objects
        self.calculate_long_term_goal()
        return None

    def target_object_callback(self, target_object):
        self.target_object = target_object
        self.calculate_long_term_goal()
        return None

    def calculate_long_term_goal(self):

        return None