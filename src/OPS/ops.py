import rospy
from geometry_msgs.msg import Pose
import numpy as np


class OPS:
    def __init__(self):
        rospy.Subscriber("simple_map", list,
                         self.simple_map_callback)  # [[(5,-5),(5,-4),(5,-3),(5,-3),(5,-2),(5,-1),(5,0),(5,1),(5,2),(5,3),(5,4),(5,5)], [(4,-5), ....]
        rospy.Subscriber("estimated_pose", Pose, self.estimated_pose_callback)  # (0,0)
        rospy.Subscriber("known_objects", dict, self.known_objects_callback)  # [("name", (x,y)),("name", (x,y))]
        rospy.Subscriber("target_object", str, self.target_object_callback)  # name

        rospy.publisher("goal_position", Pose)

        # SRG
        self.srg = {} #TODO

        self.simple_map_radius = 250

        # generate pdf coords
        xs, ys = np.meshgrid(range(-self.simple_map_radius, self.simple_map_radius + 1),
                             range(-self.simple_map_radius, self.simple_map_radius + 1), indexing='ij')
        self.pdf_map = np.array(list(zip(xs.ravel(), ys.ravel())), dtype='i4,i4').reshape(xs.shape)

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
        if not self._validate_subbed_vars():
            return


    def _validate_subbed_vars(self):
        return self.target_object is not None and self.estimated_pose is not None and self.known_objects is not None and self.simple_map is not None

# debugging - not for main use
# if __name__ == '__main__':
#     ops_locator = OPS()
#     print(ops_locator.pdf_map)

