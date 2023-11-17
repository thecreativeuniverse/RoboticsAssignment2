import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import numpy as np
from srg import SRG
import object_locator
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class OPS:
    def __init__(self):
        rospy.Subscriber("simple_map", String,
                         self.simple_map_callback)  # [[(5,-5),(5,-4),(5,-3),(5,-3),(5,-2),(5,-1),(5,0),(5,1),(5,2),(5,3),(5,4),(5,5)], [(4,-5), ....]
        rospy.Subscriber("estimated_pose", Pose, self.estimated_pose_callback)  # (0,0)
        rospy.Subscriber("known_objects", String, self.known_objects_callback)  # [("name", (x,y)),("name", (x,y))]
        rospy.Subscriber("target_object", String, self.target_object_callback)  # name

        self.goal_pos_pub = rospy.Publisher("goal_position", String, queue_size=10)

        # SRG
        self.srg = SRG()  # TODO
        # {"thing":{"thing2":2, "thing3":5}, "thing2":{"thing":2,"thing3":10}, "thing3":{"thing":5,"thing2":10}}

        self.simple_map_radius = 250

        # generate pdf coords
        xs, ys = np.meshgrid(range(-self.simple_map_radius, self.simple_map_radius + 1),
                             range(-self.simple_map_radius, self.simple_map_radius + 1), indexing='xy')
        self.simple_map = np.array(list(zip(xs.ravel(), ys.ravel())), dtype='i4,i4').reshape(xs.shape)

        self.pdf_map = np.zeros([self.simple_map_radius * 2, self.simple_map_radius * 2])

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

        # known_objects = self.known_objects
        known_objects = [("oven", (-50, -50)), ("sofa", (70, 100)), ("kettle", (-150, 150))]

        simple_map = self.simple_map

        srg = self.srg
        # target_object = self.target_object
        target_object = "bed"
        known_object_locations = []

        for obj in known_objects:
            distance = srg.get_distance(obj[0], target_object)
            known_object_locations.append((obj[1], distance, 50))

        res = object_locator.calculate_likelihoods(simple_map, known_object_locations)

        # fig, ax = plt.subplots()
        #
        # x, y, z = [], [], []
        # for part in res:
        #     for tup in part:
        #         x.append(tup[0])
        #         y.append(tup[1])
        #         z.append(tup[2])
        #
        # ax.scatter(x, y, c=z, cmap='cool', marker='.')
        #
        # for name, (x,y) in known_objects:
        #     col = np.random.random(3,)
        #     ax.plot(x,y,color=col,marker='x', label=name)
        #     circle = patches.Circle((x,y), radius=srg.get_distance(name,target_object), facecolor='none', edgecolor=col)
        #     ax.add_patch(circle)
        #
        # ax.legend()
        # ax.axis('equal')
        # ax.set_xlim(-250,250)
        # ax.set_ylim(-250,250)
        # fig.savefig("thing.png")
        return res

    def _validate_subbed_vars(self):
        return self.target_object is not None and self.known_objects is not None and self.simple_map is not None


# debugging - not for main use
if __name__ == '__main__':
    rospy.init_node("ops")
    ops_locator = OPS()
    rospy.spin()