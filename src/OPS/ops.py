import rospy
from geometry_msgs.msg import Pose
import numpy as np
from srg import SRG
import object_locator
import matplotlib.pyplot as plt

class OPS:
    def __init__(self):
        #rospy.Subscriber("simple_map", list,
        #                 self.simple_map_callback)  # [[(5,-5),(5,-4),(5,-3),(5,-3),(5,-2),(5,-1),(5,0),(5,1),(5,2),(5,3),(5,4),(5,5)], [(4,-5), ....]
        #rospy.Subscriber("estimated_pose", Pose, self.estimated_pose_callback)  # (0,0)
        #rospy.Subscriber("known_objects", dict, self.known_objects_callback)  # [("name", (x,y)),("name", (x,y))]
        #rospy.Subscriber("target_object", str, self.target_object_callback)  # name

        #rospy.publisher("goal_position", Pose)

        # SRG
        self.srg = SRG() #TODO
        # {"thing":{"thing2":2, "thing3":5}, "thing2":{"thing":2,"thing3":10}, "thing3":{"thing":5,"thing2":10}}

        self.simple_map_radius = 50

        # generate pdf coords
        xs, ys = np.meshgrid(range(-self.simple_map_radius, self.simple_map_radius + 1),
                             range(-self.simple_map_radius, self.simple_map_radius + 1), indexing='xy')
        self.simple_map = np.array(list(zip(xs.ravel(), ys.ravel())), dtype='i4,i4').reshape(xs.shape)
        print(self.simple_map)

        self.pdf_map = np.zeros([self.simple_map_radius * 2,self.simple_map_radius * 2])

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
        #if not self._validate_subbed_vars():
        #    return

        #known_objects = self.known_objects
        # known_objects = [("oven", (5,-10)), ("sofa", (10, 30))]
        known_objects = [("oven", (5,10))]
        simple_map = self.simple_map
        srg = self.srg
        #target_object = self.target_object
        target_object = "bed"
        known_object_locations = []

        for object in known_objects:
            # distance = srg.get_distance(object[0], target_object)
            distance = 5
            known_object_locations.append((object[1], distance, 2))

        res = object_locator.calculate_likelihoods(simple_map, known_object_locations)

         # Create and plot multivariate normal distribution
        x = np.random.uniform(-15, 15, size=10000).T
        y = np.random.uniform(-15, 15, size=10000).T

         # ((x,y), dist, var)
        #known_obj_locs_and_distributions = [((-6, 0), 5, 1), ((3, 2), 8, 3), ((1, -4), 6, 2)]

        # xy = zip(x, y)
        #res = calculate_likelihoods(xy, known_obj_locs_and_distributions)

        # Plot z
        # x, y, z = [zip(*part) for part in res]
        fig, ax = plt.subplots()
        for part in res:
            x,y,z = zip(*part)
            ax.scatter(x, y, c=z, cmap='cool', marker='.')

        fig.savefig("thing.png")

        return res


    def _validate_subbed_vars(self):
        return self.target_object is not None and self.estimated_pose is not None and self.known_objects is not None and self.simple_map is not None

# debugging - not for main use
if __name__ == '__main__':
    ops_locator = OPS()
    res = ops_locator.calculate_long_term_goal()

