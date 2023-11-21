#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import numpy as np
from srg import SRG, TrainingSRG
import object_locator
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class OPS:
    def __init__(self):
        rospy.Subscriber("map", OccupancyGrid,
                         self.simple_map_callback)  # [[(5,-5),(5,-4),(5,-3),(5,-3),(5,-2),(5,-1),(5,0),(5,1),(5,2),(5,3),(5,4),(5,5)], [(4,-5), ....]
        rospy.Subscriber("known_objects", String, self.known_objects_callback)  # [("name", (x,y)),("name", (x,y))]
        rospy.Subscriber("target_object", String, self.target_object_callback)  # name

        self.goal_pos_pub = rospy.Publisher("goal_position", String, queue_size=1)

        self.estimated_pose = None
        self.known_objects = None
        self.target_object = None

        # SRG
        self.srg = SRG()  # TODO
        # {"thing":{"thing2":2, "thing3":5}, "thing2":{"thing":2,"thing3":10}, "thing3":{"thing":5,"thing2":10}}

        self.simple_map_radius = 250

        # generate pdf coords
        xs, ys = np.meshgrid(range(-self.simple_map_radius, self.simple_map_radius + 1),
                             range(-self.simple_map_radius, self.simple_map_radius + 1), indexing='xy')
        self.simple_map = np.array(list(zip(xs.ravel(), ys.ravel())), dtype='i4,i4').reshape(xs.shape)

        self.pdf_map = np.zeros([self.simple_map_radius * 2, self.simple_map_radius * 2])

        self.HAS_DONE_ONE = False

    def simple_map_callback(self, simple_map):
        self.simple_map = simple_map
        self.calculate_long_term_goal()
        return None

    def estimated_pose_callback(self, estimated_pose):
        print("received") #debugging
        self.estimated_pose = estimated_pose
        self.calculate_long_term_goal()
        return None

    def known_objects_callback(self, known_objects):
        self.goal_pos_pub.publish(String(str(type(known_objects.data))))
        self.known_objects = eval(known_objects.data)
        self.calculate_long_term_goal()
        return None

    def target_object_callback(self, target_object):
        self.target_object = target_object
        self.calculate_long_term_goal()
        return None

    def calculate_long_term_goal(self):
        if not self._validate_subbed_vars():
            return

        data = np.array(self.simple_map.data)
        self.goal_pos_pub.publish(String(f"len data {len(data)}"))
        size = int(np.sqrt(len(data)))
        data_2d = np.split(data, size)
        self.goal_pos_pub.publish(String(f"shape new data {np.shape(data_2d)}"))

        known_objects = self.known_objects

        self.goal_pos_pub.publish(String(str(known_objects)))

        # simple_map = self.simple_map
        #
        # srg = self.srg
        # # target_object = self.target_object
        # target_object = "bed"
        # known_object_locations = []
        #
        # for obj in known_objects:
        #     distance = srg.get_distance(obj[0], target_object)
        #     known_object_locations.append((obj[1], distance, 50))
        #
        # res = object_locator.calculate_likelihoods(simple_map, known_object_locations)
        #
        # return res

    def train(self):

        target = "oven"

        print(self.srg.get_target_distribution(target))

        solid_known_objects = [("kettle", (-150, 150)), ("toaster", (-200, 180)), ("oven", (-200, -150)),
                               ("cushion", (80, -200)), ("sofa", (70, 100)), ("bed", (50, -76))]
        simple_map = self.simple_map

        colors = {}

        res = object_locator.calculate_likelihoods(simple_map, target, self.srg, solid_known_objects)
        colors = self._plot(res, solid_known_objects, target, f"train-first", colors)

        for i in range(10):
            known_objects = []
            for obj, (x, y) in solid_known_objects:
                x += np.random.normal(scale=25)
                y += np.random.normal(scale=25)
                known_objects.append((obj, (x, y)))

            estimated = known_objects.copy()
            del estimated[2]
            res = object_locator.calculate_likelihoods(simple_map, target, self.srg, estimated)
            colors = self._plot(res, solid_known_objects, target, f"train-{i}", colors)
            for j in range(len(known_objects)):
                obj1, (x1, y1) = known_objects[j]
                for k in range(j + 1, len(known_objects)):
                    obj2, (x2, y2) = known_objects[k]
                    distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                    self.srg.update_weights(obj1, obj2, distance)
            print(f"Iteration {i} done")

        known_objects = []
        for obj, (x, y) in solid_known_objects:
            x += np.random.normal(scale=50)
            y += np.random.normal(scale=50)
            known_objects.append((obj, (x, y)))

        res = object_locator.calculate_likelihoods(simple_map, target, self.srg, solid_known_objects)
        self._plot(res, solid_known_objects, target, f"train-final", colors)
        print("done")

        print(self.srg.get_target_distribution(target))

    def _plot(self, res, known_objects, target, figname, colors):
        fig, ax = plt.subplots()

        x, y, z = [], [], []
        for part in res:
            for tup in part:
                x.append(tup[0])
                y.append(tup[1])
                z.append(tup[2])

        highest_prob = max(z)
        highest_prob_index = z.index(highest_prob)
        highest_prob_coords = (x[highest_prob_index], y[highest_prob_index])


        ax.scatter(x, y, c=z, cmap='cool', marker='.')

        ax.plot(highest_prob_coords[0], highest_prob_coords[1], 'k.', markersize=5)

        for name, (x, y) in known_objects:
            if name not in colors:
                col = np.random.random(3, )
                colors.update({name: col})
            else:
                col = colors.get(name)
            ax.plot(x, y, color=col, marker='x', label=name)
            circle = patches.Circle((x, y), radius=self.srg.get_distance(name, target), facecolor='none',
                                    edgecolor=col)
            ax.add_patch(circle)

        ax.legend()
        ax.axis('equal')
        ax.set_xlim(-250, 250)
        ax.set_ylim(-250, 250)
        fig.savefig(f"{figname}.png")
        plt.close(fig)
        return colors

    def _validate_subbed_vars(self):
        return self.target_object is not None and self.known_objects is not None and self.simple_map is not None


# debugging - not for main use
if __name__ == '__main__':
    rospy.init_node('ops', anonymous=True)
    ops_locator = OPS()
    rospy.spin()
