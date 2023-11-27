#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Header
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Pose, Point, Quaternion
import numpy as np
from srg import SRG, TrainingSRG
import object_locator
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time


class OPS:
    def __init__(self):
        # initialising functions/variables
        rospy.Subscriber("target_object", String, self.target_object_callback)  # name
        rospy.Subscriber("map", OccupancyGrid,
                         self.simple_map_callback)  # [[(5,-5),(5,-4),(5,-3),(5,-3),(5,-2),(5,-1),(5,0),(5,1),(5,2),(5,3),(5,4),(5,5)], [(4,-5), ....]
        rospy.Subscriber("known_objects", String, self.known_objects_callback)  # [("name", (x,y)),("name", (x,y))]

        self.goal_pos_pub = rospy.Publisher("goal_position", PointCloud, queue_size=1)
        self.particle_cloud_pub = rospy.Publisher("goal_pos_estimation", PointCloud, queue_size=1)

        self.estimated_pose = None
        self.known_objects = None
        self.target_object = None

        # SRG
        self.srg = TrainingSRG() # Change depending on training
        # {"thing":{"thing2":2, "thing3":5}, "thing2":{"thing":2,"thing3":10}, "thing3":{"thing":5,"thing2":10}}

        self.simple_map_radius = 250

        # generate pdf coords
        xs, ys = np.meshgrid(range(-self.simple_map_radius, self.simple_map_radius + 1),
                             range(-self.simple_map_radius, self.simple_map_radius + 1), indexing='xy')
        self.simple_map = np.array(list(zip(xs.ravel(), ys.ravel())), dtype='i4,i4').reshape(xs.shape)

        self.pdf_map = np.zeros([self.simple_map_radius * 2, self.simple_map_radius * 2])

        self.particle_cloud = PointCloud()
        self.particle_cloud.header.frame_id = "map"
        self.particle_cloud.header.stamp = rospy.Time.now()

        self.pose_array_size = 200

        self.particle_cloud.points = [self.generate_pose() for _ in range(self.pose_array_size)]

        self.particle_cloud_pub.publish(self.particle_cloud)
        print("")

        self.ALREADY_CALCULATING = False

        time.sleep(60)
        self.train()

    def simple_map_callback(self, simple_map):
        self.simple_map = simple_map.data
        self.update_particle_cloud()

    def estimated_pose_callback(self, estimated_pose):
        self.estimated_pose = estimated_pose
        self.update_particle_cloud()

    def known_objects_callback(self, known_objects):
        self.known_objects = eval(known_objects.data)
        self.update_particle_cloud()

    def target_object_callback(self, target_object):
        self.target_object = target_object.data
        self.update_particle_cloud()

    def calculate_long_term_goal(self):
        if not self._validate_subbed_vars():
            "invalid"
            return
        if self.ALREADY_CALCULATING:
            return

        self.ALREADY_CALCULATING = True

        data = np.array(self.simple_map)
        size = int(np.sqrt(len(data)))
        simple_map = np.split(data, size)

        known_objects = self.known_objects
        print(f"we know {len(known_objects)} objects and are looking for {self.target_object}")

        srg = self.srg
        known_object_locations = []

        for obj in known_objects:
            distance = srg.get_distance(obj[0], self.target_object)
            known_object_locations.append((obj[1], distance, 50))

        res = object_locator.calculate_likelihoods(simple_map=simple_map, target=self.target_object, srg=srg,
                                                   known_obj_locs=known_object_locations)

        (x, y, prob) = max(res, key=lambda val: val[-1])

        target_pointcloud = PointCloud()
        # filling pointcloud header
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        target_pointcloud.header = header
        target_pointcloud.points.append(Point32(x / 20, y / 20, 0))

        self.goal_pos_pub.publish(target_pointcloud)
        self.ALREADY_CALCULATING = False

    def update_particle_cloud(self):
        self.particle_cloud_pub.publish(self.particle_cloud)
        if not self._validate_subbed_vars():
            "invalid"
            return
        if self.ALREADY_CALCULATING:
            return
        self.ALREADY_CALCULATING = True

        print("valid")
        # Get current poses
        initial_particles = self.particle_cloud.points

        # Calculate the weights of all particles and append them to an array, then calculate an average.
        # This will be used specifically to calulate how many particles to redraw.
        # particles_weights = [self.sensor_model.get_weight(scan, particle) for particle in initial_particles]

        data = np.array(self.simple_map)
        size = int(np.sqrt(len(data)))
        # simple_map = np.split(data, size)

        known_objects = self.known_objects

        srg = self.srg
        known_object_locations = []

        for obj in known_objects:
            distance = srg.get_distance(obj[0], self.target_object)
            known_object_locations.append((obj[1], distance, 50))

        particles_weights = [object_locator.get_weight(particle, self.target_object, srg, known_object_locations) for
                             particle in
                             initial_particles]

        sum_of_weights = sum(particles_weights)
        average_weight = sum_of_weights / len(particles_weights)

        # we vary the amount of particles based on how certain the algorithm is of the robot's position
        # particles_to_keep = round((100 / average_weight ** 0.5) - (average_weight / 10))
        particles_to_keep = len(initial_particles)  # TODO?

        # new_predicted_readings = max(20, round(10 * average_weight))
        # self.set_pose_array_size(particles_to_keep)
        # self.set_num_predicted_readings(new_predicted_readings)

        # Add 15% more particles in uniform random locations; this will allow us to find a more accurate estimation of
        # the robot's position in the event the current estimation is completely wrong
        initial_particles += [self.generate_pose() for _ in range(round(len(initial_particles) * 0.15))]

        # Recalculate the weights of all particles and append them to an array, then calculate an average
        # particles_weights = [self.sensor_model.get_weight(scan, particle) for particle in initial_particles]
        particles_weights = [object_locator.get_weight(particle, self.target_object, srg, known_object_locations) for
                             particle in
                             initial_particles]

        # if the algorithm needs to remove particles it will remove the particles with the lowest weight
        while len(particles_weights) > particles_to_keep:
            index = particles_weights.index((min(particles_weights)))
            del (particles_weights[index])
            del (initial_particles[index])

        # if the algorithm needs to generate particles it generates 50
        # particles and places the particle with the highest weight
        while len(particles_weights) < particles_to_keep:
            new_poses = []
            for i in range(50):
                new_pose = self.generate_pose()
                new_weight = object_locator.get_weight(new_pose, self.target_object, srg, known_object_locations)
                new_poses.append((new_pose, new_weight))
            new_poses = sorted(new_poses, key=lambda x: x[1])

            initial_particles.append(new_poses[-1][0])
            particles_weights.append(new_poses[-1][1])

        # derive the variance for random particle generation based on the updated weights for all the particles
        sum_of_weights = sum(particles_weights)
        average_weight = sum_of_weights / len(particles_weights)
        variance = (1 / average_weight)

        # normalise the particle weights
        particles_weights = [w / sum_of_weights for w in particles_weights]
        particles_kept = []
        current_cum_weight = particles_weights[0]
        cum_weights = [current_cum_weight]

        # calculate the cumulative weights for all particles
        for i in range(1, self.pose_array_size):
            cum_weights.append(cum_weights[i - 1] + particles_weights[i])

        # calculate the increment size and initial threshold
        tick_size = 1 / self.pose_array_size
        current_threshold = np.random.uniform(0, tick_size)

        # Safety in case it picks 0, shouldn't really enter this
        while current_threshold == 0:
            current_threshold = np.random.uniform(0, tick_size)

        # remove particles which don't have a high enough cumulative weight and replace them with variations of the
        # previous particle that did have a high enough cumulative weight
        i = 0
        for j in range(self.pose_array_size):
            while current_threshold > cum_weights[i]:
                i += 1

            particles_kept.append(self.generate_pose(pose=initial_particles[i], variance=variance))
            current_threshold += tick_size

        # update the particle cloud to have the new particles
        print("particles to keep", particles_to_keep)
        self.particle_cloud.points = particles_kept
        self.particle_cloud_pub.publish(self.particle_cloud)
        self.ALREADY_CALCULATING = False

    def train(self):
        solid_known_objects = self.known_objects
        simple_map = self.simple_map

        print(solid_known_objects)

        if solid_known_objects == None:
            return

        for target, coords in solid_known_objects:
            # print(self.srg.get_target_distribution(target))

            for i in range(10):
                known_objects = []
                for obj, (x, y) in solid_known_objects:
                    x += np.random.normal(scale=25)
                    y += np.random.normal(scale=25)
                    known_objects.append((obj, (x, y)))

                estimated = known_objects.copy()
                for obj1, (x1, y1) in known_objects:
                    for obj2, (x2, y2) in known_objects:
                        distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2 ** 2))
                        self.srg.update_weights(obj1, obj2, distance)

            # print(self.srg.get_target_distribution(target))

        # target = "oven"
        #
        # print(self.srg.get_target_distribution(target))
        #
        # solid_known_objects = [("kettle", (-150, 150)), ("toaster", (-200, 180)), ("oven", (-200, -150)),
        #                        ("cushion", (80, -200)), ("sofa", (70, 100)), ("bed", (50, -76))]
        # simple_map = self.simple_map
        #
        # colors = {}
        #
        # res = object_locator.calculate_likelihoods(simple_map, target, self.srg, solid_known_objects)
        # colors = self._plot(res, solid_known_objects, target, f"train-first", colors)
        #
        # for i in range(10):
        #     known_objects = []
        #     for obj, (x, y) in solid_known_objects:
        #         x += np.random.normal(scale=25)
        #         y += np.random.normal(scale=25)
        #         known_objects.append((obj, (x, y)))
        #
        #     estimated = known_objects.copy()
        #     del estimated[2]
        #     res = object_locator.calculate_likelihoods(simple_map, target, self.srg, estimated)
        #     colors = self._plot(res, solid_known_objects, target, f"train-{i}", colors)
        #     for j in range(len(known_objects)):
        #         obj1, (x1, y1) = known_objects[j]
        #         for k in range(j + 1, len(known_objects)):
        #             obj2, (x2, y2) = known_objects[k]
        #             distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
        #             self.srg.update_weights(obj1, obj2, distance)
        #     print(f"Iteration {i} done")
        #
        # known_objects = []
        # for obj, (x, y) in solid_known_objects:
        #     x += np.random.normal(scale=50)
        #     y += np.random.normal(scale=50)
        #     known_objects.append((obj, (x, y)))
        #
        # res = object_locator.calculate_likelihoods(simple_map, target, self.srg, solid_known_objects)
        # self._plot(res, solid_known_objects, target, f"train-final", colors)
        # print("done")

    def _plot(self, res, known_objects, target, figname, colors):
        fig, ax = plt.subplots()

        x, y, z = [], [], []
        for tup in res:
            x.append(tup[0])
            y.append(tup[1])
            z.append(tup[2])

        highest_prob = max(z)
        highest_prob_index = z.index(highest_prob)
        highest_prob_coords = (x[highest_prob_index], y[highest_prob_index])

        ax.scatter(x, y, c=z, cmap='cool', marker='.')

        # ax.plot(highest_prob_coords[0], highest_prob_coords[1], 'k.', markersize=5)

        # for name, (x, y) in known_objects:
        #     if name not in colors:
        #         col = np.random.random(3, )
        #         colors.update({name: col})
        #     else:
        #         col = colors.get(name)
        #     ax.plot(x, y, color=col, marker='.' if name == target else 'x', label=name)
            # circle = patches.Circle((x, y), radius=self.srg.get_distance(name, target), facecolor='none',
            #                         edgecolor=col)
            # ax.add_patch(circle)

        ax.legend()
        ax.axis('equal')
        ax.set_xlim(-250, 250)
        ax.set_ylim(-250, 250)
        fig.savefig(f"{figname}.png")
        plt.close(fig)
        return colors

    def _validate_subbed_vars(self):
        return self.target_object is not None and self.known_objects is not None and self.simple_map is not None

    @staticmethod
    def generate_pose(pose=None, variance=0.0):
        variance = 10  # FIXME
        width = 500 / 20
        height = 500 / 20
        if pose is not None:
            return Point32(pose.x + np.random.normal(0, variance),
                           pose.y + np.random.normal(0, variance),
                           0)
        else:
            return Point32((np.random.uniform(-width / 2, width / 2)), np.random.uniform(-height / 2, height / 2), 0)


if __name__ == '__main__':
    rospy.init_node('ops', anonymous=True)
    ops_locator = OPS()
    rospy.spin()
    # debugging - not for main use
    # known_objs = [('bed', (43, -8)), ('coffee table', (44, -6)), ('bedside table', (85, -29)), ('mirror', (56, -22)),
    #               ('mirror', (54, -28)), ('chair', (47, -2)), ('cushion', (85, -6)), ('cushion', (76, -8)),
    #               ('cushion', (87, -28)), ('lamp', (85, -4)), ('plant', (60, -20)), ('plant', (75, -16)),
    #               ('wardrobe', (64, -35)), ('cushion', (77, -40)), ('sofa', (57, -74)), ('television', (53, -63)),
    #               ('cushion', (55, -72)), ('mirror', (64, -79)), ('shelf', (72, -77)), ('shelf', (77, -98)),
    #               ('lamp', (68, -96))]
    # known_objs = [('sofa', (-19, -33)), ('bedside table', (-16, -37)), ('mirror', (-15, -29)), ('bed', (-27, -71)),
    #               ('wardrobe', (-34, -59)), ('coat stand', (-27, -26)), ('cushion', (-29, -54)), ('kettle', (-33, -43)),
    #               ('lamp', (-24, -28)), ('computer', (-36, -77)), ('plant', (-37, -58)), ('sink', (-56, -62)),
    #               ('shower', (-57, -54)), ('washing machine', (-76, -68)), ('bath', (-70, -71)), ('toilet', (-45, -43)),
    #               ('plant', (0, -71))]
    # ops_locator = OPS()
    # res = object_locator.calculate_likelihoods(simple_map=np.zeros(shape=(4000, 4000)), target="sink", srg=SRG(),
    #                                            known_obj_locs=known_objs)
    # ops_locator._plot(res, known_objs, "sink", figname="eeeeee", colors={})
#