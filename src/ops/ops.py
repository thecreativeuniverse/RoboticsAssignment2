#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Header
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Pose, Point, Quaternion
import traceback
import numpy as np
from srg import SRG, TrainingSRG
import object_locator
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import sys
import subprocess
import signal
import psutil


class OPS:
    def __init__(self, train=False, filename=None):
        # initialising functions/variables
        rospy.Subscriber("target_object", String, self.target_object_callback)  # name

        # [[(5,-5),(5,-4),(5,-3),(5,-3),(5,-2),(5,-1),(5,0),(5,1),(5,2),(5,3),(5,4),(5,5)], [(4,-5), ....]
        rospy.Subscriber("map", OccupancyGrid, self.simple_map_callback)

        # [("name", (x,y)),("name", (x,y))]
        rospy.Subscriber("known_objects", String, self.known_objects_callback, queue_size=1)

        self.goal_pos_pub = rospy.Publisher("goal_position", PointCloud, queue_size=1)
        self.particle_cloud_pub = rospy.Publisher("goal_pos_estimation", PointCloud, queue_size=1)

        self.estimated_pose = None
        self.known_objects = None
        self.target_object = None

        # SRG
        self.srg = TrainingSRG(filename=filename) if train else SRG(filename=filename)  # Change depending on training
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

        self.ALREADY_CALCULATING = False

        if train:
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

    def calculate_long_term_goal(self, known_object_locations=None):
        particles_weights = [object_locator.get_weight(particle, self.target_object, self.srg, known_object_locations)
                             for particle in self.particle_cloud.points]
        if len(particles_weights) == 0:
            return
        index = particles_weights.index(max(particles_weights))
        estimated_pos = self.particle_cloud.points[index]

        goal_pointcloud = PointCloud()
        goal_pointcloud.header.frame_id = "map"
        goal_pointcloud.header.stamp = rospy.Time.now()
        goal_pointcloud.points = [estimated_pos]

        self.goal_pos_pub.publish(goal_pointcloud)
    #     if not self._validate_subbed_vars():
    #         "invalid"
    #         return
    #     if self.ALREADY_CALCULATING:
    #         return
    #
    #     self.ALREADY_CALCULATING = True
    #
    #     data = np.array(self.simple_map)
    #     size = int(np.sqrt(len(data)))
    #     simple_map = np.split(data, size)
    #
    #     known_objects = self.known_objects
    #     print(f"we know {len(known_objects)} objects and are looking for {self.target_object}")
    #
    #     srg = self.srg
    #     known_object_locations = []
    #
    #     for obj in known_objects:
    #         distance = srg.get_distance(obj[0], self.target_object)
    #         known_object_locations.append((obj[1], distance, 50))
    #
    #     res = object_locator.calculate_likelihoods(simple_map=simple_map, target=self.target_object, srg=srg,
    #                                                known_obj_locs=known_object_locations)
    #
    #     (x, y, prob) = max(res, key=lambda val: val[-1])
    #
    #     target_pointcloud = PointCloud()
    #     # filling pointcloud header
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = 'map'
    #     target_pointcloud.header = header
    #     target_pointcloud.points.append(Point32(x / 20, y / 20, 0))
    #
    #     self.goal_pos_pub.publish(target_pointcloud)
    #     self.ALREADY_CALCULATING = False

    def update_particle_cloud(self):
        self.particle_cloud_pub.publish(self.particle_cloud)
        if not self._validate_subbed_vars():
            return
        if self.ALREADY_CALCULATING:
            return
        self.ALREADY_CALCULATING = True

        try:
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

            # we vary the amount of particles based on how certain the algorithm is of the robot's position
            # particles_to_keep = round((100 / average_weight ** 0.5) - (average_weight / 10))
            particles_to_keep = self.pose_array_size  # TODO?

            # self.set_pose_array_size(particles_to_keep)

            # Add 15% more particles in uniform random locations; this will allow us to find a more accurate estimation of
            # the robot's position in the event the current estimation is completely wrong
            initial_particles += [self.generate_pose() for _ in range(round(len(initial_particles) * 0.15))]

            # Recalculate the weights of all particles and append them to an array, then calculate an average
            # particles_weights = [self.sensor_model.get_weight(scan, particle) for particle in initial_particles]
            particles_weights = [object_locator.get_weight(particle, self.target_object, srg, known_object_locations) for
                                 particle in initial_particles]

            sum_of_weights = sum(particles_weights)
            if sum_of_weights > 0:
                # derive the variance for random particle generation based on the updated weights for all the particles
                variance = 4

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

                # remove particles which don't have a high enough cumulative weight and replace them with variations of the
                # previous particle that did have a high enough cumulative weight
                i = 0
                for j in range(self.pose_array_size):
                    while i < len(cum_weights) and current_threshold > cum_weights[i]:
                        i += 1

                    if i < len(cum_weights):
                        break
                    particles_kept.append(self.generate_pose(pose=particles_kept[i], variance=(particles_weights[i] * 1000)))
                    current_threshold += tick_size

                particles_weights = [object_locator.get_weight(particle, self.target_object, srg, known_object_locations) for particle in particles_kept]

                while len(particles_weights) < particles_to_keep:
                    new_poses = []
                    for i in range(50):
                        new_pose = self.generate_pose()
                        new_weight = object_locator.get_weight(new_pose, self.target_object, srg, known_object_locations)
                        new_poses.append((new_pose, new_weight))
                    new_poses = sorted(new_poses, key=lambda x: x[1])

                    particles_kept.append(new_poses[-1][0])
                    particles_weights.append(new_poses[-1][1])

                # Safety in case it picks 0, shouldn't really enter this
                while current_threshold == 0:
                    current_threshold = np.random.uniform(0, tick_size)

                # update the particle cloud to have the new particles
                self.particle_cloud.points = particles_kept
            else:
                self.particle_cloud.points = [self.generate_pose() for _ in range(self.pose_array_size)]
            self.particle_cloud_pub.publish(self.particle_cloud)
            self.calculate_long_term_goal(known_object_locations=known_object_locations)
        except Exception as e:
            print("SOMETHING WENT WRONG:", e)
            print(traceback.format_exc())
        finally:
            self.ALREADY_CALCULATING = False

    def train(self):
        known_objects = self.known_objects

        if known_objects is None:
            return

        for obj1, (x1, y1) in known_objects:
            for obj2, (x2, y2) in known_objects:
                if obj1 == obj2:
                    continue
                distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                self.srg.update_weights(obj1, obj2, distance)
        self.srg.save_in_file()

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
        #
        # for name, (x, y) in known_objects:
        #     if name not in colors:
        #         col = np.random.random(3, )
        #         colors.update({name: col})
        #     else:
        #         col = colors.get(name)
        #     ax.plot(x, y, color=col, marker='.' if name == target else 'x', label=name)
        #     circle = patches.Circle((x, y), radius=self.srg.get_distance(name, target), facecolor='none',
        #                             edgecolor=col)
        #     ax.add_patch(circle)
        #
        #
        # ax.legend(bbox_to_anchor=(1,-1))
        ax.axis('equal')
        ax.set_xlim(-500, 500)
        ax.set_ylim(-500, 500)
        fig.savefig(f"{figname}.png")
        plt.close(fig)
        return colors

    def _validate_subbed_vars(self):
        return self.target_object is not None and self.known_objects is not None and self.simple_map is not None

    @staticmethod
    def generate_pose(pose=None, variance=0.0):
        # variance = 1
        width = 250 / 20
        height = 250 / 20
        if pose is not None:
            return Point32(pose.x + np.random.normal(0, variance),
                           pose.y + np.random.normal(0, variance),
                           0)
        else:
            return Point32((np.random.uniform(-width / 2, width / 2)), np.random.uniform(-height / 2, height / 2), 0)


if __name__ == '__main__':
    train = '--train' in sys.argv
    rospy.init_node('ops', anonymous=True)
    ops_locator = OPS(train=train, filename="out/srg.json")
    rospy.spin()

    # # debugging - not for main use
    # known_objs = [('fridge', (-5.4447517285518074, -10.511059390465505)),
    #               ('plant', (10.608419273698644, -87.45484271443448)),
    #               ('table', (-59.556690199019435, 99.06569097804586)),
    #               ('wardrobe', (-15.476453459049793, -122.73921405994496)),
    #               ('chair', (19.84616008613868, -72.60512049378275)),
    #               ('computer', (7.078984713188433, -104.15601402786699)),
    #               ('sofa', (-55.24512692101183, 14.022006410029306)),
    #               ('shelf', (68.19734379389512, -147.87893592516596)),
    #               ('cushion', (11.089820180766182, -88.7380641387179)),
    #               ('television', (-20.391789731646455, -8.565092528433112)),
    #               ('oven', (-58.71444612949229, -25.02076611695668)),
    #               ('desk', (-37.64326397440233, -137.7039960111055)),
    #               ('bedside table', (14.144943113750202, -107.77240408022112)),
    #               ('mirror', (-13.071875825168078, -87.59109840428454)),
    #               ('lamp', (-36.691051270977766, -90.60896619161309)),
    #               ('tumble dryer', (46.349451001924905, -149.25574643433427)),
    #               ('washing machine', (52.79477177118423, -151.77654956374687)),
    #               ('toilet', (53.34043442295351, -147.9053218065922)),
    #               ('shower', (78.79916836406649, -142.84975242267168))]
    #
    # ops_locator = OPS(filename="out/srg.json")
    # target  = "bedside table"
    # res = object_locator.calculate_likelihoods(simple_map=np.zeros(shape=(4000, 4000)), target=target, srg=ops_locator.srg,
    #                                            known_obj_locs=known_objs)
    # ops_locator._plot(res, known_objs, target, figname="multiplier", colors={})
