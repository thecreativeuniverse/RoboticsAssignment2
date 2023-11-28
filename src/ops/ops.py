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
import sys
import subprocess
import signal
import psutil


class OPS:
    def __init__(self, train=False, filename=None):
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
        # TODO pass file into SRG()
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
        particles_weights = [object_locator.get_weight(particle, self.target_object, self.srg, known_object_locations) for
                             particle in
                             self.particle_cloud.points]
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

        # particles_to_remove = len(particles_weights) * 0.8
        # while len(particles_weights) > particles_to_remove:
        #     index = particles_weights.index((min(particles_weights)))
        #     del (particles_weights[index])
        #     del (initial_particles[index])
        #
        # while len(particles_weights) < particles_to_keep:
        #     new_poses = []
        #     for i in range(50):
        #         new_pose = self.generate_pose()
        #         new_weight = object_locator.get_weight(new_pose, self.target_object, srg, known_object_locations)
        #         new_poses.append((new_pose, new_weight))
        #     new_poses = sorted(new_poses, key=lambda x: x[1])
        #
        #     initial_particles.append(new_poses[-1][0])
        #     particles_weights.append(new_poses[-1][1])

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
        self.particle_cloud.points = particles_kept
        self.particle_cloud_pub.publish(self.particle_cloud)
        self.ALREADY_CALCULATING = False
        self.calculate_long_term_goal(known_object_locations=known_object_locations)

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
        ax.set_xlim(-250, 250)
        ax.set_ylim(-250, 250)
        fig.savefig(f"{figname}.png")
        plt.close(fig)
        return colors

    def _validate_subbed_vars(self):
        return self.target_object is not None and self.known_objects is not None and self.simple_map is not None

    @staticmethod
    def generate_pose(pose=None, variance=0.0):
        variance = 1
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

    # debugging - not for main use
    # known_objs = [('shelf', (-93.14452835164016, 73.41920915439394)), ('desk', (46.07865370213018, 80.40953218912472)),
    #               ('plant', (-10.427236259360711, -0.17810306183933733)),
    #               ('sofa', (-33.65711668199049, 57.855197469185754)), ('bed', (-34.7715647348744, -38.21090435312075)),
    #               ('coffee table', (-14.166142515099438, -30.158604026081818)),
    #               ('bedside table', (46.30342386383106, 41.1464173125231)),
    #               ('wardrobe', (-35.51305982231008, -16.825521752556767)),
    #               ('mirror', (-26.203383686732792, -24.47547812019029)),
    #               ('coat stand', (40.40673701886692, -2.4930133497383906)),
    #               ('cushion', (-14.16441404153609, 48.57381137185976)),
    #               ('computer', (34.690800588431586, 104.39804906177278)),
    #               ('shower', (79.54296843206673, 43.28594128099246)),
    #               ('washing machine', (-104.11959035193044, 72.55501874460295)),
    #               ('bath', (78.52785331243035, 92.2634601763056)),
    #               ('chair', (-106.42684179633581, -145.52016379274906)),
    #               ('lamp', (-18.591778258952147, -30.223221526798127)),
    #               ('toilet', (82.2877797375686, 19.765424362627996)),
    #               ('sink', (101.39199048899516, 102.91990740292268)),
    #               ('fridge', (-101.95904260453358, 59.127138586017054)),
    #               ('kettle', (-119.48416605934726, 38.49017875437319)),
    #               ('tumble dryer', (83.56569680252926, -9.194623264697938)),
    #               ('table', (-80.19083296789941, -80.80142776115142)),
    #               ('freezer', (-119.26186834687455, 21.314964158402404)),
    #               ('television', (-119.0127816461465, -22.152607912201432)),
    #               ('dishwasher', (-112.11745516599255, 63.211830087112645)),
    #               ('kitchen cabinet', (-103.07335678576173, 95.01340321771372)),
    #               ('toaster', (-110.91938459807002, 114.49453926038305)),
    #               ('oven', (-95.6510263394145, 81.4635198814303)),
    #               ('microwave', (-95.54827053802222, 73.85526397870721))]
    #
    # # # known_objs = [('sofa', (-19, -33)), ('bedside table', (-16, -37)), ('mirror', (-15, -29)), ('bed', (-27, -71)),
    # # #               ('wardrobe', (-34, -59)), ('coat stand', (-27, -26)), ('cushion', (-29, -54)), ('kettle', (-33, -43)),
    # # #               ('lamp', (-24, -28)), ('computer', (-36, -77)), ('plant', (-37, -58)), ('sink', (-56, -62)),
    # # #               ('shower', (-57, -54)), ('washing machine', (-76, -68)), ('bath', (-70, -71)), ('toilet', (-45, -43)),
    # # #               ('plant', (0, -71))]
    # ops_locator = OPS(filename="out/srg.json")
    # res = object_locator.calculate_likelihoods(simple_map=np.zeros(shape=(4000, 4000)), target="sink", srg=ops_locator.srg,
    #                                            known_obj_locs=known_objs)
    # ops_locator._plot(res, known_objs, "shelf", figname="aaaaa", colors={})
