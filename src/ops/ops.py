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
import time
import sys
from sklearn.cluster import DBSCAN
from sklearn.metrics import silhouette_score

from sklearn.exceptions import ConvergenceWarning


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

        particles = self.particle_cloud.points
        if len(particles) == 0:
            print("No particles available to estimate long term goal!")
            return

        particle_coords = [(p.x, p.y) for p in particles]

        dbscan = DBSCAN(eps=0.5, min_samples=3)
        dbscan.fit(particle_coords)
        labels = dbscan.labels_
        unique_labels = set(labels)

        particles = np.array(particles.copy())
        cluster_weights = []
        best_particle_per_cluster = []

        simple_map_flat = np.array(self.simple_map)
        size = int(np.sqrt(len(simple_map_flat)))
        simple_map = np.split(simple_map_flat, size)

        for k in unique_labels:
            if k == -1:
                continue
            cluster = particles[labels == k]
            if len(cluster) == 0:
                print("cluster of len 0!")
                continue
            weights = [object_locator.get_weight(particle, target=self.target_object, srg=self.srg,
                                                 known_obj_locs=known_object_locations, simple_map=simple_map) for
                       particle in cluster]
            best_particle = cluster[weights.index(max(weights))]
            ave_weight = sum(weights) / len(weights)

            cluster_weights.append(ave_weight)
            best_particle_per_cluster.append(best_particle)

        best_cluster = cluster_weights.index(max(cluster_weights))
        estimated_pos = best_particle_per_cluster[best_cluster]

        ###################################

        # particles_weights = [object_locator.get_weight(particle, self.target_object, self.srg, known_object_locations)
        #                      for particle in self.particle_cloud.points]
        # if len(particles_weights) == 0:
        #     return
        # index = particles_weights.index(max(particles_weights))
        # estimated_pos = self.particle_cloud.points[index]

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

            simple_map_flat = np.array(self.simple_map)
            size = int(np.sqrt(len(simple_map_flat)))
            simple_map = np.split(simple_map_flat, size)

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
            particles_weights = [
                object_locator.get_weight(particle, self.target_object, srg, known_object_locations, simple_map)
                for
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
                    particles_kept.append(
                        self.generate_pose(pose=particles_kept[i], variance=(particles_weights[i] * 1000)))
                    current_threshold += tick_size

                particles_weights = [
                    object_locator.get_weight(particle, self.target_object, srg, known_object_locations, simple_map) for particle in
                    particles_kept]

                while len(particles_weights) < particles_to_keep:
                    new_poses = []
                    for i in range(50):
                        new_pose = self.generate_pose()
                        new_weight = object_locator.get_weight(new_pose, self.target_object, srg,
                                                               known_object_locations, simple_map)
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
        width = 500 / 20
        height = 500 / 20
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

    # # # debugging - not for main use
    # known_objs = [('oven', (11.0, 11.0)), ('table', (-0.9999999999999858, 18.999999999999986)), ('coffee table', (48.0, 33.0)), ('shelf', (-51.00001621119738, -62.99993801122765)), ('desk', (-81.32812499999997, -106.95312499999999)), ('television', (-35.98632812500003, -94.94970703125)), ('chair', (-39.999832019209876, -92.99830630794168)), ('cushion', (-39.00067917060491, -118.99980371070329)), ('plant', (-58.99999999996727, -109.99999999990182)), ('sink', (94.0, 2.000000000000014)), ('shower', (86.0, 34.000000000000014)), ('washing machine', (96.0, -3.000000000000014)), ('tumble dryer', (88.0, 2.000000000000014)), ('bath', (82.0, -16.000000000000043)), ('toilet', (80.0, 22.999999999999986)), ('sofa', (-30.0, -28.999999999999986)), ('fridge', (-15.999999999999986, -13.000000000000014)), ('lamp', (-48.0, -104.00000000000001)), ('mirror', (-77.99999995529649, -83.00000001490116)), ('wardrobe', (-57.99999999999999, -70.0)), ('bed', (-74.00000000000003, -123.99999999999999)), ('bedside table', (-51.009765625, -50.134765625)), ('kettle', (-55.00000000000001, -119.00000000000003))]
    # #
    # ops_locator = OPS(filename="out/srg.json")
    # target = "bedside table"
    # res = object_locator.calculate_likelihoods(simple_map=np.zeros(shape=(4000, 4000)), target=target, srg=ops_locator.srg,
    #                                            known_obj_locs=known_objs)
    # ops_locator._plot(res, known_objs, target, figname="somethibng", colors={})
