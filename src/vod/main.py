#!/usr/bin/env python3
import copy
import os.path
from kmeans import generate_centers
import traceback
import numpy as np

# 1)begin by subscribing to both robot pos and laserscan
# 2)use the combination to identify if an object is visible
# 3)add it to list of seen objects (unclustered)
# 4)cluster list
# 5)publish it!
# 6 )NO STRESS THIS TIME!!!!!!!!!!!!!!


import math
import rospy
import random
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg

from tf.transformations import euler_from_quaternion


class SVOD:

    def __init__(self):
        self.clustered_obj = rospy.Publisher('/known_objects', String, queue_size=1)
        self.unclustered_obj = rospy.Publisher('/known_objects_unclustered', PointCloud, queue_size=1)
        self.bot = rospy.Publisher("/my_pointcloud_topic2", PointCloud, queue_size=1)

        self.current_location = (0, 0, 0)
        self.estimated_location = (0, 0, 0)

        path = os.path.dirname(__file__)
        path = os.path.join(path, "../mapping/out/itemList.txt")
        file1 = open(path, 'r')
        self.lines = file1.readlines()

        # self.foundObjects = []
        # self.allObjects = []
        # self.itemLists = []
        # self.estimated_clusters = []
        # self.itemListNames = []

        self.all_items = {}

        for item in self.lines:
            (obj, _, _) = eval(item)
            self.all_items.update({obj: Item(name=obj)})

        self.ALREADY_CALCULATING = False

    # def generateItemLists(self, item):
    #     name, x_coord, y_coord = item
    #     if name in self.itemListNames:
    #         index = self.itemListNames.index(item[0])
    #         self.itemLists[index].append([x_coord, y_coord])
    #         self.estimated_clusters[index] += 1
    #     else:
    #         self.itemListNames.append(name)
    #         self.itemLists.append([[x_coord, y_coord]])
    #         self.estimated_clusters.append(1)

    def doKmeans(self, scanned_objs):

        for item in self.all_items.values():
            all_seen = item.all_locations
            if len(all_seen) == 0:
                continue
            min_cluster = item.num_clusters
            num_scanned = scanned_objs.get(item.name) + 1 if item.name in scanned_objs.keys() else 1
            max_cluster = min_cluster + num_scanned

            # for i in range(len(self.itemListNames)):
            #     if self.itemListNames[i] not in scanned_objs.keys():
            #         min_cluster = self.estimated_clusters[i]
            #         max_cluster = min_cluster + 1
            #     else:
            #         min_cluster = self.estimated_clusters[i] - 2
            #         max_cluster = self.estimated_clusters[i] + scanned_objs.get(self.itemListNames[i], 1)
            all_seen_list = list(all_seen)
            centers = generate_centers(all_seen_list.copy(), max(min_cluster, 2), min(max_cluster, len(all_seen)))
            item.num_clusters = len(centers)
            new_found_objs = set()
            for coords in centers:
                if coords[0] != 300:
                    coords = tuple(coords)
                    new_found_objs.add(coords)
                    distances = [self.dist(seen, coords) for seen in all_seen_list]
                    to_keep = min(len(all_seen), max(len(centers), 5))
                    while len(new_found_objs) < to_keep and len(distances) > 0:
                        index = distances.index(min(distances))
                        del distances[index]
                        new_found_objs.add(all_seen_list[index])
            item.all_locations = new_found_objs

    def bearing(self, x1, y1, x2, y2):
        rad2deg = 57.2957795130823209
        if x1 == x2 and y1 == y2:
            return 0
        theta = math.atan2(x1 - x2, y1 - y2)
        return rad2deg * theta

    def addPositionNoise(self):
        return (1 * 2 * (0.5 - random.random()))

    def addRotationNoise(self):
        return (2 * 2 * (0.5 - random.random()))

    def generateItemCoords(self, robot_position_x, robot_position_y, item_angle, item_distance):
        item_x = math.sin(math.radians(item_angle)) * item_distance + robot_position_x
        item_y = math.cos(math.radians(item_angle)) * item_distance + robot_position_y

        return item_x, item_y

    def odomCallback(self, msg):
        pos = [msg.pose.pose.position.x * 20, msg.pose.pose.position.y * 20]
        quat = (0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quat)
        degrees = 360 - (270 + ((180 * euler[2]) / math.pi)) % 360
        self.current_location = (pos[0], pos[1], degrees)

    def estimate_callback(self, msg):
        pos = [msg.pose.position.x * 20, msg.pose.position.y * 20]
        quat = (0, 0, msg.pose.orientation.z, msg.pose.orientation.w)
        euler = euler_from_quaternion(quat)
        degrees = 360 - (270 + ((180 * euler[2]) / math.pi)) % 360
        self.estimated_location = (pos[0], pos[1], degrees)

    def lsCallback(self, msg):
        try:
            # robot position and laserscan pointcloud
            (estimate_x, estimate_y, estimate_theta) = copy.deepcopy(self.estimated_location)
            (current_x, current_y, current_theta) = copy.deepcopy(self.current_location)

            robot_pos = PointCloud()
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            robot_pos.header = header
            # robot_pos.points.append(Point32(current_x / 20, current_y / 20, 0))
            robot_pos.points.append(Point32(estimate_x / 20, estimate_y / 20, 0))
            for i in range(len(msg.ranges)):
                laser_angle = current_theta
                laser_x = math.sin(math.radians(laser_angle + 90 - ((i * 180) / 500))) * msg.ranges[i] + current_x / 20
                laser_y = math.cos(math.radians(laser_angle + 90 - ((i * 180) / 500))) * msg.ranges[i] + current_y / 20
                robot_pos.points.append(Point32(laser_x, laser_y, 0))
            self.bot.publish(robot_pos)

            all_objects_pointcloud = PointCloud()
            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'map'
            all_objects_pointcloud.header = header
            for item in self.all_items.values():
                for (x, y) in item.all_locations:
                    all_objects_pointcloud.points.append(Point32(x / 20, y / 20, 0))
            self.unclustered_obj.publish(all_objects_pointcloud)

            if self.ALREADY_CALCULATING:
                return
            self.ALREADY_CALCULATING = True
            scanned_objs = {}
            for item in self.lines:
                (obj, x, y) = eval(item)
                x -= 250
                y -= 250
                y *= -1
                distance = math.sqrt((x - current_x) ** 2 + (y - current_y) ** 2)
                angle = 360 + self.bearing(x, y, current_x, current_y)
                robot_angle = estimate_theta
                if 360 + (robot_angle - 90) % 360 < 360 + angle % 360 < 360 + (robot_angle + 90) % 360:
                    line_index = (round(((90 - (angle % 360 - robot_angle)) / 180) * 499))
                    if distance < 20 * msg.ranges[line_index]:
                        count = scanned_objs.get(obj) if obj in scanned_objs.keys() else 0
                        scanned_objs.update({obj: count + 1})
                        item = self.all_items.get(obj)
                        item_pos_x, item_pos_y = self.generateItemCoords(estimate_x, estimate_y, angle % 360, distance)
                        item.add(item_pos_x, item_pos_y)

            self.doKmeans(scanned_objs)
            stuff = String(str(self.get_all_locs()))
            self.clustered_obj.publish(stuff)
        except Exception as e:
            print("Something went wrong in kmeans:", e)
            print(traceback.format_exc())
        finally:
            self.ALREADY_CALCULATING = False

    def get_all_locs(self):
        locs = []
        for name, item in self.all_items.items():
            for (x, y) in item.all_locations:
                locs.append((name, (x, y)))
        return locs

    def listener(self):
        rospy.init_node('known_objects', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        rospy.Subscriber('base_scan', LaserScan, self.lsCallback, queue_size=1)
        rospy.Subscriber('/estimated_pose', PoseStamped, self.estimate_callback)
        rospy.spin()

    @staticmethod
    def dist(coords1, coords2):
        x1, y1 = coords1
        x2, y2 = coords2
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


class Item:

    def __init__(self, name=""):
        self.name = name
        self.all_locations = set()
        self.num_clusters = 0

    def add(self, x, y):
        self.all_locations.add((int(x), int(y)))


if __name__ == '__main__':
    try:
        new = SVOD()
        new.listener()
    except rospy.ROSInterruptException:
        pass
