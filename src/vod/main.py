#!/usr/bin/env python3
import copy
import os.path
from kmeans import generate_centers

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


class SVOD():

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

        self.foundObjects = []
        self.allObjects = []
        self.itemLists = []
        self.itemListNames = []

    def generateItemLists(self, item):
        name, x_coord, y_coord = item
        if name in self.itemListNames:
            self.itemLists[self.itemListNames.index(item[0])].append([x_coord, y_coord])
        else:
            self.itemListNames.append(name)
            self.itemLists.append([[x_coord, y_coord]])

    def doKmeans(self):
        self.foundObjects = []
        for i in range(len(self.itemListNames)):
            centers = generate_centers(self.itemLists[i])
            self.itemLists[i] = []
            for coords in centers:
                if tuple(coords)[0] != 300:
                    self.foundObjects.append((self.itemListNames[i], tuple(coords)))
                    self.itemLists[i].append(tuple(coords))

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
        (current_x, current_y, current_theta) = copy.deepcopy(self.current_location)
        (estimate_x, estimate_y, estimate_theta) = copy.deepcopy(self.estimated_location)
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
                    exists = False
                    for _, (pos_x, pos_y) in self.allObjects:
                        if pos_x == x and pos_y == y:
                            exists = True
                        else:
                            pass
                    if not exists:
                        # estimate new coords
                        # FIXME
                        item_pos_x, item_pos_y = self.generateItemCoords(estimate_x, estimate_y, angle % 360, distance)
                        self.allObjects.append((obj, (item_pos_x, item_pos_y)))
                        self.generateItemLists((obj, item_pos_x, item_pos_y))

        self.doKmeans()
        stuff = String(str(self.foundObjects))
        self.clustered_obj.publish(stuff)
        ###########################
        # robot position and laserscan pointcloud
        robot_pos = PointCloud()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        robot_pos.header = header
        robot_pos.points.append(Point32(current_x / 20, current_y / 20, 0))
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
        for data in self.allObjects:
            all_objects_pointcloud.points.append(Point32(data[1][0] / 20, data[1][1] / 20, 0))
        self.unclustered_obj.publish(all_objects_pointcloud)

    def listener(self):
        rospy.init_node('known_objects', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.odomCallback)
        rospy.Subscriber('base_scan', LaserScan, self.lsCallback, queue_size=1)
        rospy.Subscriber('/estimated_pose', PoseStamped, self.estimate_callback)
        rospy.spin()


if __name__ == '__main__':
    try:
        new = SVOD()
        new.listener()
    except rospy.ROSInterruptException:
        pass
