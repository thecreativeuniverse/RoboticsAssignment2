#!/usr/bin/env python
import copy
import os.path

# 1)begin by subscribing to both robot pos and laserscan
# 2)use the combination to identify if an object is visible
# 3)add it to list of seen objects (unclustered)
# 4)cluster list
# 5)publish it!
# 6 )NO STRESS THIS TIME!!!!!!!!!!!!!!


import math
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

from tf.transformations import euler_from_quaternion


class SVOD():

    def __init__(self):

        self.pub = rospy.Publisher('/known_objects', String, queue_size=1)

        self.current_location = (0, 0, 0)

        path = os.path.dirname(__file__)
        path = os.path.join(path, "../mapping/out/itemList.txt")
        file1 = open(path, 'r')
        self.lines = file1.readlines()

        self.foundObjects = []

    def bearing(self, a1, a2, b1, b2):
        RAD2DEG = 57.2957795130823209
        if a1 == b1 and a2 == b2:
            return 0
        theta = math.atan2(b1 - a1, a2 - b2)
        if theta < 0.0:
            theta += math.pi * 2
        return RAD2DEG * theta

    def odomCallback(self, msg):
        scale = 20
        pos = [msg.pose.pose.position.x * 20, msg.pose.pose.position.y * 20]
        rot = [msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        w = rot[1]
        z = rot[0]
        quat = (0, 0, z, w)
        euler = euler_from_quaternion(quat)
        degrees = 360 - (270 + (180 * euler[2]) / math.pi) % 360
        self.current_location = (pos[0], pos[1], degrees)

    def lsCallback(self, msg):
        fixedCurrentLocation = copy.deepcopy(self.current_location)
        for item in self.lines:
            data = item[1:-2]
            data = data.split(",")
            distance = math.sqrt(
                (float(data[1]) - fixedCurrentLocation[1]) ** 2 + (float(data[2]) - fixedCurrentLocation[2]) ** 2)
            angle = 360 + self.bearing(float(data[1]), float(data[2]), fixedCurrentLocation[1], fixedCurrentLocation[2])
            robotAngle = 360 + float(fixedCurrentLocation[2])
            if distance < 100:
                if angle > (robotAngle - 90) and angle < (robotAngle) + 90:
                    lineIndex = ((90 - abs(robotAngle - angle)) / 180) * 500
                    if distance < 20 * msg.ranges[round(lineIndex)]:
                        exists = False
                        for objectPosition in self.foundObjects:
                            if objectPosition[1] == float(data[1]) and objectPosition[2] == float(data[2]):
                                exists = True
                            else:
                                pass
                        if not exists:
                            self.foundObjects.append((data[0], float(data[1]), float(data[2])))
        stuff = String(str(self.foundObjects))
        self.pub.publish(stuff)

    def listener(self, ):
        rospy.init_node('known_objects', anonymous=True)
        rospy.Subscriber('/odom', Odometry, self.odomCallback)

        rospy.Subscriber('base_scan', LaserScan, self.lsCallback)
        rospy.spin()


if __name__ == '__main__':
    try:
        new = SVOD()
        new.listener()
    except rospy.ROSInterruptException:
        pass
