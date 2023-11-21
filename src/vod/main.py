#!/usr/bin/env python3
import copy
import os.path
#test

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
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, Pose
import std_msgs.msg


from tf.transformations import euler_from_quaternion, quaternion_from_euler


class SVOD():

    def __init__(self):

        self.pub = rospy.Publisher('/known_objects', String, queue_size=1)
        self.bot = rospy.Publisher("/my_pointcloud_topic2", PointCloud, queue_size=1)


        self.current_location = (0, 0, 0)

        path = os.path.dirname(__file__)
        path = os.path.join(path, "../mapping/out/itemList.txt")
        file1 = open(path, 'r')
        self.lines = file1.readlines()

        self.foundObjects = []

    def bearing(self, x1, y1, x2, y2):
        RAD2DEG = 57.2957795130823209
        if x1 == x2 and y1 == y2:
            return 0
        theta = math.atan2(x2 - x1, y2 - y1)
        return 90-(RAD2DEG * theta)

    def odomCallback(self, msg):
        scale = 20
        pos = [msg.pose.pose.position.x * 20, msg.pose.pose.position.y * 20]
        quat = (0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quat)
        degrees = 360 - (270 + ((180 * euler[2]) / math.pi)) % 360
        self.current_location = (pos[0], pos[1], degrees)

    def lsCallback(self, msg):
        (current_x,current_y, current_theta) = copy.deepcopy(self.current_location)
        for item in self.lines:
            (obj, x, y) = eval(item)
            x -= 250
            y -= 250
            y*=-1
            distance = math.sqrt((x - current_x) ** 2 + (y - current_y) ** 2)
            angle = 360 + self.bearing(x, y, current_x, current_y)
            robotAngle = 360 + current_theta
            #print(f"OBJECT {obj} loc {x}, {y} robotPos {current_x}, {current_y}, dist {distance}") #debugging
            if distance < 100:
                #print(f"dist<100 angle {angle} robotangle {robotAngle}") #debugging
                if robotAngle - 90 < angle < robotAngle + 90:
                    #print("within angle") #debugging
                    lineIndex = (round(((90 - (angle % 360) - (robotAngle % 360)) / 180) * 500))
                    #print(f"lineindex {lineIndex} thing dist {msg.ranges[lineIndex]}") #debugging
                    if distance < 20 * msg.ranges[lineIndex]:
                        #print("distance < 20*lineindec") #debugging
                        exists = False
                        for objectPosition in self.foundObjects:
                            if objectPosition[1] == x and objectPosition[2] == y:
                                exists = True
                            else:
                                pass
                        if not exists:
                            self.foundObjects.append((obj, x, y))
            #print("=============================")
        stuff = String(str(self.foundObjects))

        robotPos = PointCloud()
        # filling pointcloud header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        robotPos.header = header
        robotPos.points.append(Point32(current_x/20,current_y/20,0))
        self.bot.publish(robotPos)

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
