#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from tf.transformations import euler_from_quaternion


from PIL import Image

class ASTAR:
    def __init__(self):
        rospy.Subscriber("map", OccupancyGrid,self.make_map,queue_size=1)
        rospy.Subscriber("goal_position",PointCloud,self.addDestination,queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odomCallback)

        self.simple_map = []
        self.destination =[]
        self.current_location =[]

        rospy.spin()

    def addDestination(self,msg):
        self.destination = msg.data[0]

    def odomCallback(self, msg):
        pos = [msg.pose.pose.position.x * 20, msg.pose.pose.position.y * 20]
        quat = (0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quat)
        degrees = 360 - (270 + ((180 * euler[2]) / math.pi)) % 360
        self.current_location = (pos[0], pos[1], degrees)
    def make_map(self,msg):

        data = msg.data
        data = np.asarray(data)
        size = int(np.sqrt(len(data)))
        self.simple_map = np.split(data, size)
        self.generateOutputImage()
    def generateOutputImage(self):
        # -1 = unvisited
        # 100 = floor
        # 0 = wall
        print(self.current_location)
        print(self.destination)
        robot_x,robot_y,robot_theta = self.current_location
        robot_x = 2000+round(robot_x)
        robot_y = 2000+round(robot_y)
        print(self.simple_map[robot_x][robot_y])
        self.simple_map[robot_x][robot_y] = 200
        PIL_image = Image.fromarray(np.uint8(self.simple_map))
        pixels = PIL_image.getdata()
        print(PIL_image.width,PIL_image.height)

        PIL_image.save("test.png","PNG")

        #PIL_image = Image.fromarray(numpy_image.astype('uint8'), 'RGB')



if __name__ == "__main__":
    rospy.init_node('ASTAR', anonymous=True)

    ASTAR()
