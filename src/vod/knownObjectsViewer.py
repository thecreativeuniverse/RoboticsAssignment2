#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
import std_msgs.msg
from geometry_msgs.msg import Point32

class KnownThings():

    def __init__(self):
        rospy.Subscriber("/known_objects", std_msgs.msg.String, callback=self.known_objects_callback)
        self.pub = rospy.Publisher("/known_obj_cloud", PointCloud, queue_size=1)

    def known_objects_callback(self, msg):
        # declaring pointcloud
        my_awesome_pointcloud = PointCloud()
        # filling pointcloud header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        my_awesome_pointcloud.header = header

        known_objects = eval(msg.data)
        #print(known_objects)
        for obj, x,y in known_objects:
            my_awesome_pointcloud.points.append(Point32(x / 20, y / 20, 0.0))

        self.pub.publish(my_awesome_pointcloud)





if __name__ == '__main__':
    rospy.init_node("knownObjectThings")
    knownThings = KnownThings()
    rospy.spin()
