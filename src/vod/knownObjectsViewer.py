#!/usr/bin/env python3
import time

import rospy
from sensor_msgs.msg import PointCloud
import std_msgs.msg
from geometry_msgs.msg import Point32


class KnownThings():

    def __init__(self):
        rospy.Subscriber("/known_objects", std_msgs.msg.String, callback=self.known_objects_callback,queue_size=1)
        self.all_cloud_pub = rospy.Publisher("/known_obj_cloud", PointCloud, queue_size=1)

        self.known_objects = None
        self.target_obj = None

    def known_objects_callback(self, msg):
        # declaring pointcloud
        pointcloud = PointCloud()
        # filling pointcloud header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pointcloud.header = header

        known_objects = eval(msg.data)
        self.known_objects = known_objects
        for obj, (x, y) in known_objects:
            pointcloud.points.append(Point32(x / 20, y / 20, 0.0))

        self.all_cloud_pub.publish(pointcloud)


if __name__ == '__main__':
    rospy.init_node("knownObjectThings")
    knownThings = KnownThings()
    time.sleep(10)
    rospy.spin()
