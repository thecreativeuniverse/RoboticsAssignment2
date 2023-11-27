#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud
import std_msgs.msg
from geometry_msgs.msg import Point32


class KnownThings():

    def __init__(self):
        rospy.Subscriber("/known_objects", std_msgs.msg.String, callback=self.known_objects_callback)
        rospy.Subscriber("/target_object", std_msgs.msg.String, callback=self.target_obj_callback)
        self.all_cloud_pub = rospy.Publisher("/known_obj_cloud", PointCloud, queue_size=1)
        self.target_obj_pub = rospy.Publisher("/target_obj_loc", PointCloud, queue_size=1)

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
        # print(known_objects)
        for obj, (x, y) in known_objects:
            pointcloud.points.append(Point32(x / 20, y / 20, 0.0))

        self.all_cloud_pub.publish(pointcloud)
        self.publish_target_obj()

    def target_obj_callback(self, msg):
        self.target_obj = msg.data
        self.publish_target_obj()

    def publish_target_obj(self):
        if self.target_obj is None and self.known_objects is None:
            return

        target_obj = [x for x in self.known_objects if x[0] == self.target_obj]
        if len(target_obj) == 0:
            return

        # declaring pointcloud
        pointcloud = PointCloud()
        # filling pointcloud header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        pointcloud.header = header

        _, (x, y) = target_obj[0]
        pointcloud.points.append(Point32(x / 20, y / 20, 0.0))

        self.target_obj_pub.publish(pointcloud)


if __name__ == '__main__':
    rospy.init_node("knownObjectThings")
    knownThings = KnownThings()
    rospy.spin()
