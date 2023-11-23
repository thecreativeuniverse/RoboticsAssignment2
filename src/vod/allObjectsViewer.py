#!/usr/bin/env python3

import rospy
import os.path

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg
import time

if __name__ == '__main__':
    '''
    Publishes example pointcloud
    '''
    rospy.init_node('point_polygon_scaler')
    pointcloud_publisher = rospy.Publisher("/my_pointcloud_topic", PointCloud, queue_size=10)

    rospy.loginfo("pcl_publish_example")
    #giving some time for the publisher to register
    rospy.sleep(0.5)
    #declaring pointcloud
    itemPointcloud = PointCloud()
    #filling pointcloud header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    itemPointcloud.header = header
    #filling some points
    path = os.path.dirname(__file__)
    path = os.path.join(path, "../mapping/out/itemList.txt")
    file1 = open(path, 'r')
    lines = file1.readlines()

    itemPointcloud.points.append(Point32(1.0, 1.0, 0.0))
    itemPointcloud.points.append(Point32(2.0, 2.0, 0.0))
    itemPointcloud.points.append(Point32(3.0, 3.0, 0.0))
    for item in lines:
        data = item[1:-2]
        data = data.split(",")
        itemPointcloud.points.append(Point32((float(data[1]) - 250) / 20, -1 * ((float(data[2]) - 250) / 20), 0.0))

    #publish
    while True:
        pointcloud_publisher.publish(itemPointcloud)
        time.sleep(1)