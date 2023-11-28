#!/usr/bin/env python3

import rospy
import os.path

from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg
import time
import numpy as np


def target_obj_callback(self, msg):
    self.target_obj = msg.data
    self.publish_target_obj()


def publish_target_obj():
    target_obj = [x for x in self.known_objects if x[0] == self.target_obj]
    if len(target_obj) == 0:
        return


if __name__ == '__main__':
    '''
    Publishes example pointcloud
    '''
    rospy.init_node('point_polygon_scaler')
    pointcloud_publisher = rospy.Publisher("/my_pointcloud_topic", PointCloud, queue_size=10)
    target_obj_publisher = rospy.Publisher("/target_object", std_msgs.msg.String, queue_size=10)
    target_obj_pub = rospy.Publisher("/target_obj_loc", PointCloud, queue_size=1)

    rospy.loginfo("pcl_publish_example")
    # giving some time for the publisher to register
    rospy.sleep(0.5)
    # declaring pointcloud
    itemPointcloud = PointCloud()
    # filling pointcloud header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    itemPointcloud.header = header
    # filling some points
    path = os.path.dirname(__file__)
    path = os.path.join(path, "../mapping/out/itemList.txt")
    with open(path, 'r') as file1:
        lines = file1.readlines()

    for item in lines:
        data = item[1:-2]
        data = data.split(",")
        itemPointcloud.points.append(Point32((float(data[1]) - 250) / 20, -1 * ((float(data[2]) - 250) / 20), 0.0))

    target_obj, _, _ = eval(lines[np.random.randint(0, high=len(lines))])
    target_obj_publisher.publish(std_msgs.msg.String(target_obj))
    print("target obj ", target_obj)  # debugging

    target_obj_locs = []
    for line in lines:
        obj, x, y = eval(line)
        if obj != target_obj:
            continue
        target_obj_locs.append((x,y))
    # publish
    pointcloud_publisher.publish(itemPointcloud)

    # declaring pointcloud
    target_pointcloud = PointCloud()
    # filling pointcloud header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'map'
    target_pointcloud.header = header
    for x, y in target_obj_locs:
        target_pointcloud.points.append(Point32((x - 250) / 20, (-y + 250) / 20, 0.0))

    time.sleep(5)
    target_obj_pub.publish(target_pointcloud)
