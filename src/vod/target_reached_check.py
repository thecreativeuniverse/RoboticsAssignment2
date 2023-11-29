#!/usr/bin/env python3
import os

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud
import subprocess

class TargetChecker:

    def __init__(self):
        self.location = None
        self.target_obj_locations = set()

        rospy.Subscriber("my_pointcloud_topic2", PointCloud, self.location_callback)
        rospy.Subscriber("target_obj_loc", PointCloud, self.target_obj_callback)

    def location_callback(self, location):
        self.location = (location.points[0].x, location.points[0].y)
        self.check_is_close()

    def target_obj_callback(self, target_obj):
        for point in target_obj.points:
            self.target_obj_locations.add((point.x, point.y))
        self.check_is_close()

    def check_is_close(self):
        if len(self.target_obj_locations) == 0 or self.location is None:
            return
        print(self.target_obj_locations)
        x, y = self.location
        for temp_x, temp_y in self.target_obj_locations:
            dist = self.dist(x, y, temp_x, temp_y)
            print(dist)
            if dist < 0.5:
                print("Successfully found target object. Ending episode.")
                path = os.path.join(os.path.dirname(__file__), "../main/stop")
                subprocess.run(path)
                exit(0)

    @staticmethod
    def dist(x1, y1, x2, y2):
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


if __name__ == '__main__':
    rospy.init_node("target_reached_check")
    TargetChecker()
    rospy.spin()