#!/usr/bin/env python3
import copy

import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Twist, Point32
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from time import sleep
import traceback

from PIL import Image


def get_distance(start, end):
    start_x, start_y = start
    end_x, end_y = end
    distance = math.sqrt((start_x - end_x) ** 2 + (start_y - end_y) ** 2)
    return distance


class ASTAR:
    def __init__(self):
        self.object_threshold = 40
        self.threshold_met = False
        self.target_object = None
        rospy.Subscriber('known_objects', String, self.known_objects_callback)
        rospy.Subscriber('target_object', String, self.target_callback)

        self.simple_map = None  # 2d map of space
        self.size = None  # size of map
        # subscribes to map
        rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber("goal_position", PointCloud, self.destination_callback, queue_size=1)

        print("waiting for observations")
        while not self.threshold_met:
            sleep(0.5)
        print("Seeen it alll")

        # subscribes to final destination
        # subscribes to robot pos
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber('base_scan', LaserScan, self.ls_callback, queue_size=1)

        self.target_publisher = rospy.Publisher('astar_target', PointCloud, queue_size=1)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.goal_location = None  # actual place we are trying to go
        self.current_location = None  # where the robot is, and orientation
        self.pathfind_to = None
        self.PATHFINDING_COUNT = 0

    # takes the x,y coordinates of from goal position and saves them to a variable
    def destination_callback(self, msg):
        x = round(self.size / 2 + (msg.points[0].x * 20))
        y = round(self.size / 2 + (msg.points[0].y * 20))
        if not self.threshold_met:
            return
        if self.goal_location is not None:
            if get_distance((x, y), self.goal_location) < 1:
                return
        self.PATHFINDING_COUNT += 1
        self.calculate_path()
        self.goal_location = (x, y)

    def ls_callback(self, msg):
        self.hit_the_road(self.PATHFINDING_COUNT)


    def target_callback(self, msg):
        print("received target ", msg.data)
        self.target_object = msg.data

    def odom_callback(self, msg):
        # get robot coordinates
        x, y = msg.pose.pose.position.x * 20, msg.pose.pose.position.y * 20

        # get robot bearing
        quaternion = (0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        degrees = 360 - (270 + ((180 * euler[2]) / math.pi)) % 360

        # set them to current location
        self.current_location = (round(self.size / 2 + x), round(self.size / 2 - y), degrees)

        # determine if we have just started or if we have reached our target we can navigate to:
        # just started
        if self.pathfind_to is None:
            self.calculate_path()
            return

        # calculate the distance to the target we can navigate to
        distance_from_destination = get_distance(self.current_location, self.goal_location)

        # we have reached our target we can navigate to
        if distance_from_destination < 0.5:
            self.calculate_path()

    def calculate_path(self):  # calculates the path to the next destination we can drive to
        """
        We are somewhere, and we know where we want to go long term, but we can't just drive there since we don't know
        what is between us and the location. The solution is to drive as far as we can go, then hopefully we can see
        a little further, rinse and repeat.
        1) A* to the final destination until we get to an empty space
        2) say that empty space is where we want to drive to
        3) simplify the directions
        4) drive there
        """
        simple_map = self.simple_map
        if simple_map is None:
            return

        goal_x, goal_y = self.goal_location
        if simple_map[goal_y][goal_x] >= 0:
            self.pathfind_to = (goal_x, goal_y)
        else:
            distances = []
            for y in range(len(simple_map)):
                for x in [temp for temp in range(len(simple_map[y])) if simple_map[y][temp] == 0]:
                    next_to_nothing = False
                    for i in range(-1, 2):
                        for j in range(-1, 2):
                            if simple_map[y + i][x + j] == -1:
                                next_to_nothing = True
                                break
                    if not next_to_nothing:
                        continue
                    distances.append((get_distance(self.goal_location, (y, x)), x, y))
            dist, x, y = min(distances, key=lambda a: a[0])
            self.pathfind_to = (x, y)
        self.hit_the_road(self.PATHFINDING_COUNT)

    def bearing(self, x1, y1, x2, y2):
        rad2deg = 57.2957795130823209
        if x1 == x2 and y1 == y2:
            return 0
        theta = math.atan2(x1 - x2, y1 - y2)
        return rad2deg * theta

    def hit_the_road(self, pathfinding_count):
        print("Hitting the road") #debugging
        if self.pathfind_to is None:
            self.calculate_path()
            return
        try:
            particle_cloud = PointCloud()
            particle_cloud.header.frame_id = "map"
            particle_cloud.header.stamp = rospy.Time.now()
            particle_cloud.points = [Point32((self.pathfind_to[0] - (len(self.simple_map) / 2)) / 20,
                                             ((self.pathfind_to[1] - (len(self.simple_map) / 2)) / 20), 0)]
            particle_cloud.points.append(Point32((self.goal_location[0] - (len(self.simple_map) / 2)) / 20,
                                                 ((self.goal_location[1] - (len(self.simple_map) / 2)) / 20), 0))
            self.target_publisher.publish(particle_cloud)

            # self.generate_output_image()
            route = self.actually_do_a_star()

            if len(route) != 0:
                for location in range(len(route)):
                    x_pos, y_pos, rotation = self.current_location
                    x1, y1 = route[location]
                    direction = self.bearing(x_pos, y_pos, x1, y1)
                    if direction < 0:
                        direction += 360
                    while (not direction - 5 + 360 < (self.current_location[2]) % 360 + 360 < direction + 5 + 360
                           and self.PATHFINDING_COUNT == pathfinding_count):
                        print("bearing", direction, self.current_location[2])
                        base_data = Twist()
                        base_data.angular.z = 0.5
                        if self.PATHFINDING_COUNT != pathfinding_count:

                            return
                    while not x1 - 0.5 < self.current_location[0] < x1 + 0.5 or not y1 - 0.5 < self.current_location[1] < y1 + 0.5 and self.PATHFINDING_COUNT == pathfinding_count:
                        base_data = Twist()
                        base_data.linear.x = 0.2
                        self.cmd_pub.publish(base_data)
        except Exception as e:
            print("error", e)
            print(traceback.format_exc())

    def is_wall_nearby(self, x, y):  # checks if a square is nearby to a wall
        rows, cols = len(self.simple_map), len(self.simple_map[0])
        for i in range(max(0, x - 2), min(rows, x + 3)):
            for j in range(max(0, y - 2), min(cols, y + 3)):
                if self.simple_map[j][i] == 100:
                    return True

        return False

    def actually_do_a_star(self):
        path = []
        visited = []
        connections = []
        start_x, start_y, _ = self.current_location
        visited.append((start_x, start_y,))
        connections.append(0)
        end_x, end_y = self.pathfind_to
        end_condition = False
        while not end_condition:
            potential_locations = []
            for location in range(len(visited)):
                x, y = visited[location]
                for i in range(-1, 2):
                    for j in range(-1, 2):
                        coordinates = (x + i, y + j)
                        # checking the coordinates are not already visited or a wall / near a wall
                        if coordinates not in visited and not self.is_wall_nearby(x + i, y + j):
                            potential_locations.append(
                                (coordinates, get_distance(coordinates, (end_x, end_y)), location))

            # now all locations we can visit are in a list. Next we pick the one wish the smallest distance and break
            # if it is an unexplored space
            lowest = 0
            for coordinate_index in range(len(potential_locations)):
                if potential_locations[coordinate_index][1] < potential_locations[lowest][1]:
                    lowest = coordinate_index

            new_coordinates, _, new_connection = potential_locations[lowest]

            visited.append(new_coordinates)
            connections.append(new_connection)
            # next we check if we have found either an unknown square or our destination square
            if self.simple_map[new_coordinates[1]][new_coordinates[0]] == -1 or new_coordinates == (
                    end_x, end_y) or len(visited) > 100:
                end_condition = True
                # now we generate a path from the start to the final value in our visited list
                current_node = -1
                while current_node != 0:
                    path.append(visited[current_node])
                    current_node = connections[current_node]
                path.append(visited[current_node])
                path.reverse()
                return path
        return path

    def map_callback(self, msg):
        data = msg.data
        data = np.asarray(data)
        self.size = int(np.sqrt(len(data)))
        self.simple_map = np.split(data, self.size)

    def generate_output_image(self):
        robot_x, robot_y, robot_theta = self.current_location
        robot_x = round(robot_x)
        robot_y = round(robot_y)
        map_copy = copy.deepcopy(self.simple_map)
        map_copy[robot_y][robot_x] = 100
        destination_x, destination_y = self.final_destination
        map_copy[destination_y][destination_x] = 69
        for coords in self.all_visited:
            map_copy[coords[1]][coords[0]] = map_copy[coords[1]][coords[0]] + 50
        pil_image = Image.fromarray(np.uint8(map_copy))
        # pixels = pil_image.getdata()

        pil_image.save("test.png", "PNG")

    def known_objects_callback(self, msg):
        known_objs = eval(msg.data)
        if len(known_objs) >= self.object_threshold or (self.target_object is not None and self.target_object in [obj for obj, _, _ in known_objs]):
            self.threshold_met = True


if __name__ == "__main__":
    rospy.init_node('pathfinder', anonymous=True)
    try:
        ASTAR()
    except Exception as e:
        print(traceback.format_exc())
    rospy.spin()
