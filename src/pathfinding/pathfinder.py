#!/usr/bin/env python3
import rospy
import math
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud
from tf.transformations import euler_from_quaternion

from PIL import Image


def get_distance(start, end):
    start_x, start_y = start
    end_x, end_y = end
    distance = math.sqrt((start_x + end_x) ** 2 + (start_y + end_y) ** 2)


class ASTAR:
    def __init__(self):
        # subscribes to map
        rospy.Subscriber("map", OccupancyGrid, self.map_callback, queue_size=1)
        # subscribes to final destination
        rospy.Subscriber("goal_position", PointCloud, self.destination_callback, queue_size=1)
        # subscribes to robot pos
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)

        self.simple_map = []  # 2d map of space
        self.final_destination = []  # actual place we are trying to go
        self.current_location = []  # where the robot is, and orientation
        self.current_destination = [69, 420]  # where we can currently navigate to
        self.size = 0  # size of map
        rospy.spin()

    def destination_callback(self, msg):  # takes the x,y coordinates of from goal position and saves them to a variable
        x = round(self.size / 2 + msg.points[0].x)
        y = round(self.size / 2 + msg.points[0].y)

        self.final_destination = [x, y]

    def odom_callback(self, msg):
        # get robot coordinates
        x, y = msg.pose.pose.position.x * 20, msg.pose.pose.position.y * 20

        # get robot bearing
        quaternion = (0, 0, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        degrees = 360 - (270 + ((180 * euler[2]) / math.pi)) % 360

        # set them to current location
        self.current_location = [round(self.size / 2 + x), round(self.size / 2 + y), degrees]

        # calculate the distance to the target we can navigate to
        distance_from_destination = math.sqrt((x + self.current_location[0]) ** 2 + (y + self.current_location[1]) ** 2)

        # determine if we have just started or if we have reached our target we can navigate to:
        # just started
        if self.current_destination[0] == 69 and self.current_destination[1] == 420:
            self.calculate_path()

        # we have reached our target we can navigate to
        elif distance_from_destination < 0.5:
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
        route = self.actually_do_a_star()
        simple_route = self.simplify_route(route)
        pass

    def simplify_route(self,route):
        line_start = route[0]
        line_end = route[1]
        trial_end = route[2]


        return route

    def is_wall_nearby(self, x, y):  # checks if a square is nearby to a wall
        rows, cols = len(self.simple_map), len(self.simple_map[0])
        for i in range(max(0, x - 2), min(rows, x + 3)):
            for j in range(max(0, y - 2), min(cols, y + 3)):
                if self.simple_map[i][j] == 100:
                    return True

        return False

    def actually_do_a_star(self):
        path = []
        visited = []
        connections = []
        start_x, start_y, _ = self.current_location
        visited.append((start_x, start_y,))
        connections.append(0)
        end_x, end_y = self.final_destination
        end_condition = False
        while not end_condition:
            potential_locations = []
            for location in range(len(visited)):
                x, y = visited[location]
                for i in range(-1, 1):
                    for j in range(-1, 1):
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
            if self.simple_map[new_coordinates[0]][new_coordinates[1]] == -1 or new_coordinates == (end_x, end_y):
                end_condition = True
                # now we generate a path from the start to the final value in our visited list
                current_node = -1
                while current_node !=0:
                    path.append(visited[current_node])
                    current_node = connections[current_node]
                path.append(visited[current_node])

        path = path.reverse()
        return path

    def map_callback(self, msg):
        data = msg.data
        data = np.asarray(data)
        self.size = int(np.sqrt(len(data)))
        self.simple_map = np.split(data, self.size)
        self.generate_output_image()

    def generate_output_image(self):
        print("@@@@@@@@@@@@@@@@@@@@@@@")
        # -1 = unvisited
        # 100 = wall
        # 0 = floor
        print(self.current_location)
        print(self.final_destination)
        robot_x, robot_y, robot_theta = self.current_location
        robot_x = round(robot_x)
        robot_y = round(robot_y)
        self.simple_map[robot_x][robot_y] = 200
        pil_image = Image.fromarray(np.uint8(self.simple_map))
        # pixels = pil_image.getdata()
        pil_image.save("test.png", "PNG")


if __name__ == "__main__":
    rospy.init_node('ASTAR', anonymous=True)

    ASTAR()
