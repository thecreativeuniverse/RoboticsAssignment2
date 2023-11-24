#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time

pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)

def callback(msg):
    lowest = [msg.data[0], msg.data[1]]
    highest = [msg.data[2], msg.data[3]]

    # Averages: right, middle right, middle, middle left, left
    averages = [msg.data[4], msg.data[5], msg.data[6], msg.data[7], msg.data[8]]

    # position = msg.data[1]

    base_data = Twist()

    # TODO: implement crash recovery, it cant just contemplate life in front of the wall
    # TODO: Subscribe to object count use this as a basis for when to switch to A* searching after meeting a threshold of the number of objects found
    # TODO: If statement that does crash recovery deals well with walls, but not doorways, need to add separate condition that is prior to this that deals with entering doorways.
    # TODO: Add some random rotational movement in rooms so that it will "discover" the middles of rooms
    # TODO: Add publisher which converts gmapping localisation transformation function to actual location
    """
    This section is used to determine the direction the robot will go
    If the robot sees that...
        the space infront of it is empty then it will keep going straight
        there is a wall to the right and middle right of it, it will turn to go to another direction (its in a corner)
        there is a wall to the left and middle left of it, it will turn to go to another direction (its in a corner)
        there is a wall right in front of it, turn left right or backwards based on randomnesss
    """

    # Crash recovery first
    if averages[0] < 0.5 or averages[1] < 0.4 or averages[2] < 0.4 or averages[3] < 0.4 or averages[4] < 0.5:
    # elif lowest[0] < 0.3:
        base_data.linear.x = -100
        pub.publish(base_data)
        rospy.sleep(0.1)
        base_data.linear.x = 0
        base_data.angular.z = -5
        pub.publish(base_data)
        rospy.sleep(0.1)
        # sleep for 5 seconds
    elif averages[2] > 0.5:
        base_data.linear.x = 0.5
    elif averages[0] > 0.5 or averages[1] > 0.5:
        base_data.linear.x = 0
        base_data.angular.z = -1.5
    elif averages[3] > 0.5 or averages[4] > 0.5:
        base_data.linear.x = 0
        base_data.angular.z = 1.5
    elif averages[2] <= 0.5:
        # direction = random.randint(1, 3)
        base_data.linear.x = 0
        # if direction == 1:
            # base_data.angular.z = 1
        # elif direction == 2: 
        base_data.angular.z = 1
        # else:
        #     base_data.angular.z = -1
    else:
        # base_data.angular.z = 5
        base_data.linear.x = 1

    # base_data.linear.x *= 1.5
    #    else:
    #        base_data.angular.z = -4
    #    base_data.linear.x = 0
    pub.publish(base_data)
    # else:
    #    base_data.linear.x = 1
    #    pub.publish(base_data)
    # rospy.sleep(rospy.Duration(1, 0))

def listener():
    rospy.init_node("Navigation", anonymous=True)
    rospy.Subscriber('proximity_sensor', Float64MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
