#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs import TransformStamped
import time

odomRecent = []
cmdPub = rospy.Publisher("cmd_vel", Twist, queue_size=100)

def callback(msg):
    lowest = [msg.data[0], msg.data[1]]
    highest = [msg.data[2], msg.data[3]]

    # Averages: right, middle right, middle, middle left, left
    averages = [msg.data[4], msg.data[5], msg.data[6], msg.data[7], msg.data[8]]

    base_data = Twist()

    # TODO: Subscribe to object count use this as a basis for when to switch to A* searching after meeting a threshold of the number of objects found
    # TODO: Add publisher which converts gmapping localisation transformation function to actual location
    # TODO: Further Add some random rotational movement in rooms so that it will "discover" the middles of rooms
    """
    This section is used to determine the direction the robot will go
    If the robot sees that...
        the space infront of it is empty then it will keep going straight
        there is a wall to the right and middle right of it, it will turn to go to another direction (its in a corner)
        there is a wall to the left and middle left of it, it will turn to go to another direction (its in a corner)
        there is a wall right in front of it, turn left right or backwards based on randomnesss
    """

    # Crash recovery first
    if averages[0] < 0.45 or averages[1] < 0.4 or averages[2] < 0.4:
        base_data.linear.x = -5
        cmdPub.publish(base_data)
        rospy.sleep(0.1)
        base_data.linear.x = 0
        base_data.angular.z = 0.5
        cmdPub.publish(base_data)
        rospy.sleep(0.1)

    elif averages[3] < 0.4 or averages[4] < 0.45:
        base_data.linear.x = -5
        cmdPub.publish(base_data)
        rospy.sleep(0.1)
        base_data.linear.x = 0
        base_data.angular.z = -0.5
        cmdPub.publish(base_data)
        rospy.sleep(0.1)

    elif averages[2] >= 0.4:
        base_data.linear.x = 0.5
        base_data.angular.z += (random.random() *0.5) -0.25

    elif (abs(odomRecent[0].position.x - odomRecent[1].position.x) <= 0.01) & (abs(odomRecent[0].position.y - odomRecent[1].position.y) <= 0.01):
        base_data.linear.x = -0.5
        base_data.angular.z += (random.random() *0.5) -0.25

    else:
        base_data.linear.x = -1

    cmdPub.publish(base_data)

def odomCallback(msg):
    if (len(odomRecent) < 5):
        odomRecent.append(msg.pose.pose)
    else:
        odomRecent.pop(0)
        odomRecent.append(msg.pose.pose)

def listener():
    rospy.init_node("Navigation", anonymous=True)
    rospy.Subscriber('proximity_sensor', Float64MultiArray, callback)
    rospy.Subscriber('odom', Odometry, odomCallback )
    rospy.Subscriber('tf', TransformStamped, tfCallback)
    odomPub = rospy.Publisher("estimated_pose", Odometry, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
