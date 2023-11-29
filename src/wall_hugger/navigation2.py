#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, String
from nav_msgs.msg import Odometry
from tf.msg import tfMessage
from copy import deepcopy
import numpy as np

objectThreshold = 5
thresholdMet = False
odomRecent = []
cmdPub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
odomPub = rospy.Publisher("estimated_pose", PoseStamped, queue_size=1)


def callback(msg):
    # Averages: right, middle right, middle, middle left, left
    averages = [msg.data[4], msg.data[5], msg.data[6], msg.data[7], msg.data[8]]
    base_data = Twist()

    # TODO: Subscribe to object count use this as a basis for when to switch to A* searching after meeting a threshold of the number of objects found
    # TODO: Prioritise entering nearby doorways.

    if thresholdMet:
        discoveryAlgorithm(base_data, averages)
    else:
        discoveryAlgorithm(base_data, averages)


def discoveryAlgorithm(base_data, averages):
    # Crash recovery first
    if averages[0] < 0.45 or averages[1] < 0.4 or averages[2] < 0.4:
        base_data.linear.x = -1
        cmdPub.publish(base_data)
        rospy.sleep(0.1)
        base_data.linear.x = 0
        base_data.angular.z = 0.5
        cmdPub.publish(base_data)
        rospy.sleep(0.1)

    elif averages[3] < 0.4 or averages[4] < 0.45:
        base_data.linear.x = -1
        cmdPub.publish(base_data)
        rospy.sleep(0.1)
        base_data.linear.x = 0
        base_data.angular.z = -0.5
        cmdPub.publish(base_data)
        rospy.sleep(0.1)

    elif averages[2] >= 0.4:
        base_data.linear.x = 0.5
        base_data.angular.z += (random.random() * 0.5) - 0.25

    elif (abs(odomRecent[0].pose.pose.position.x - odomRecent[1].pose.pose.position.x) <= 0.01) & (
            abs(odomRecent[0].pose.pose.position.y - odomRecent[1].pose.pose.position.y) <= 0.01):
        base_data.linear.x = -0.5
        base_data.angular.z += (random.random() * 0.5) - 0.25

    else:
        base_data.linear.x = -1

    cmdPub.publish(base_data)


def odom_callback(msg):
    global odomRecent
    if len(odomRecent) < 5:
        odomRecent.append(msg)
    else:
        odomRecent.pop(0)
        odomRecent.append(msg)


def tf_callback(msg):
    odom = deepcopy(odomRecent[0])

    if (msg.transforms[0].child_frame_id != "odom" or msg.transforms[0].header.frame_id != "map"):
        return

    translation = msg.transforms[0].transform.translation
    rotation = msg.transforms[0].transform.rotation

    odom.pose.pose.position.x += translation.x
    odom.pose.pose.position.y += translation.y
    odom.pose.pose.position.z += translation.z

    odomOrientation = [odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z]
    rotation = [rotation.w, rotation.x, rotation.y, rotation.z]
    orientation = quaternion_multiply(odomOrientation, rotation)
    odom.pose.pose.orientation.w = orientation[0]
    odom.pose.pose.orientation.x = orientation[1]
    odom.pose.pose.orientation.y = orientation[2]
    odom.pose.pose.orientation.z = orientation[3]

    poseStamped = PoseStamped()
    poseStamped.header.frame_id = "map"
    poseStamped.header.stamp = rospy.Time.now()
    poseStamped.pose = odom.pose.pose
    odomPub.publish(poseStamped)

# Code taken from https://automaticaddison.com/how-to-multiply-two-quaternions-together-using-python/ 29/11/23 12:32pm GMT
def quaternion_multiply(Q0,Q1):
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
     
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]
     
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
     
    final_quaternion = [Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z]

    return final_quaternion

def known_objects_callback(msg):
    if len(eval(msg.data)) >= objectThreshold:
        global thresholdMet
        thresholdMet = True


def listener():
    rospy.init_node("Navigation", anonymous=True)
    rospy.Subscriber('proximity_sensor', Float64MultiArray, callback)
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.Subscriber('tf', tfMessage, tf_callback)
    rospy.Subscriber('known_objects', String, known_objects_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
