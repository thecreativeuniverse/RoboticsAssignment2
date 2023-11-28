#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from std_msgs.msg import Float64MultiArray, String
from nav_msgs.msg import Odometry
from tf.msg import tfMessage
from copy import copy

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
        base_data.linear.x = -5
        cmdPub.publish(base_data)
        rospy.sleep(0.1)
        base_data.linear.x = 0
        base_data.angular.z = 0.75
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
        base_data.angular.z += (random.random() * 0.5) - 0.25

    elif (abs(odomRecent[0].pose.pose.position.x - odomRecent[1].pose.pose.position.x) <= 0.01) & (
            abs(odomRecent[0].pose.pose.position.y - odomRecent[1].pose.pose.position.y) <= 0.01):
        base_data.linear.x = -0.5
        base_data.angular.z += (random.random() * 0.5) - 0.25

    else:
        base_data.linear.x = -1

    cmdPub.publish(base_data)


# def odom_callback(msg):
#     global odomRecent
#     if len(odomRecent) < 5:
#         odomRecent.append(msg)
#     else:
#         odomRecent.pop(0)
#         odomRecent.append(msg)


def tf_callback(msg):
    odom = copy(odomRecent[0])

    if (msg.transforms[0].child_frame_id != "odom" or msg.transforms[0].header.frame_id != "map"):
        return

    translation = msg.transforms[0].transform.translation
    rotation = msg.transforms[0].transform.rotation

    odom.pose.pose.position.x += translation.x
    odom.pose.pose.position.y += translation.y
    odom.pose.pose.position.z += translation.z

    odom.pose.pose.orientation.w += rotation.w
    odom.pose.pose.orientation.x += rotation.x
    odom.pose.pose.orientation.y += rotation.y
    odom.pose.pose.orientation.z += rotation.z

    poseStamped = PoseStamped()
    poseStamped.header.frame_id = "map"
    poseStamped.header.stamp = rospy.Time.now()
    poseStamped.pose = odom.pose.pose
    odomPub.publish(poseStamped)


def known_objects_callback(msg):
    if len(eval(msg.data)) >= objectThreshold:
        global thresholdMet
        thresholdMet = True


def listener():
    rospy.init_node("Navigation", anonymous=True)
    rospy.Subscriber('proximity_sensor', Float64MultiArray, callback)
    # rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.Subscriber('tf', tfMessage, tf_callback)
    rospy.Subscriber('known_objects', String, known_objects_callback)

    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
