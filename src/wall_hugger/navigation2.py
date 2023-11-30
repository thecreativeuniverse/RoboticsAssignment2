#!/usr/bin/env python3
import rospy
import random
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, String
from nav_msgs.msg import Odometry
from tf.msg import tfMessage
from copy import deepcopy

object_threshold = 10
threshold_met = False
odom_recent = None
cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
odom_pub = rospy.Publisher("estimated_pose", PoseStamped, queue_size=1)
prior_z = 0

'''
    Called everytime we receive laser readings, checks if the object threshold has been met, and 
    executes discovery_algorithm to decide how to move if the threshold hasn't been met.
'''
def callback(msg):
    # Averages: right, middle right, middle, middle left, left
    averages = [msg.data[4], msg.data[5], msg.data[6], msg.data[7], msg.data[8]]
    base_data = Twist()

    # TODO: Prioritise entering nearby doorways.

    if not threshold_met:
        discovery_algorithm(base_data, averages)

#Tells the robot how to move based on laser inputs.
def discovery_algorithm(base_data, averages):
    ''' 
    If:
    -   the lasers detect an average of below 0.4 in the forward facing or middle-right laser cones or below 0.45 on the 
    far right laser cone, reverse immediately then stop and rotate left to avoid them.
    -   the lasers detect an average of below 0.4 in the middle-left laser cones or below 0.45 on the far right left 
    laser cone, reverse immediately then stop and rotate right to avoid them.
    -   the lasers detect an average of equal to or above 0.4 in the forward facing laser cone, move forwards but slowly drift the 
    angular velocity so the robot does some exploration in a non-straight line.
    '''
    global prior_z

    if averages[0] < 0.45 or averages[1] < 0.4 or averages[2] < 0.4:
        base_data.linear.x = -1
        base_data.angular.z = 0.5
        cmd_pub.publish(base_data)
        prior_z = 0

    elif averages[3] < 0.4 or averages[4] < 0.45:
        base_data.linear.x = -1
        base_data.angular.z = -0.5
        cmd_pub.publish(base_data)
        prior_z = 0

    elif averages[2] >= 0.4:
        base_data.linear.x = 0.5
        if ((random.random() * 100) >= 90):
            prior_z = 0

        if (prior_z >= 0.02):
            base_data.angular.z = prior_z + (random.random() * 0.01)
        elif (prior_z <= -0.02):
            base_data.angular.z = prior_z + (random.random() * -0.01) 
        else:
            base_data.angular.z = (random.random() * 0.2) - 0.1
        prior_z = base_data.angular.z

    cmd_pub.publish(base_data)

'''
    Whenever the odom topic is published to, update the odom_recent item to contain the most recent odom object
'''
def odom_callback(msg):
    global odom_recent
    odom_recent = msg

'''
    This performs the pose estimation of the robot. It is called everytime a new transformation is published by gmapping.
    It filters to only use transformations that put poses on the odometry frame onto the gmmaping map frame. It then creates
    a copy of the most recent odometry published, applies the position translation, then applies the orientation rotation
    by using a quaternion multiplication, before putting the transformed values into a PoseStamped varaible. This variable
    now contains the transformed value, that is, the pose of the robot in the map frame. This is then published using the 
    odom_pub publisher.
'''
def tf_callback(msg):
    if (msg.transforms[0].child_frame_id != "odom" or msg.transforms[0].header.frame_id != "map"):
        return
    
    odom = deepcopy(odom_recent)

    translation = msg.transforms[0].transform.translation
    rotation = msg.transforms[0].transform.rotation

    odom.pose.pose.position.x += translation.x
    odom.pose.pose.position.y += translation.y
    odom.pose.pose.position.z += translation.z

    odom_orientation = [odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z]
    rotation = [rotation.w, rotation.x, rotation.y, rotation.z]
    estimated_orientation = quaternion_multiply(odom_orientation, rotation)
    odom.pose.pose.orientation.w = estimated_orientation[0]
    odom.pose.pose.orientation.x = estimated_orientation[1]
    odom.pose.pose.orientation.y = estimated_orientation[2]
    odom.pose.pose.orientation.z = estimated_orientation[3]

    estimated_pose = PoseStamped()
    estimated_pose.header.frame_id = "map"
    estimated_pose.header.stamp = rospy.Time.now()
    estimated_pose.pose = odom.pose.pose
    odom_pub.publish(estimated_pose)

# This performs Quaternion multiplication. It is used to apply the rotation in the transformation to get the robot pose.
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

'''
    This is called whenever the known_objects topic gets published to. It checks how many objects have been found, and updates the
    threshold_met variable if the object_threshold value has been met. 
'''
def known_objects_callback(msg):
    if len(eval(msg.data)) >= object_threshold:
        global threshold_met
        threshold_met = True


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
