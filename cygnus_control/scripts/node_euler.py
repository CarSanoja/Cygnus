#!/usr/bin/env python 

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

roll = pitch = yaw = 0.0

def get_rotation (msg,args):
    global roll, pitch, yaw
    pub = args[0]
    message = args[1]
    orientation_q = msg.pose.pose.orientation
    q1=orientation_q.x
    q2=orientation_q.y
    q3=orientation_q.z
    q4=orientation_q.w
    orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
    #euler = tf.transformations.euler_from_quaternion(orientation_list)
    roll_ = math.atan2(2*(q1*q2 + q3*q4), 1- 2*(q2*q2 + q3*q3))
    roll = math.degrees(roll_)
    pitch_ = math.asin(2*(q1*q3 - q2*q4))
    pitch = math.degrees(pitch_)
    yaw_ = math.atan2(2*(q1*q4 + q2*q3),1-2*(q3*q3 + q4*q4))
    yaw = math.degrees(yaw_)
    message.pose.pose.orientation.x = roll
    message.pose.pose.orientation.y = pitch
    message.pose.pose.orientation.z = yaw
    print("quaternion")
    print(orientation_q.x,orientation_q.y,orientation_q.z,orientation_q.w)
    print(roll,pitch,yaw)
    message.pose.pose.position.x = msg.pose.pose.position.x
    message.pose.pose.position.y = msg.pose.pose.position.y
    message.pose.pose.position.z = msg.pose.pose.position.z
    message.twist.twist.angular.x = msg.twist.twist.angular.x
    message.twist.twist.angular.x = msg.twist.twist.angular.y
    message.twist.twist.angular.x = msg.twist.twist.angular.z
    #pub.publish(message)

if __name__ == '__main__':
    try:
        rospy.init_node('quaternion_to_euler')
        pub = rospy.Publisher('/cygnus/ground_truth/odometry_euler', Odometry, queue_size=10)
        odometria = Odometry()
        while not rospy.is_shutdown():
			sub = rospy.Subscriber ('/cygnus/ground_truth/odometry', Odometry, get_rotation, (pub,odometria))
    except rospy.ROSInterruptException:
        pass
