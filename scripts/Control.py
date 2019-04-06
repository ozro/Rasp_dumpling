#!/usr/bin/env python
from PololuSMC import MotorController
import time

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

def callback(twist):
    speeds = [0] * 4
    speeds[0] = twist.linear.x-twist.linear.y - twist.angular.z
    speeds[1] = twist.linear.x+twist.linear.y - twist.angular.z
    speeds[2] = - twist.linear.x+twist.linear.y - twist.angular.z
    speeds[3] = - twist.linear.x-twist.linear.y - twist.angular.z
    array = Float32MultiArray()
    array.data = speeds 
    motor_pub.publish(array)
def start_node():
    global motor_pub
    motor_pub = rospy.Publisher('motor_vel', Float32MultiArray, queue_size=1)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.init_node("Control")
    rospy.spin()
if __name__ == '__main__':
    start_node()
