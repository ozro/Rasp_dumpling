#!/usr/bin/env python
from PololuSMC import MotorController
import time

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    max_speed = 0.2 #in m/s
    max_turn = 0.1  #in rad/s
    twist = Twist()
    twist.linear.x = data.axes[1] * max_speed
    twist.linear.y = data.axes[0] * -1 * max_speed 
    twist.angular.z = data.axes[2] * max_turn 
    vel_pub.publish(twist)	


    speeds = [0] * 4
    speeds[0] = twist.linear.y-twist.linear.x + twist.angular.z
    speeds[1] = twist.linear.y+twist.linear.x - twist.angular.z
    speeds[2] = twist.linear.y-twist.linear.x - twist.angular.z
    speeds[3] = twist.linear.y+twist.linear.x + twist.angular.z
    array = Float32MultiArray()
    array.data = speeds 
    motor_pub.publish(array)	
    
def start_node():
    global vel_pub
    global motor_pub
    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    motor_pub = rospy.Publisher('motor_vel', Float32MultiArray, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)

    rospy.init_node("Teleop")
    rospy.spin()

if __name__ == '__main__':
    start_node()
