#!/usr/bin/env python
from PololuSMC import MotorController
import time

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def callback(data):
    global isBraked
    global isStopped
    global isStart
    global vel_pub
    global motor_pub
    global start_pub
    global brake_pub
    if(isBraked and not data.buttons[2]): #Reset brake flag
        isBraked = False
    if(isStopped and not data.buttons[8]): #Reset stop flag
        isStopped = False
    if(isStart and not data.buttons[9]): #Reset stop flag
        isStart = False

    if(not isStopped and data.buttons[8]): #Back button down
        isStopped = True
        start = Bool()
        start.data = False
        start_pub.publish(start)
    if(not isStart and not data.buttons[8] and data.buttons[9]): #Start button down
        isStart = True
        start = Bool()
        start.data = True
        start_pub.publish(start)
    if(not isBraked and data.buttons[2]): #B button down
        isBraked = True
        brake = Bool()
        brake.data = True
        brake_pub.publish(brake)

    if(not isBraked):
        max_speed = 1
        max_turn = 1
        twist = Twist()
        twist.linear.x = data.axes[1] * max_speed
        twist.linear.y = data.axes[0] * max_speed 
        twist.angular.z = data.axes[2] * -1 * max_turn 
        vel_pub.publish(twist)	


        speeds = [0] * 4
        speeds[0] = twist.linear.x-twist.linear.y - twist.angular.z
        speeds[1] = twist.linear.x+twist.linear.y - twist.angular.z
        speeds[2] = - twist.linear.x+twist.linear.y - twist.angular.z
        speeds[3] = - twist.linear.x-twist.linear.y - twist.angular.z
        array = Float32MultiArray()
        array.data = speeds 
        motor_pub.publish(array)	
    
def start_node():
    global isBraked
    global isStopped
    global isStart
    global vel_pub
    global motor_pub
    global start_pub
    global brake_pub

    isBraked = False
    isStopped = False
    isStart = False

    vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    motor_pub = rospy.Publisher('motor_vel', Float32MultiArray, queue_size=1)
    start_pub = rospy.Publisher('motor_cmd/start', Bool, queue_size=10)
    brake_pub = rospy.Publisher('motor_cmd/brake', Bool, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)

    rospy.init_node("Teleop")
    rospy.spin()

if __name__ == '__main__':
    start_node()
