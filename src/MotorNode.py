#!/usr/bin/env python
from PololuSMC import MotorController
import time
import rospy
from std_msgs.msg import Float32MultiArray

port_name = "/dev/ttyS0"
baud_rate = 9600

def callback(data):
    for motor in range(4):
        rospy.loginfo(rospy.get_caller_id() + " says set motor %s to speed %s", motor,data.data[motor])
        controller.set_target_speed(motor, data.data[i])
    
def init_control():

    controller = MotorController(port_name, baud_rate, debug=True)
    controller.safe_start_all()
    rospy.init_node('motor_control')

    rospy.Subscriber("/motor_vel", Float32MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    init_control()




