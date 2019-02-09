#!/usr/bin/env python
from PololuSMC import MotorController
import time
import rospy
from std_msgs.msg import Float32MultiArray

port_name = "/dev/ttyS0"
baud_rate = 9600

controller = MotorController(port_name, baud_rate, debug=True)
controller.safe_start_all()
time.sleep(0.2)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data[0])
    for motor in range(4):
        controller.set_target_speed(motor, data.data[i])
        
    time.sleep(0.2)
    for motor in range(4):
        controller.set_target_speed(motor, 0)
    
def listen_control():

    rospy.init_node('listen_control', anonymous=True)

    rospy.Subscriber("/mecanum_cmd", Float32MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listen_control()





