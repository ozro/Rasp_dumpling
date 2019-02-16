#!/usr/bin/env python
from PololuSMC import MotorController
import time
import rospy
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16MultiArray

port_name = "/dev/ttyS0"
baud_rate = 9600
max_speed = 1

def callback(data):
    global controller
    for motor in range(4):
        speed = int(round(data.data[motor]/max_speed * 3200))
        rospy.loginfo(rospy.get_caller_id() + " says set motor %d to %d (%.4f rad/s)", motor,speed,data.data[motor])
        controller.set_target_speed(motor, speed)
    
def init_control():
    global controller
    controller = MotorController(port_name, baud_rate, debug=True)
    controller.safe_start_all()

    rospy.init_node('motor_control')

    #rospy.Subscriber("/motor_vel", Float32MultiArray, callback)
    error_pub = rospy.Publisher("/motor_status/errors", UInt16MultiArray, queue_size=10)

    rate = rospy.Rate(5)
    while not rospy.core.is_shutdown():
        status = [0] * 4
        for motor in range(4):
            status[motor] = controller.get_error_status(motor)

        array = UInt16MultiArray()
        array.data = status
        error_pub.publish(array) 
        
        rate.sleep()

if __name__ == '__main__':
    init_control()





