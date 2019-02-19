#!/usr/bin/env python
from PololuSMC import MotorController
import time
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Int16MultiArray

port_name = "/dev/ttyS0"
baud_rate = 9600
max_speed = 1

def vel_callback(data):
    global controller
    for motor in range(4):
        speed = int(round(data.data[motor]/max_speed * 3200))
        controller.set_target_speed(motor, speed)

def start_callback(data):
    global controller
    if(data.data):
        rospy.loginfo("Starting all motors")
        controller.safe_start_all()
    else:
        rospy.loginfo("Stopping all motors")
        controller.stop_all()

def brake_callback(data):
    global controller
    if(data.data):
        rospy.loginfo("Braking all motors")
        controller.brake_all()
    
def init_control():
    global controller

    rospy.init_node('motor_control')

    rospy.Subscriber("/motor_vel", Float32MultiArray, vel_callback)
    rospy.Subscriber("/motor_cmd/start", Bool, start_callback)
    rospy.Subscriber("/motor_cmd/brake", Bool, brake_callback)

    error_pub = rospy.Publisher("/motor_status/errors", UInt16MultiArray, queue_size=10)
    target_pub = rospy.Publisher("/motor_status/target_speeds", Int16MultiArray, queue_size=10)
    speed_pub = rospy.Publisher("/motor_status/current_speeds", Int16MultiArray, queue_size=10)

    controller = MotorController(port_name, baud_rate, debug=True)

    rate = rospy.Rate(10)
    while not rospy.core.is_shutdown():

        pub_errors(error_pub)
        pub_targets(target_pub)
        #pub_speeds(speed_pub)
        
        rate.sleep()

def pub_errors(pub):
    status = [0] * 4
    for motor in range(4):
        status[motor] = controller.get_error_status(motor)

    array = UInt16MultiArray()
    array.data = status
    pub.publish(array) 

def pub_targets(pub):
    speeds = [0] * 4
    for motor in range(4):
        speeds[motor] = controller.get_target_speed(motor)

    array = Int16MultiArray()
    array.data = speeds
    pub.publish(array) 

def pub_speeds(pub):
    speeds = [0] * 4
    for motor in range(4):
        speeds[motor] = controller.get_current_speed(motor)

    array = Int16MultiArray()
    array.data = speeds
    pub.publish(array) 

if __name__ == '__main__':
    init_control()

