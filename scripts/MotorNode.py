#!/usr/bin/env python
from PololuSMC import MotorController
import time
import rospy
import sys
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Int16MultiArray

port_name = "/dev/ttyS0"
baud_rate = 9600
max_speed = 1

def vel_callback(data):
    global controller
    for motor in range(4):
        speed = int(round(data.data[motor]/max_speed * 1600))
        controller.set_target_speed(motor, speed)

def tray_callback(data):
    global controller
    speed = int(round(data.data/max_speed * 3200))
    controller.set_target_speed(4, speed)

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

    controller = MotorController(port_name, baud_rate, debug=True)
    controller.stop_all()

    rospy.Subscriber("/motor_vel", Float32MultiArray, vel_callback, queue_size=3)
    rospy.Subscriber("/tray_vel", Float32, tray_callback, queue_size=1)
    rospy.Subscriber("/motor_cmd/start", Bool, start_callback, queue_size=1)
    rospy.Subscriber("/motor_cmd/brake", Bool, brake_callback, queue_size=10)

    error_pub = rospy.Publisher("/motor_status/errors", UInt16MultiArray, queue_size=1)
    limit_pub = rospy.Publisher("/motor_status/limit_status", UInt16MultiArray, queue_size=1)
    target_pub = rospy.Publisher("/motor_status/target_speeds", Int16MultiArray, queue_size=1)
    speed_pub = rospy.Publisher("/motor_status/current_speeds", Int16MultiArray, queue_size=1)
    temp_pub = rospy.Publisher("/motor_status/temps", UInt16MultiArray, queue_size=1)
    curr_pub = rospy.Publisher("/motor_status/currents", UInt16MultiArray, queue_size=1)
    volt_pub = rospy.Publisher("/motor_status/voltages", UInt16MultiArray, queue_size=1)

    rate = rospy.Rate(1000)
    cycle = 10
    while not rospy.core.is_shutdown():
        try:

            if(cycle % 1 == 0):
                pub_speeds(speed_pub)
            if((cycle-1) % 2 == 0):
                pub_errors(error_pub)
            if((cycle-2) % 100 == 0):
                pub_limits(limit_pub)
            if((cycle-3) % 1 == 0):
                pub_targets(target_pub)
            if((cycle-4) % 50 == 0):
                pub_temps(temp_pub)
            if((cycle-5) % 5 == 0):
                pub_curr(curr_pub)
            if((cycle-6) % 5 == 0):
                pub_volt(volt_pub)
        except:
            e = sys.exc_info()[0]
            rospy.loginfo("Error:%s" % e)

        cycle += 1
        if(cycle > 500):
            cycle = cycle % 500
        rate.sleep()

def pub_errors(pub):
    status = [0] * 5
    for motor in range(5):
        status[motor] = controller.get_error_status(motor)

    array = UInt16MultiArray()
    array.data = status
    pub.publish(array) 

def pub_limits(pub):
    status = [0] * 5
    for motor in range(5):
        status[motor] = controller.get_limit_status(motor)

    array = UInt16MultiArray()
    array.data = status
    pub.publish(array) 

def pub_targets(pub):
    speeds = [0] * 5
    for motor in range(5):
        speeds[motor] = controller.get_target_speed(motor)

    array = Int16MultiArray()
    array.data = speeds
    pub.publish(array) 

def pub_speeds(pub):
    speeds = [0] * 5
    for motor in range(5):
        speeds[motor] = controller.get_current_speed(motor)

    array = Int16MultiArray()
    array.data = speeds
    pub.publish(array) 

def pub_temps(pub):
    temps = [0] * 5
    for motor in range(5):
        (temp_A, temp_B) = controller.get_temperatures(motor)
        temps[motor] =(temp_A + temp_B)/2

    array = UInt16MultiArray()
    array.data = temps
    pub.publish(array) 

def pub_curr(pub):
    curr = [0] * 5
    for motor in range(5):
        curr[motor] = controller.get_current(motor)

    array = UInt16MultiArray()
    array.data = curr
    pub.publish(array) 

def pub_volt(pub):
    volt = [0] * 5
    for motor in range(5):
        volt[motor] = controller.get_input_voltage(motor)

    array = UInt16MultiArray()
    array.data = volt
    pub.publish(array) 

if __name__ == '__main__':
    init_control()

