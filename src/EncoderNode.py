#!/usr/bin/env python
from RPi import GPIO
import time
import rospy
from std_msgs.msg import Int16MultiArray

# Channel A pins (yellow)
Apins = [27, 19, 16 ,6]
# Channel B pins (green)
Bpins = [22, 26, 20, 13]

# Channel A state
Astate = [False] * 4
# Channel B state
Bstate = [False] * 4

# Encoder counts
count = [0] * 4

def callback(channel):
    pin = Apins.index(channel)
    rospy.loginfo("Callback on channel %d pin %d", channel, pin)
    if(GPIO.input(Bpins[pin])):
        count[pin] -= 1
    else:
        count[pin] += 1

def init_node():
    global pub

    GPIO.setmode(GPIO.BCM)
    for i in range(4):
        GPIO.setup(Apins[i], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(Bpins[i], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(Apins[i], GPIO.FALLING, callback=callback)  


    rospy.init_node('encoder_reader')
    pub = rospy.Publisher("/motor_status/encoder_counts", Int16MultiArray, queue_size=1)
    rate = rospy.Rate(1000)
    while not rospy.core.is_shutdown():
        array = Int16MultiArray()
        array.data = count
        pub.publish(array)
        rate.sleep()

if __name__ == '__main__':
    init_node()

