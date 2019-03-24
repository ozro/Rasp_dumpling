#!/usr/bin/env python
from rotary_encoder import decoder
import time
import pigpio
import rospy
from rasp.msg import EncoderCounts

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

# Decoders
decoders = [None] * 4

def callback(way, gpio):
    motor = Apins.index(gpio)
    count[motor] += way

def init_node():
    pi = pigpio.pi()
    for i in range(4):
        decoders[i] = decoder(pi, Apins[i], Bpins[i], callback)

    rospy.init_node('encoder_reader')
    pub = rospy.Publisher("/motor_status/encoder_counts", EncoderCounts, queue_size=10)
    rate = rospy.Rate(10000)
    rospy.loginfo("Initialized encoders")
    while not rospy.core.is_shutdown():
        array = EncoderCounts()
        array.header.stamp = rospy.get_rostime()
        array.counts = count
        pub.publish(array)
        rate.sleep()

    for dec in decoders:
        dec.cancel()
    pi.stop()

if __name__ == '__main__':
    init_node()

