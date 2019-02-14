#!/usr/bin/env python
from PololuSMC import MotorController
from inputs import get_gamepad
import time

RX = 0
RY = 0
LX = 0
LY = 0

DX = 0
DY = 0

BA = False
BAP = False
BB = False
BBP = False
BX = False
BXP = False
BY = False
BYP = False

RB = False
RBP = False
RT = False
RTP = False
LB = False
LBP = False
LT = False
LTP = False

BACK = False
BACKP = False
START = False
STARTP = False
R3 = False
R3P = False
L3 = False
L3P = False

port_name = "/dev/ttyS0"
baud_rate = 9600
controller = MotorController(port_name, baud_rate, debug=True)

while 1:
    events = get_gamepad()
    for event in events:
        #0 to 255
        if(event.code == 'ABS_RZ'): #Right stick vertical, 0 is up
            RY = event.state
        if(event.code == 'ABS_Z'): #Right stick horizontal, 0 is left
            RX = event.state
        if(event.code == 'ABS_Y'): #Left stick vertical, 0 is up
            LY = event.state
        if(event.code == 'ABS_X'): #Right stick horizontal, 0 is left
            LX = event.state

        #-1, 0, or 1
        if(event.code == 'ABS_HAT0X'): #Dpad horizontal 1 is right
            DX = event.state
        if(event.code == 'ABS_HAT0Y'): #Dpad vertical, 1 is down
            DY = event.state

        #0 or 1 ish
        if(event.code == 'BTN_EAST'): #A button
            BAP = not BA and event.state
            BA = event.state != 0
        if(event.code == 'BTN_C'): #B button
            BBP = not BB and event.state
            BB = event.state != 0
        if(event.code == 'BTN_SOUTH'): #X button
            BXP = not BX and event.state
            BX = event.state != 0
        if(event.code == 'BTN_NORTH'): #Y button
            BYP = not BY and event.state
            BY = event.state != 0
        if(event.code == 'BTN_Z'): #Right bumper
            RBP = not RB and event.state
            RB = event.state != 0
        if(event.code == 'BTN_TR'): #Right trigger
            RTP = not RT and event.state
            RT = event.state != 0
        if(event.code == 'BTN_WEST'): #Left bumper
            LBP = not LB and event.state
            LB = event.state != 0
        if(event.code == 'BTN_TL'): #Left trigger
            LTP = not LT and event.state
            LT = event.state != 0
        if(event.code == 'BTN_TL2'): #Back button
            BACKP = not BACK and event.state
            BACK = event.state != 0
        if(event.code == 'BTN_TR2'): #Start button
            STARTP = not START and event.state
            START = event.state != 0
        if(event.code == 'BTN_START'): #R3
            R3P = not R3 and event.state
            R3 = event.state != 0
        if(event.code == 'BTN_SELECT'): #L3
            L3P = not L3 and event.state
            L3 = event.state != 0

    if(STARTP):
        controller.safe_start_all()

    MAX_SPEED = 3200
    MAX_ANG = 1
    MAX_INPUT = 255

    VX = (float(LX)/MAX_INPUT - 0.5)*2 * MAX_SPEED
    VY = (float(LY)/MAX_INPUT - 0.5)*2 * MAX_SPEED
    VA = (float(RX)/MAX_INPUT - 0.5)*2 * MAX_SPEED
    
    controller.set_all_speeds(VX, VY, VA)

    msg = """Inputs=({0},{1},{2},{3})
Velocities=({4:.2f}, {5:.2f}, {6:.2f})
({7:.0f},{8:.0f},{9:.0f},{10:.0f})
""".format(LX, LY, RX, RY, VX, VY, VA, VY-VX+VA, VY+VX-VA, VY-VX-VA, VY+VX+VA)
    #print(msg)
