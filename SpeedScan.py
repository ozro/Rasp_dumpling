from PololuSMC import MotorController
import time

port_name = "/dev/ttyS0"
baud_rate = 9600

controller = MotorController(port_name, baud_rate, debug=True)
controller.safe_start_all()
while(True):
    for i in range(-3200, 3200):
        print("setting speed to {}".format(i))
        controller.set_target_speed(0, i)
        controller.set_target_speed(1, -1*i)
        time.sleep(0.01)
    for i in range(3200, -3200, -1):
        print("setting speed to {}".format(i))
        controller.set_target_speed(0, i)
        controller.set_target_speed(1, -1*i)
        time.sleep(0.01)
