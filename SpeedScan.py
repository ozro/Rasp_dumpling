from PololuSMC import MotorController
import time

port_name = "/dev/ttys0"
baud_rate = 9600

controller = MotorController(port_name, baud_rate, debug=True)

while(True):
    for i in range(-3200, 3200):
        controller.safe_start_all()
        controller.set_target_speed(0, i)
        time.sleep(0.2)
    for i in range(3200, -3200, -1):
        controller.safe_start_all()
        controller.set_target_speed(0, i)
        time.sleep(0.2)
