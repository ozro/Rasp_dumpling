from PololuSMC import MotorController
import time

port_name = "/dev/ttyS0"
baud_rate = 9600

controller = MotorController(port_name, baud_rate, debug=True)
controller.safe_start_all()
for motor in range(4):
    for i in range(-3200, 3200, 320):
        controller.set_target_speed(motor, i)
        #controller.get_current(motor, True)
        time.sleep(0.2)
    for i in range(10):
        time.sleep(0.2)
    for i in range(3200, 0, -320):
        controller.set_target_speed(motor, i)
        #controller.get_current(motor, True)
        time.sleep(0.1)
    controller.set_target_speed(motor, 0)

