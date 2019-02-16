from PololuSMC import MotorController
import time

port_name = "/dev/ttyS0"
baud_rate = 9600

controller = MotorController(port_name, baud_rate, debug=True)
time.sleep(0.5)
controller.get_error_status(0, log=True)
time.sleep(0.2)
time.sleep(0.5)
controller.safe_start_all()
time.sleep(0.2)
controller.get_error_status(0, log=True)
time.sleep(0.5)
controller.get_error_status(0, log=True)
time.sleep(0.2)
time.sleep(0.5)
controller.stop_all()
time.sleep(0.5)
controller.get_error_status(0, log=True)
