# Uses the pySerial library to send and receive data from a
# Simple Motor Controller G2.
#
# NOTE: The Simple Motor Controller's input mode must be "Serial/USB".
# NOTE: You might need to change the "port_name =" line below to specify the
#   right serial port.
 
import serial

class MotorController(object):
    def __init__(self, port_name, baud_rate=9600, timeout=0.1, write_timeout=0.1, debug=False, device_count=4):
        self.debug = debug
            
        self.debug_log("Opening serial port at {} with baud rate {}...".format(port_name, baud_rate))
        self.port = serial.Serial(port_name, baud_rate, timeout=timeout, write_timeout=write_timeout)
        self.debug_log("Success!\n")

        self.debug_log("Initializing {} motors...".format(device_count))
        self.device_count = device_count
        self.smcs = []
        for i in range(4):
            smc = SmcG2Serial(self.port, i)
            self.smcs.append(smc)
        self.debug_log("Success!\n")

    def exit_safe_start(self,ID):
        self.debug_log("Safe starting motor {}".format(ID))
        self.smcs[ID].exit_safe_start()
    
    def get_error_status(self, ID):
        status= smcs[ID].get_error_status()
        self.debug_log("Error status: 0x{:04X}".format(status))

    def get_target_speed(self, ID):
        speed = self.smcs[ID].get_target_speed()

    def set_target_speed(self, ID, speed):
        self.smcs[ID].set_target_speed(speed)

    def get_variable(self, ID, variable_id, signed=False):
        if(signed):
            return self.smc[ID].get_variable(variable_id)
        else:
            return self.smc[ID].get_variable_signed(variable_id)

    def safe_start_all(self):
        for ID in range(len(self.smcs)):
            self.exit_safe_start(ID)

    def debug_log(self, msg):
        if(self.debug):
            print(msg)
 
class SmcG2Serial(object):
    def __init__(self, port, device_number=None):
        self.port = port
        self.device_number = device_number
 
    def send_command(self, cmd, *data_bytes):
        if self.device_number == None:
            header = [cmd]  # Compact protocol
        else:
            header = [0xAA, self.device_number, cmd & 0x7F]  # Pololu protocol
        self.port.write(header + list(data_bytes))
 
  # Sends the Exit Safe Start command, which is required to drive the motor.
    def exit_safe_start(self):
        self.send_command(0x83)
 
  # Sets the SMC's target speed (-3200 to 3200).
    def set_target_speed(self, speed):
        cmd = 0x85  # Motor forward
        if speed < 0:
            cmd = 0x86  # Motor reverse
            speed = -speed
        self.send_command(cmd, speed & 0x1F, speed >> 5 & 0x7F)
 
  # Gets the specified variable as an unsigned value.
    def get_variable(self, id):
        self.send_command(0xA1, id)
        result = self.port.read(2)
        if len(result) != 2:
            raise RuntimeError("Expected to read 2 bytes, got {}."
            .format(len(result)))
        b = bytearray(result)
        return b[0] + 256 * b[1]
 
  # Gets the specified variable as a signed value.
    def get_variable_signed(self, id):
        value = self.get_variable(id)
        if value >= 0x8000:
            value -= 0x10000
        return value
 
  # Gets the target speed (-3200 to 3200).
    def get_target_speed(self):
        return self.get_variable_signed(20)
 
  # Gets a number where each bit represents a different error, and the
  # bit is 1 if the error is currently active.
  # See the user's guide for definitions of the different error bits.
    def get_error_status(self):
        return self.get_variable(0)
