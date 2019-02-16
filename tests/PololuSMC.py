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
    
    def get_error_status(self, ID, log=False):
        status = self.smcs[ID].get_error_status()
        if(log):
            msg = []
            if(status&1):
                msg.append("Safe Start Violation")
            if(status>>1&1):
                msg.append("Required channel invalid")
            if(status>>2&1):
                msg.append("Serial error")
            if(status>>3&1):
                msg.append("Command timeout")
            if(status>>4&1):
                msg.append("Limit/kill switch")
            if(status>>5&1):
                msg.append("Low VIN")
            if(status>>6&1):
                msg.append("High VIN")
            if(status>>7&1):
                msg.append("Over temperature")
            if(status>>8&1):
                msg.append("Motor driver error")
            if(status>>9&1):
                msg.append("ERR line high")
            self.debug_log("Motor {} Error status: {}".format(ID,','.join(msg)))
        return status

    def get_limit_status(self, ID, log=False):
        status = self.smcs[ID].get_variable(3)
        if(log):
            msg = ""
            if(status&1):
                msg.append("Cannot run due to safe start or error")
            if(status&1):
                msg.append("Temperature is reducing target speed") 
            if(status&2):
                msg.append("Max speed limit is reducing target speed") 
            if(status&3):
                msg.append("Starting speed limit is actively reducing target speed to zero")
            if(status&4):
                msg.append("Motor speed is not equal to target speed because of acceleration, deceleration, or brake duration limits")
            if(status&5):
                msg.append("RC1 is configured as a limit/kill switch and the switch is active)")
            if(status&6):
                msg.append("RC2 limit/kill switch is active")
            if(status&7):
                msg.append("AN1 limit/kill switch is active")
            if(status&8):
                msg.append("AN2 limit/kill switch is active") 
            if(status&9):
                msg.append("USB kill switch is active") 
            self.debug_log("Motor {} Error status: {}".format(ID,"\n".join(msg)))
        return status

    def get_uptime(self, ID, log=False):
        time_low = self.smcs[ID].get_variable(28)
        time_high = self.smcs[ID].get_variable(29)
        time = time_low + time_high * 65536
        if(log):
            self.debug_log("SMC {} has been up for {} ms".format(ID, time))
        return time

    def get_temperatures(self, ID, log=False):
        temp_A = self.smcs[ID].get_variable(24)
        temp_B = self.smcs[ID].get_variable(25)
        if(log):
            self.debug_log("SMC {} has average temperature {} C ({}, {})".format(ID, (temp_A+temp_B)/20, temp_A/10, temp_B/10))
        return (temp_A, temp_B)

    def get_input_voltage(self, ID, log=False):
        voltage = self.smcs[ID].get_variable(23)
        if(log):
            self.debug_log("SMC {} has input voltage {} V".format(ID, voltage/1000))
        return voltage

    def get_current(self, ID, log=False):
        current = self.smcs[ID].get_variable(44)
        if(log):
            self.debug_log("SMC {} has current {} mA".format(ID, current))
        return current

    def get_current_speed(self, ID, log=False):
        speed = self.smcs[ID].get_variable_signed(21)
        if(log):
            self.debug_log("Motor {} has current speed {}".format(ID, speed))
        return speed

    def get_target_speed(self, ID, log=False):
        speed = self.smcs[ID].get_target_speed()
        if(log):
            self.debug_log("Motor {} has target speed {}".format(ID, speed))
        return speed

    def set_target_speed(self, ID, speed):
        self.smcs[ID].set_target_speed(speed)

    def get_variable(self, ID, variable_id, signed=False):
        if(signed):
            return self.smcs[ID].get_variable(variable_id)
        else:
            return self.smcs[ID].get_variable_signed(variable_id)

    def stop_all(self):
        for ID in range(len(self.smcs)):
            self.smcs[ID].send_command(0x60)

    def brake_all(self):
        for ID in range(len(self.smcs)):
            self.set_target_speed(ID, 0)

    def safe_start_all(self):
        for ID in range(len(self.smcs)):
            self.exit_safe_start(ID)
    def set_all_speeds(self, VX, VY, VA):
        speeds = [VY - VX + VA,
                  VY + VX - VA,
                  VY - VX - VA,
                  VY + VX + VA]
        for motor in range(self.device_count):
            self.set_target_speed(motor, speeds[motor])

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
