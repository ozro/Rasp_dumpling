// Uses POSIX functions to send and receive data from a
// Simple Motor Controller G2.
// NOTE: The Simple Motor Controller's input mode must be set to Serial/USB.
// NOTE: You must change the 'const char * device' line below.
 
#ifndef SMCSerial_HPP
#define SMCSerial_HPP
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
 
class SMCSerial{
	public:
	int fd;
	int id;
	SMCSerial(int port_number, int device_number);
	// Writes bytes to the serial port, returning 0 on success and -1 on failure.
	int write_port(const uint8_t * buffer, size_t size);
	 
	// Reads bytes from the serial port.
	// Returns after all the desired bytes have been read, or if there is a
	// timeout or other error.
	// Returns the number of bytes successfully read into the buffer, or -1 if
	// there was an error reading.
	ssize_t read_port(uint8_t * buffer, size_t size);
	int send_command(const uint8_t* cmd);
	 
	// Reads a variable from the SMC.
	// Returns 0 on success or -1 on failure.
	int get_variable(uint8_t variable_id, uint16_t * value);
	 
	// Gets the target speed (-3200 to 3200).
	// Returns 0 on success, -1 on failure.
	int get_target_speed(int16_t * value);

	int get_current_speed(int16_t * value);
	 
	// Gets a number where each bit represents a different error, and the
	// bit is 1 if the error is currently active.
	// See the user's guide for definitions of the different error bits.
	// Returns 0 on success, -1 on failure.
	int get_error_status(uint16_t * value);
	 
	// Sends the Exit Safe Start command, which is required to drive the motor.
	// Returns 0 on success, -1 on failure.
	int exit_safe_start();
	int stop_motor();
	 
	// Sets the SMC's target speed (-3200 to 3200).
	// Returns 0 on success, -1 on failure.
	int set_target_speed(int speed);
};

#endif
