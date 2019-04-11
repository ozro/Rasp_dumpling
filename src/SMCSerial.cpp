// Uses POSIX functions to send and receive data from a
// Simple Motor Controller G2.
// NOTE: The Simple Motor Controller's input mode must be set to Serial/USB.
// NOTE: You must change the 'const char * device' line below.
 
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
 
class SMCSerial{
	public:
	int fd;
	int id;
	SMCSerial(int port_number, int device_number){
		fd = port_number;
		id = device_number;
	}
	 
	// Writes bytes to the serial port, returning 0 on success and -1 on failure.
	int write_port(const uint8_t * buffer, size_t size)
	{
	  ssize_t result = write(fd, buffer, size);
	  if (result != (ssize_t)size)
	  {
		//perror("failed to write to port");
		return -1;
	  }
	  return 0;
	}
	 
	// Reads bytes from the serial port.
	// Returns after all the desired bytes have been read, or if there is a
	// timeout or other error.
	// Returns the number of bytes successfully read into the buffer, or -1 if
	// there was an error reading.
	ssize_t read_port(uint8_t * buffer, size_t size)
	{
	  size_t received = 0;
	  while (received < size)
	  {
		ssize_t r = read(fd, buffer + received, size - received);
		if (r < 0)
		{
		  //perror("failed to read from port");
		  return -1;
		}
		if (r == 0)
		{
		  // Timeout
		  break;
		}
		received += r;
	  }
	  return received;
	}
	int send_command(const uint8_t* cmd){
		uint8_t command[] = {0xAA, id, *cmd & 0x7F};
		write_port(command, sizeof(command));
		return 0;
	}
	 
	// Reads a variable from the SMC.
	// Returns 0 on success or -1 on failure.
	int get_variable(uint8_t variable_id, uint16_t * value)
	{
	  uint8_t command[] = { 0xA1, variable_id };
	  int result = send_command(command);
	  if (result) { return -1; }
	  uint8_t response[2];
	  ssize_t received = read_port(response, sizeof(response));
	  if (received < 0) { return -1; }
	  if (received != 2)
	  {
		fprintf(stderr, "read timeout: expected 2 bytes, got %zu\n", received);
		return -1;
	  }
	  *value = response[0] + 256 * response[1];
	  return 0;
	}
	 
	// Gets the target speed (-3200 to 3200).
	// Returns 0 on success, -1 on failure.
	int get_target_speed(int16_t * value)
	{
	  return get_variable(20, (uint16_t *)value);
	}

	int get_current_speed(int16_t * value)
	{
	  return get_variable(21, (uint16_t *)value);
	}
	 
	// Gets a number where each bit represents a different error, and the
	// bit is 1 if the error is currently active.
	// See the user's guide for definitions of the different error bits.
	// Returns 0 on success, -1 on failure.
	int get_error_status(uint16_t * value)
	{
	  return get_variable(0, value);
	}
	 
	// Sends the Exit Safe Start command, which is required to drive the motor.
	// Returns 0 on success, -1 on failure.
	int exit_safe_start()
	{
	  const uint8_t command = 0x83;
	  return send_command(&command);
	}
	int stop_motor()
	{
	  const uint8_t command = 0x60;
	  return send_command(&command);
	}
	 
	// Sets the SMC's target speed (-3200 to 3200).
	// Returns 0 on success, -1 on failure.
	int set_target_speed(int speed)
	{
	  uint8_t command[3];
	 
	  if (speed < 0)
	  {
		command[0] = 0x86; // Motor Reverse
		speed = -speed;
	  }
	  else
	  {
		command[0] = 0x85; // Motor Forward
	  }
	  command[1] = speed & 0x1F;
	  command[2] = speed >> 5 & 0x7F;
	 
	  return send_command(command);
	}
};
