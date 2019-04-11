// Uses POSIX functions to send and receive data from a
// Simple Motor Controller G2.
// NOTE: The Simple Motor Controller's input mode must be set to Serial/USB.
// NOTE: You must change the 'const char * device' line below.
 
#include "SMCSerial.hpp"
 
SMCSerial::SMCSerial(int port_number, int device_number){
    fd = port_number;
    id = device_number;
}
 
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
int SMCSerial::write_port(const uint8_t * buffer, size_t size)
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
ssize_t SMCSerial::read_port(uint8_t * buffer, size_t size)
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
 
// Reads a variable from the SMC.
// Returns 0 on success or -1 on failure.
int SMCSerial::get_variable(uint8_t variable_id, uint16_t * value)
{
  uint8_t command[] = { 0xAA, id, 0x21, variable_id};
  int result = write_port(command, sizeof(command));
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
int SMCSerial::get_target_speed(int16_t * value)
{
  return get_variable(20, (uint16_t *)value);
}

int SMCSerial::get_current_speed(int16_t * value)
{
  return get_variable(21, (uint16_t *)value);
}
 
// Gets a number where each bit represents a different error, and the
// bit is 1 if the error is currently active.
// See the user's guide for definitions of the different error bits.
// Returns 0 on success, -1 on failure.
int SMCSerial::get_error_status(uint16_t * value)
{
  return get_variable(0, value);
}
 
// Sends the Exit Safe Start command, which is required to drive the motor.
// Returns 0 on success, -1 on failure.
int SMCSerial::exit_safe_start()
{
  uint8_t command[] = { 0xAA, id, 0x03};
  int result = write_port(command, sizeof(command));
  return result;
}
int SMCSerial::stop_motor()
{
  uint8_t command[] = { 0xAA, id, 0x60};
  int result = write_port(command, sizeof(command));
  return result;
}
 
// Sets the SMC's target speed (-3200 to 3200).
// Returns 0 on success, -1 on failure.
int SMCSerial::set_target_speed(int speed)
{
  uint8_t command[3];
 
  if (speed < 0)
  {
    command[0] = 0x06; // Motor Reverse
    speed = -speed;
  }
  else
  {
    command[0] = 0x05; // Motor Forward
  }
  command[1] = speed & 0x1F;
  command[2] = speed >> 5 & 0x7F;
 
  uint8_t msg[] = { 0xAA, id, command[0], command[1], command[2]};
  int result = write_port(msg, sizeof(msg));
  return result;
}
