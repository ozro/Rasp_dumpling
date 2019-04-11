#include <string>
#include <iostream>
#include <vector>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>

#include "SMCSerial.hpp"

using namespace std;

class PololuSMC{
    public:
	char* port_name;
	int baud_rate;
	int device_count;
	int fd;

	vector<SMCSerial> smcs;

	PololuSMC();
	void init_port();
	void exit_safe_start(int id);
	uint16_t get_error_status(int id);
	uint16_t get_serial_status(int id);
	uint16_t get_limit_status(int id);
	uint16_t get_uptime(int id);
	uint16_t get_temperatures(int id);
	uint16_t get_input_voltage(int id);
	uint16_t get_current(int id);
	int16_t get_current_speed(int id);
	int16_t get_target_speed(int id);
	void set_target_speed(int id, int16_t speed);
	void stop_all();
	void brake_all();
	void safe_start_all();
	int open_serial_port();
};
