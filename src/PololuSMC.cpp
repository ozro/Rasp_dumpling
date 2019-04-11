#include <string>
#include <iostream>
#include <vector>

#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>

#include "SMCSerial.cpp"

using namespace std;

class PololuSMC(object){
    Public:

	char* port_name = "/dev/ttyS0";
	int baud_rate = 9600;

	int device_count = 5;

	int fd;

	vector<SMCSerial> smcs(5);

	MotorController(){
		//Open port
		printf("Opening serial port on %s with baud rate %d...",
			port_name, baud_rate);    
		try{
			open_serial_port();
			printf("Success! Port ID: %d", fd);
		}
		catch (int e){
			printf("Failed to open port");
			throw e; 
		}
		//Construct smc objects
		printf("Initializing motor objects...");
		for (int i = 0; i < 4; i++):
			smcs[i] = SMCSerial(fd, i); 
		smcs[4] = SMCSerial(fd, 13); //Actuator board ID = 13
		printf("Success!");
	}
	void exit_safe_start(int id){
		smcs[id].exit_safe_start();
	}
	uint16_t get_error_status(int id){
		uint16_t status;
		success = smcs[id].get_error_status(&status);
		return status;
	}
	uint16_t get_serial_status(int id){
		uint16_t status;
		success = smcs[id].get_variable(2, &status);
		return status;
	}
	uint16_t get_limit_status(int id){
		uint16_t status;
		success = smcs[id].get_variable(3, &status);
		return status;
	}
	uint16_t get_uptime(int id){
		uint16_t time_low;
		uint16_t time_high;
		success = smcs[id].get_variable(28, &time_low);
		success = smcs[id].get_variable(29, &time_high);
		return time_low + time_high * 65536;
	}
	uint16_t get_temperatures(int id){
		uint16_t tempA;
		uint16_t tempB;
		success = smcs[id].get_variable(24, &tempA);
		success = smcs[id].get_variable(25, &tempB);
		return (tempA+tempB)/2;
	}
	uint16_t get_input_voltage(int id){
		uint16_t voltage;
		success = smcs[id].get_variable(23, &voltage);
		return voltage;
	}
	uint16_t get_current(int id){
		uint16_t current;
		success = smcs[id].get_variable(44, &current);
		return current;
	}
	int16_t get_current_speed(int id){
		int16_t speed;
		success = smcs[id].get_current_speed(&speed)
		return speed;
	}
	int16_t get_target_speed(int id){
		int16_t speed;
		success = smcs[id].get_target_speed(&speed)
		return speed;
	}
	void set_target_speed(int id, int16_t speed){
		success = smcs[id].set_target_speed(speed)
	}
	void stop_all(){
		for (i = 0; i < smcs.size(); i++){
			self.smcs[i].send_command(0x60);
		}
	}
	void brake_all(){
		for (i = 0; i < smcs.size(); i++){
			self.smcs[i].set_target_speed(0);
		}
	}
	void safe_start_all(){
		for (i = 0; i < smcs.size(); i++){
			self.smcs[i].exit_safe_start();
		}
	}
	void open_serial_port(){
		fd = open(device, O_RDWR | O_NOCTTY);

		//Failed to open port
		if (fd == -1) {
			perror(device);
			return -1;
		}

		// Flush away any bytes previously read or written.
		int result = tcflush(fd, TCIOFLUSH);
		if (result){
			perror("tcflush failed");  // just a warning, not a fatal error
		}

		// Get the current configuration of the serial port.
		struct termios options;
		result = tcgetattr(fd, &options);
		if (result){
			perror("tcgetattr failed");
			close(fd);
			fd=-1;
		}

		// Turn off any options that might interfere with our ability to send and
		// receive raw binary bytes.
		options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
		options.c_oflag &= ~(ONLCR | OCRNL);
		options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

		// Set up timeouts: Calls to read() will return as soon as there is
		// at least one byte available or when 100 ms has passed.
		options.c_cc[VTIME] = 1;
		options.c_cc[VMIN] = 0;

		// This code only supports certain standard baud rates. Supporting
		// non-standard baud rates should be possible but takes more work.
		switch (baud_rate)
		{
			case 4800:   cfsetospeed(&options, B4800);   break;
			case 9600:   cfsetospeed(&options, B9600);   break;
			case 19200:  cfsetospeed(&options, B19200);  break;
			case 38400:  cfsetospeed(&options, B38400);  break;
			case 115200: cfsetospeed(&options, B115200); break;
			default:
				fprintf(stderr, "warning: baud rate %u is not supported, using 9600.\n",
				  baud_rate);
				cfsetospeed(&options, B9600);
				break;
		}
		cfsetispeed(&options, cfgetospeed(&options));

		result = tcsetattr(fd, TCSANOW, &options);
		if (result)
		{
			perror("tcsetattr failed");
			close(fd);
			fd=-1;
		}
	} 
}
