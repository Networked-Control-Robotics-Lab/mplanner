#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include <string>
#include "ros/ros.h"

#define MAX_UAV_CNT 100

using namespace std;

int uav_cnt = 0;
int serial_fd[MAX_UAV_CNT] = {0};

int get_registered_uav_count()
{
	return uav_cnt;
}

void reg_serial_with_uav(int id, const char *port_name, int baudrate)
{
	if((id < 1) || (id > MAX_UAV_CNT)) {
		ROS_FATAL("Vailed UAV id range is between 1~100!");
		exit(0);
	}

	if(id != (uav_cnt + 1)) {
		ROS_FATAL("Skipped id is not allowed for registering the UAV!");
	}

	//open the port
	serial_fd[id-1] = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(serial_fd[id] == -1) {
		ROS_FATAL("Failed to open the serial port.");
		exit(0);
	}

	//config the port
	struct termios options;

	tcgetattr(serial_fd[id-1], &options);

	options.c_cflag = CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;

	switch(baudrate) {
	case 9600:
		options.c_cflag |= B9600;
		break;
	case 57600:
		options.c_cflag |= B57600;
		break;
	case 115200:
		options.c_cflag |= B115200;
		break;
	default:
		ROS_FATAL("Unknown baudrate. try 9600, 57600, 115200 or check \"serial.cpp\".");
		exit(0);
	}

	tcflush(serial_fd[id-1], TCIFLUSH);
	tcsetattr(serial_fd[id-1], TCSANOW, &options);

	uav_cnt++;
}

void serial_puts(int id, char *s, size_t size)
{
	/* notice that uav id is start from 1 but arrary of serial
	 * file descriptor is start from 0 */
	write(serial_fd[id-1], s, size);
}

int serial_getc(int id, char *c)
{
	/* notice that uav id is start from 1 but arrary of serial
         * file descriptor is start from 0 */
	return read(serial_fd[id-1], c, 1);
}
