#include "mavlink.h"
#include "serial.hpp"

void send_mavlink_msg_to_serial(mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	serial_puts((char *)buf, len);
}

void send_mavlink_takeoff_cmd(void)
{
	uint8_t target_sys = 0;
	uint8_t target_comp = 0;
	uint16_t cmd = 22; //takeoff command
	uint8_t confirm = 0;
	float params[7];

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(0, 1, MAVLINK_COMM_0, &msg, target_sys, target_comp,
                                           cmd, confirm, params[0], params[1], params[2], params[3],
                                           params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(&msg);
}
