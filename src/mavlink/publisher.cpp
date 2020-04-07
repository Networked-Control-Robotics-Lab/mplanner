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

void send_mavlink_land_cmd(void)
{
	uint8_t target_sys = 0;
	uint8_t target_comp = 0;
	uint16_t cmd = 23; //land command
	uint8_t confirm = 0;
	float params[7];

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(0, 1, MAVLINK_COMM_0, &msg, target_sys, target_comp,
                                           cmd, confirm, params[0], params[1], params[2], params[3],
                                           params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(&msg);
}

void send_mavlink_trajectory_waypoint(float *pos, float *vel, float *acc, float yaw, float yaw_rate)
{
	uint64_t time_usec = 0;
	uint8_t valid_points = 0;
	uint16_t cmd;

	mavlink_message_t msg;
	mavlink_msg_trajectory_representation_waypoints_pack_chan(0, 1, MAVLINK_COMM_0, &msg,
		time_usec, valid_points, &pos[0], &pos[1], &pos[2], &vel[0], &vel[1], &vel[2],
		&acc[0], &acc[1], &acc[2], &yaw, &yaw_rate, &cmd);

	send_mavlink_msg_to_serial(&msg);
}
