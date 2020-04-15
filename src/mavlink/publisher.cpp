#include "mavlink.h"
#include "serial.hpp"

static void send_mavlink_msg_to_serial(mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	serial_puts((char *)buf, len);
}

void send_mavlink_do_set_mode_cmd(uint8_t target_mode)
{
	uint8_t custom_mode = 0;
	uint32_t custom_sub_mode = 0;

	mavlink_message_t msg;
	mavlink_msg_set_mode_pack_chan(0, 1, MAVLINK_COMM_0, &msg, target_mode, custom_mode,
                                       custom_sub_mode);
	send_mavlink_msg_to_serial(&msg);
}

void send_mavlink_takeoff_cmd(void)
{
	uint8_t target_sys = 0;
	uint8_t target_comp = 0;
	uint16_t cmd = MAV_CMD_NAV_TAKEOFF;
	uint8_t confirm = 0;
	float params[7] = {0};

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
	uint16_t cmd = MAV_CMD_NAV_LAND;
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

void send_mavlink_goto_cmd(float *pos, float yaw)
{
	uint8_t target_sys = 0;
	uint8_t target_comp = 0;
	uint16_t cmd = MAV_CMD_OVERRIDE_GOTO;
	uint8_t confirm = 0;
	float params[7] = {0};

	params[0] = MAV_GOTO_DO_HOLD;
	params[1] = MAV_GOTO_HOLD_AT_SPECIFIED_POSITION;
	params[2] = MAV_FRAME_LOCAL_ENU;
	params[3] = yaw;
	params[4] = pos[0];
	params[5] = pos[1];
	params[6] = pos[2];

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(0, 1, MAVLINK_COMM_0, &msg, target_sys, target_comp,
                                           cmd, confirm, params[0], params[1], params[2], params[3],
                                           params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(&msg);
}

void send_mavlink_halt_cmd(float *pos)
{
	uint8_t target_sys = 0;
	uint8_t target_comp = 0;
	uint16_t cmd = MAV_CMD_OVERRIDE_GOTO;
	uint8_t confirm = 0;
	float params[7] = {0};

	params[0] = MAV_GOTO_DO_HOLD;
	params[1] = MAV_GOTO_HOLD_AT_CURRENT_POSITION;
	params[2] = MAV_FRAME_LOCAL_ENU;

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(0, 1, MAVLINK_COMM_0, &msg, target_sys, target_comp,
                                           cmd, confirm, params[0], params[1], params[2], params[3],
                                           params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(&msg);
}

void send_mavlink_mission_resume_cmd()
{
	uint8_t target_sys = 0;
	uint8_t target_comp = 0;
	uint16_t cmd = MAV_CMD_OVERRIDE_GOTO;
	uint8_t confirm = 0;
	float params[7] = {0};

	params[0] = MAV_GOTO_DO_CONTINUE;

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(0, 1, MAVLINK_COMM_0, &msg, target_sys, target_comp,
                                           cmd, confirm, params[0], params[1], params[2], params[3],
                                           params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(&msg);
}

void send_mavlink_position_target(float *pos_enu, float *vel_enu, float *acc_enu,
                                  float yaw, float yaw_rate)
{
	uint8_t target_system = 1;
	uint8_t target_component = 1;

	uint64_t sys_time = 0;    //ignored
	uint8_t valid_points = 0; //ignored
	uint8_t type_mask = 0;    //ignored

	mavlink_message_t msg;
	mavlink_msg_set_position_target_local_ned_pack_chan(
		0, 1, MAVLINK_COMM_0, &msg,
		sys_time, target_system, target_component,
		MAV_FRAME_LOCAL_ENU, type_mask, 
		pos_enu[0], pos_enu[1], pos_enu[2],
		vel_enu[0], vel_enu[1], vel_enu[2],
		acc_enu[0], acc_enu[1], acc_enu[2],
		yaw, yaw_rate);
	send_mavlink_msg_to_serial(&msg);
}
