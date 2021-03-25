#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "serial.hpp"

#define GROUND_STATION_ID 0 //according to the qgroundcontrol

static void send_mavlink_msg_to_serial(int uav_id, mavlink_message_t *msg)
{
	uint8_t buf[MAVLINK_MAX_PAYLOAD_LEN];
	uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

	serial_puts(uav_id, (char *)buf, len);
}

/*============================*
 * takeoff / langing messages *
 *============================*/

void send_mavlink_takeoff_cmd(uint8_t sys_id)
{
	uint8_t target_sys = sys_id;
	uint8_t target_comp = 0;
	uint16_t cmd = MAV_CMD_NAV_TAKEOFF;
	uint8_t confirm = 0;
	float params[7] = {0};

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, target_sys, target_comp,
	                                   cmd, confirm, params[0], params[1], params[2], params[3],
	                                   params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(sys_id, &msg);
}

void send_mavlink_land_cmd(uint8_t sys_id)
{
	uint8_t target_sys = sys_id;
	uint8_t target_comp = 0;
	uint16_t cmd = MAV_CMD_NAV_LAND;
	uint8_t confirm = 0;
	float params[7];

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, target_sys, target_comp,
	                                   cmd, confirm, params[0], params[1], params[2], params[3],
	                                   params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(sys_id, &msg);
}

/*===========================*
 * waypoint mission messages *
 *===========================*/

void send_mavlink_goto_cmd(uint8_t sys_id, float *pos, float yaw)
{
	uint8_t target_sys = sys_id;
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
	mavlink_msg_command_long_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, target_sys, target_comp,
	                                   cmd, confirm, params[0], params[1], params[2], params[3],
	                                   params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(sys_id, &msg);
}

void send_mavlink_halt_cmd(uint8_t sys_id)
{
	uint8_t target_sys = sys_id;
	uint8_t target_comp = 0;
	uint16_t cmd = MAV_CMD_OVERRIDE_GOTO;
	uint8_t confirm = 0;
	float params[7] = {0};

	params[0] = MAV_GOTO_DO_HOLD;
	params[1] = MAV_GOTO_HOLD_AT_CURRENT_POSITION;
	params[2] = MAV_FRAME_LOCAL_ENU;

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, target_sys, target_comp,
	                                   cmd, confirm, params[0], params[1], params[2], params[3],
	                                   params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(sys_id, &msg);
}

void send_mavlink_mission_resume_cmd(uint8_t sys_id)
{
	uint8_t target_sys = sys_id;
	uint8_t target_comp = 0;
	uint16_t cmd = MAV_CMD_OVERRIDE_GOTO;
	uint8_t confirm = 0;
	float params[7] = {0};

	params[0] = MAV_GOTO_DO_CONTINUE;

	mavlink_message_t msg;
	mavlink_msg_command_long_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, target_sys, target_comp,
	                                   cmd, confirm, params[0], params[1], params[2], params[3],
	                                   params[4], params[5], params[6]);
	send_mavlink_msg_to_serial(sys_id, &msg);
}

/*==============================*
 * trajectory planning messages *
 *==============================*/

void send_mavlink_polynomial_trajectory_start(bool loop)
{
	bool altitude_fixed = true;

	uint8_t target_system = 1; //XXX: fixed to the first UAV
	uint8_t target_component = 0;

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_cmd_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg,
                                                        target_system, target_component,
                                                        TRAJECTORY_FOLLOWING_START, altitude_fixed);
	send_mavlink_msg_to_serial(1, &msg);
}

void send_mavlink_polynomial_trajectory_stop()
{
	uint8_t target_system = 1; //XXX: fixed to the first UAV
	uint8_t target_component = 0;
	uint8_t option = 0;

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_cmd_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg,
                                                        target_system, target_component,
                                                        TRAJECTORY_FOLLOWING_STOP, option);
	send_mavlink_msg_to_serial(1, &msg);
}

void send_mavlink_polynomial_trajectory_write(uint8_t list_size, bool z_enabled, bool yaw_enabled)
{
	uint8_t _z_enabled = (z_enabled == true ? 1 : 0);
	uint8_t _yaw_enabled = (yaw_enabled == true ? 1 : 0);

        uint8_t target_system = 1; //XXX: fixed to the first UAV
        uint8_t target_component = 0;

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_write_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg, target_system,
                                                          target_component, list_size,
                                                          _z_enabled, _yaw_enabled);
	send_mavlink_msg_to_serial(1, &msg);
}

void send_mavlink_polynomial_trajectory_item(uint8_t index, uint8_t type, float *traj_poly_coeff,
                                             float flight_time)
{
	uint8_t target_system = 1; //XXX: fixed to the first UAV 
	uint8_t target_component = 0;

	mavlink_message_t msg;
	mavlink_msg_polynomial_trajectory_item_pack_chan(GROUND_STATION_ID, 1, MAVLINK_COMM_1, &msg,
                                                    target_system, target_component,
                                                    type, index, traj_poly_coeff,
                                                    flight_time);
	send_mavlink_msg_to_serial(1, &msg);
}
