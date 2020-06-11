#include <vector>
#include <thread>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "../mavlink/publisher.hpp"
#include "../mavlink/receiver.hpp"
#include "quadshell.hpp"
#include "trajectory.hpp"

using namespace std;

bool trajectory_follow_halt = false;

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
}

void shell_cmd_clear(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_cls();
}

void shell_cmd_exit(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	system("/bin/stty cooked echo");
	exit(0);
}

void shell_cmd_quit(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	system("/bin/stty cooked echo");
	exit(0);
}

void shell_cmd_arm(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
}

void shell_cmd_disarm(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
}

void shell_cmd_takeoff(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm takeoff command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		send_mavlink_takeoff_cmd();
		printf("takeoff mavlink message is sent.\n\r");

		//TODO:receive ack message
	} else {
		printf("abort.\n\r");
	}
}

void shell_cmd_land(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm landing command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		send_mavlink_land_cmd();

		printf("landing mavlink message is sent.\n\r");

		//TODO:receive ack message
	} else {
		printf("abort.\n\r");
	}
}

void shell_cmd_fly(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
}

bool send_poly_traj_write_and_confirm_recpt(uint8_t *traj_ack_val, uint8_t list_size,
                                            bool z_enabled, bool yaw_enabled)
{
	int send_trial = 10;
	do {
		printf("mavlink: send polynomial_trajectory_write message. (handshacking)\n\r");
		send_mavlink_polynomial_trajectory_write(list_size, z_enabled, yaw_enabled);

		bool ack_received = wait_mavlink_polynomial_trajectory_ack(traj_ack_val);
		if(ack_received == true) {
			printf("succeeded.\n\r");
			return true;
		} else {
			printf("timeout!\n\r");
		}
	} while(--send_trial);

	return false;
}

bool send_poly_traj_item_and_confirm_recpt(uint8_t *traj_ack_val, uint8_t index, uint8_t type,
                                           float *poly_coeff, float flight_time)
{
	string s;
	switch(type) {
	case TRAJECTORY_POSITION_X:
		s = "x";
		break;
	case TRAJECTORY_POSITION_Y:
		s = "y";
		break;
	case TRAJECTORY_POSITION_Z:
		s = "z";
		break;
	case TRAJECTORY_ANGLE_YAW:
		s = "yaw";
		break;
	}

	int send_trial = 10;
	do {
		printf("mavlink: [#%d] send %s trajectory.\n\r", index, s.c_str());
		send_mavlink_polynomial_trajectory_item(index, type, poly_coeff, flight_time);

		bool ack_received = wait_mavlink_polynomial_trajectory_ack(traj_ack_val);
		if(ack_received == true) {
			printf("succeeded.\n\r");
			return true;
		} else {
			printf("timeout!\n\r");
		}
	} while(--send_trial);

	return false;
}

void shell_cmd_traj_plan(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm trajectory following command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		trajectory_t traj[5];
		traj[0].start.pos[0] = 0;
		traj[0].start.pos[1] = 0;
		traj[0].start.pos[2] = 0.6f;
		traj[0].end.pos[0] = 0.6;
		traj[0].end.pos[1] = -0.6f;
		traj[0].end.pos[2] = 0.6f;
		traj[0].flight_time = 2.0f;

		traj[1].start.pos[0] = 0.6f;
		traj[1].start.pos[1] = -0.6f;
		traj[1].start.pos[2] = 0.6f;
		traj[1].end.pos[0] = 0.0f;
		traj[1].end.pos[1] = 0.6f;
		traj[1].end.pos[2] = 0.6f;
		traj[1].flight_time = 2.0f;

		traj[2].start.pos[0] = 0.0f;
		traj[2].start.pos[1] = 0.6f;
		traj[2].start.pos[2] = 0.6f;
		traj[2].end.pos[0] = -0.6f;
		traj[2].end.pos[1] = -0.6f;
		traj[2].end.pos[2] = 0.6f;
		traj[2].flight_time = 2.0f;

		traj[3].start.pos[0] = -0.6f;
		traj[3].start.pos[1] = -0.6f;
		traj[3].start.pos[2] = 0.6f;
		traj[3].end.pos[0] = 0.0f;
		traj[3].end.pos[1] = 0.0f;
		traj[3].end.pos[2] = 0.6f;
		traj[3].flight_time = 2.0f;

		int traj_list_size = 4; //TODO: fix hardcode

		vector<double> x_coeff_full, y_coeff_full, z_coeff_full;
		vector<double> yaw_coeff_full;
		plan_optimal_trajectory(traj, traj_list_size, x_coeff_full, y_coeff_full,
                                        z_coeff_full, yaw_coeff_full);

		uint8_t traj_ack_val;

		if(!send_poly_traj_write_and_confirm_recpt(&traj_ack_val, traj_list_size,
                                                           false, false)) {
			printf("polynomial trajectory handshacking failed.\n\r");
			return;
		}

		for(int i = 0; i < traj_list_size; i++) {
			float x_coeff[8], y_coeff[8], z_coeff[8];
			float yaw_coeff[4];

			get_polynomial_coefficient_from_list(x_coeff_full, x_coeff, i);
			get_polynomial_coefficient_from_list(y_coeff_full, y_coeff, i);
			//get_polynomial_coefficient_from_list(z_coeff_full, z_coeff, i);
			//get_polynomial_coefficient_from_list(yaw_coeff_full, yaw_coeff, i);

			if(!send_poly_traj_item_and_confirm_recpt(&traj_ack_val, i,
                                                                  TRAJECTORY_POSITION_X, x_coeff,
                                                                  traj[i].flight_time)) {
				printf("polynomial trajectory item sending is failed.\n\r");
				return;
			}

			if(!send_poly_traj_item_and_confirm_recpt(&traj_ack_val, i,
                                                                  TRAJECTORY_POSITION_Y, y_coeff,
                                                                  traj[i].flight_time)) {
				printf("polynomial trajectory item sending is failed.\n\r");
				return;
			}

#if 0
			if(!send_poly_traj_item_and_confirm_recpt(&traj_ack_val, i,
                                                                  TRAJECTORY_POSITION_Z, z_coeff,
                                                                  traj[i].flight_time)) {
				printf("polynomial trajectory item sending is failed.\n\r");
				return;
			}
#endif

#if 0
			if(!send_poly_traj_item_and_confirm_recpt(&traj_ack_val, i,
                                                                  TRAJECTORY_ANGLE_YAW, yaw_coeff,
                                                                  traj[i].flight_time)) {
				printf("polynomial trajectory item sending is failed.\n\r");
				return;
			}
#endif
		}

	} else {
		printf("abort.\n\r");
	}

}

void shell_cmd_traj_start(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	printf("mavlink: send polynomial_trajectory_start message.\n\r");
	send_mavlink_polynomial_trajectory_start(false);
}

void shell_cmd_traj(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(param_cnt == 2) {
		if(strcmp(param_list[1], "plan") == 0) {
			shell_cmd_traj_plan(param_list, param_cnt);
		} else if(strcmp(param_list[1], "start") == 0) {
			shell_cmd_traj_start(param_list, param_cnt);
		}
	}
}
