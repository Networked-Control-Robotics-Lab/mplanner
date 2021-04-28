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
#include "serial.hpp"

using namespace std;

bool trajectory_follow_halt = false;

bool parse_float_from_str(char *str, float *value)
{
	char *end_ptr = NULL;
	errno = 0;
	*value = strtof(str, &end_ptr);
	if (errno != 0 || *end_ptr != '\0') {
		return false;
	} else {
		return true;
	}
}

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	printf("supported commands:\n\r"
	       "help\n\r"
	       "clear\n\r"
	       "exitj\n\r"
               "quit\n\r"
               "takeoff\n\r"
               "land\n\r"
               "traj\n\r");
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

void shell_cmd_takeoff(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	struct shell_struct shell;

	bool multiple_mode = false;
	bool format_error = false;

	float arg1_float, arg2_float;
	int arg1, arg2;

	if(param_cnt == 2) {
		if(parse_float_from_str(param_list[1], &arg1_float) == false) {
			printf("[invalid id] argument1 is not a proper number\n\r");
			format_error = true;
		}

		arg1 = (int)arg1_float;

		float total_uav_cnt = get_registered_uav_count();
		if(arg1 > total_uav_cnt) {
			printf("[invalid id] assigned id is out of the range of regestered uav numbers!\n\r");
			format_error = true;
		}

		if((arg1 < 1)) {
			printf("[invaild id] system id must be greater than 1!\n\r");
			format_error = true;
		}

		multiple_mode = false;
	} else if(param_cnt == 3) {
		if(parse_float_from_str(param_list[1], &arg1_float) == false) {
			printf("[invaild id range] argument1 is not a proper number\n\r");
			format_error = true;
		}

		if(parse_float_from_str(param_list[2], &arg2_float) == false) {
			printf("[invaild id range] argument2 is not a proper number\n\r");
			format_error = true;
		}

		arg1 = (int)arg1_float;
		arg2 = (int)arg2_float;

		if(arg1 >= arg2) {
			printf("[invaild id range] end id must be greater than start id!\n\r");
			format_error = true;
		}

		if((arg1 < 1) || (arg2 < 1)) {
			printf("[invaild id range] system id must be greater than 1!\n\r");
			format_error = true;
		}

		float total_uav_cnt = get_registered_uav_count();
		if((arg1 > total_uav_cnt) || (arg2 > total_uav_cnt)) {
			printf("[invalid id range] id range is out of the regestered uav numbers!\n\r");
			format_error = true;
		}

		multiple_mode = true;
	} else {
		format_error = true;
	}

	if(format_error == true) {
		printf("takeoff [id]: single drone takeoff\n\r"
                       "takeoff [id_start] [id_end]: multiple drone takeoff\n\r");
		return;
	}

	shell_init_struct(&shell, "confirm takeoff command [y/n]: ");
	shell_cli(&shell);

	uint8_t sys_id = 1;

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		if(multiple_mode == false) {
			send_mavlink_takeoff_cmd(arg1);
			printf("takeoff command is sent to the drone #%d\n\r", arg1);
		} else {
			for(int i = arg1; i <= arg2; i++) {
				send_mavlink_takeoff_cmd(i);
				printf("takeoff command is sent to the drone #%d\n\r", i);
			}
		}

		//TODO:receive ack message
	} else {
		printf("abort.\n\r");
	}

}

void shell_cmd_land(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	struct shell_struct shell;

	bool multiple_mode = false;
	bool format_error = false;

	float arg1_float, arg2_float;
	int arg1, arg2;

	if(param_cnt == 2) {
		if(parse_float_from_str(param_list[1], &arg1_float) == false) {
			printf("[invalid id] argument1 is not a proper number\n\r");
			format_error = true;
		}

		arg1 = (int)arg1_float;

		float total_uav_cnt = get_registered_uav_count();
		if(arg1 > total_uav_cnt) {
			printf("[invalid id] assigned id is out of the range of regestered uav numbers!\n\r");
			format_error = true;
		}

		if((arg1 < 1)) {
			printf("[invaild id] system id must be greater than 1!\n\r");
			format_error = true;
		}

		multiple_mode = false;
	} else if(param_cnt == 3) {
		if(parse_float_from_str(param_list[1], &arg1_float) == false) {
			printf("[invaild id range] argument1 is not a proper number\n\r");
			format_error = true;
		}

		if(parse_float_from_str(param_list[2], &arg2_float) == false) {
			printf("[invaild id range] argument2 is not a proper number\n\r");
			format_error = true;
		}

		arg1 = (int)arg1_float;
		arg2 = (int)arg2_float;

		if(arg1 >= arg2) {
			printf("[invaild id range] end id must be greater than start id!\n\r");
			format_error = true;
		}

		if((arg1 < 1) || (arg2 < 1)) {
			printf("[invaild id range] system id must be greater than 1!\n\r");
			format_error = true;
		}

		float total_uav_cnt = get_registered_uav_count();
		if((arg1 > total_uav_cnt) || (arg2 > total_uav_cnt)) {
			printf("[invalid id range] id range is out of the regestered uav numbers!\n\r");
			format_error = true;
		}

		multiple_mode = true;
	} else {
		format_error = true;
	}

	if(format_error == true) {
		printf("land [id]: single drone landing\n\r"
                       "land [id_start] [id_end]: multiple drone landing\n\r");
		return;
	}

	shell_init_struct(&shell, "confirm landing command [y/n]: ");
	shell_cli(&shell);

	uint8_t sys_id = 1;

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		if(multiple_mode == false) {
			send_mavlink_land_cmd(arg1);
			printf("landing command is sent to the drone #%d\n\r", arg1);
		} else {
			for(int i = arg1; i <= arg2; i++) {
				send_mavlink_land_cmd(i);
				printf("landing command is sent to the drone #%d\n\r", i);
			}
		}

		//TODO:receive ack message
	} else {
		printf("abort.\n\r");
	}
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

void plan_trajactory_shape_pretzel(trajectory_t *traj, int &traj_list_size,
		                   vector<double> &x_coeff_full, vector<double> &y_coeff_full,
                                   vector<double> &z_coeff_full, vector<double> &yaw_coeff_full)
{
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

	traj_list_size = 4;

	plan_optimal_trajectory(traj, traj_list_size, x_coeff_full, y_coeff_full,
                                z_coeff_full, yaw_coeff_full);

}

void shell_cmd_traj_plan(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm trajectory following command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		trajectory_t traj[100];
		int traj_list_size;
		vector<double> x_coeff_full, y_coeff_full, z_coeff_full;
		vector<double> yaw_coeff_full;

		plan_trajactory_shape_pretzel(traj, traj_list_size, x_coeff_full, y_coeff_full,
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
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm trajectory following command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		printf("mavlink: send polynomial_trajectory_start message.\n\r");
		send_mavlink_polynomial_trajectory_start(false);
	} else {
                printf("abort.\n\r");
        }
}

void shell_cmd_traj_stop(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm trajectory following command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		printf("mavlink: send polynomial_trajectory_stop message.\n\r");
		send_mavlink_polynomial_trajectory_stop();
	} else {
                printf("abort.\n\r");
        }
}

void shell_cmd_traj(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	if(param_cnt == 2) {
		if(strcmp(param_list[1], "plan") == 0) {
			shell_cmd_traj_plan(param_list, param_cnt);
		} else if(strcmp(param_list[1], "start") == 0) {
			shell_cmd_traj_start(param_list, param_cnt);
		} else if(strcmp(param_list[1], "stop") == 0) {
			shell_cmd_traj_stop(param_list, param_cnt);
		}

	} else {
		printf("traj plan: plan trajectory\n\r"
		       "traj start: start trajectoy following\n\r"
		       "traj stop: stop trajectory following\n\r");
	}
}
