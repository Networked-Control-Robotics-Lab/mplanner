#include <thread>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include "../mavlink/publisher.hpp"
#include "quadshell.hpp"
#include "trajectory.hpp"

bool trajectory_follow_halt = false;

void shell_cmd_help(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
}

void shell_cmd_clear(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	shell_cls();
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

void shell_cmd_traj(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm trajectory following command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		send_mavlink_trajectory_following_cmd(true);
		sleep(0.5);
		send_mavlink_trajectory_following_cmd(true);
		sleep(0.5);
		send_mavlink_trajectory_following_cmd(true);
		sleep(0.5);

		trajectory_t traj[4];
		traj[0].start.pos[0] = 1.0f;
		traj[0].start.pos[1] = 0.0f;
		traj[0].start.pos[2] = 0.6f;
		traj[0].end.pos[0] = 0.0;
		traj[0].end.pos[1] = 1.0f;
		traj[0].end.pos[2] = 0.6f;
		traj[0].flight_time = 2.0f;

		traj[1].start.pos[0] = 0.0f;
		traj[1].start.pos[1] = 1.0f;
		traj[1].start.pos[2] = 0.6f;
		traj[1].end.pos[0] = -1.0f;
		traj[1].end.pos[1] = 0.0f;
		traj[1].end.pos[2] = 0.6f;
		traj[1].flight_time = 2.0f;

		traj[2].start.pos[0] = -1.0f;
		traj[2].start.pos[1] = 0.0f;
		traj[2].start.pos[2] = 0.6f;
		traj[2].end.pos[0] = 0.0f;
		traj[2].end.pos[1] = -1.0f;
		traj[2].end.pos[2] = 0.6f;
		traj[2].flight_time = 2.0f;

		traj[3].start.pos[0] = 0.0f;
		traj[3].start.pos[1] = -1.0f;
		traj[3].start.pos[2] = 0.6f;
		traj[3].end.pos[0] = 1.0f;
		traj[3].end.pos[1] = 0.0f;
		traj[3].end.pos[2] = 0.6f;
		traj[3].flight_time = 2.0f;

		plan_optimal_trajectory(traj, 4);
	} else {
		printf("abort.\n\r");
	}

}
