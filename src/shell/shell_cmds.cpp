#include <thread>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include "../mavlink/publisher.hpp"
#include "quadshell.hpp"
#include "trajectory.hpp"

#define WAYPOINT_UPDATE_RATE 50 //[Hz]
#define TRAJECTORY_TRAVEL_TIME 30 //[second]
#define TRAJECTORY_WP_NUM (WAYPOINT_UPDATE_RATE * TRAJECTORY_TRAVEL_TIME)

trajectory_wp_t trajectory_wp[TRAJECTORY_WP_NUM];

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

void command_uav_follow_trajectory_waypoints(void)
{
	useconds_t sleep_time = 1000000 * (1.0f / (float)WAYPOINT_UPDATE_RATE);

	int wp_num = 0;
	while(trajectory_follow_halt == false) {
		if(wp_num == TRAJECTORY_WP_NUM) return;

		send_mavlink_position_target(trajectory_wp[wp_num].pos,
			trajectory_wp[wp_num].vel,
			trajectory_wp[wp_num].acc,
			trajectory_wp[wp_num].yaw,
			trajectory_wp[wp_num].yaw_rate);

		wp_num++;

		//shell_puts("new waypoint is sent.\n\r");
		usleep(sleep_time);
	}
	trajectory_follow_halt = false;
}

void shell_cmd_traj(char param_list[PARAM_LIST_SIZE_MAX][PARAM_LEN_MAX], int param_cnt)
{
	char user_agree[CMD_LEN_MAX];
	struct shell_struct shell;
	shell_init_struct(&shell, "confirm trajectory following command [y/n]: ");
	shell_cli(&shell);

	if(strcmp(shell.buf, "y") == 0 || strcmp(shell.buf, "Y") == 0) {
		shell_puts("press [q] to leave.\n\r");

		generate_circular_trajectory(trajectory_wp, TRAJECTORY_WP_NUM, 1.5f);
		std::thread trajectory_commander_thread(command_uav_follow_trajectory_waypoints);

		trajectory_commander_thread.detach();

		while(getchar() != 'q');
		trajectory_follow_halt = true;
	} else {
		printf("abort.\n\r");
	}

}
