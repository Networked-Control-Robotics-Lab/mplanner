#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__

typedef struct {
	float pos[3];
	float vel[3];
	float acc[3];
	float yaw;
	float yaw_rate;
} trajectory_wp_t;

void plan_optimal_trajectory(trajectory_wp_t *wp_list, int waypoint_count);

#endif
