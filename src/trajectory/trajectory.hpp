#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__

typedef struct {
	struct {
		float pos[3];
		float vel[3];
		float acc[3];
		float yaw;
		float yaw_rate;
	} start;

	struct {
		float pos[3];
		float vel[3];
		float acc[3];
		float yaw;
		float yaw_rate;
	} end;

	float flight_time;
} trajectory_t;

void plan_optimal_trajectory(trajectory_t *traj, int segment_cnts);

#endif
