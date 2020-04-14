#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__

typedef struct {
	float pos[3];
	float vel[3];
} trajectory_wp_t;

void generate_circular_trajectory(trajectory_wp_t *wp_list, int waypoint_count, float height);

#endif
