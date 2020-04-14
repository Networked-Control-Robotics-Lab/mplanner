#include "math.h"
#include "trajectory.hpp"
#include "ros_thread.hpp"

void generate_circular_trajectory(trajectory_wp_t *wp_list, int waypoint_count, float height)
{
	float x, y;
	float diameter = 0.6; //[m]
	float angular_velocity = 2 * M_PI / waypoint_count;
	for(int i = 0; i < waypoint_count; i++) {
		x = diameter * cos(i * angular_velocity);
		y = diameter * sin(i * angular_velocity);

		wp_list[i].pos[0] = x;
		wp_list[i].pos[1] = y;
		wp_list[i].pos[2] = height;

		ros_trajectory_waypoint_push_back(x, y , height);
	}
}
