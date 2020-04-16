#include "math.h"
#include "trajectory.hpp"
#include "ros_thread.hpp"
#include "planner.hpp"

void generate_circular_trajectory(trajectory_wp_t *wp_list, int waypoint_count, float height)
{
	float x, y;
	float diameter = 0.6; //[m]
	float angular_velocity = 2 * M_PI / (waypoint_count - 1);
	for(int i = 0; i < waypoint_count - 1; i++) {
		x = diameter * cos(i * angular_velocity);
		y = diameter * sin(i * angular_velocity);

		wp_list[i].pos[0] = x;
		wp_list[i].pos[1] = y;
		wp_list[i].pos[2] = height;

		ros_trajectory_waypoint_push_back(wp_list[i].pos[0],
		                                  wp_list[i].pos[1],
		                                  wp_list[i].pos[2]);
	}

	/* connect the last point with the first point */
	wp_list[waypoint_count-1].pos[0] = wp_list[0].pos[0];
	wp_list[waypoint_count-1].pos[1] = wp_list[0].pos[1];
	wp_list[waypoint_count-1].pos[2] = wp_list[0].pos[2];
	ros_trajectory_waypoint_push_back(wp_list[waypoint_count-1].pos[0],
	                                  wp_list[waypoint_count-1].pos[1],
	                                  wp_list[waypoint_count-1].pos[2]);

	/* solve quadratic programming problem to get velocity and acceleration */
	path_def path;
	trajectory_profile p_last, p_now;

	/* prepare sample points */
	p_last.pos << wp_list[0].pos[0],
		      wp_list[0].pos[1],
                      wp_list[0].pos[2];
	p_last.vel << 0.0f, 0.0f, 0.0f;
	p_last.acc << 0.0f, 0.0f, 0.0f;
	for(int i = 1; i < waypoint_count; i++) {
		p_now.pos << wp_list[waypoint_count-1].pos[0],
                             wp_list[waypoint_count-1].pos[1],
                             wp_list[waypoint_count-1].pos[2];
		p_now.vel << 0.0f, 0.0f, 0.0f;
		p_now.acc << 0.0f, 0.0f, 0.0f;

		path.push_back(segments(p_last, p_now, 2));
		p_last = p_now;
	}

	/* activate quadratic programming solver */
	qptrajectory plan;
	plan.get_profile(path ,path.size(), 0.02);
}
