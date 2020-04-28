#ifndef __TRAJECTORY_HPP__
#define __TRAJECTORY_HPP__

#include <vector>

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

void plan_optimal_trajectory(trajectory_t *traj, int segment_cnts,
                             std::vector<double> &poly_x, std::vector<double> &poly_y,
                             std::vector<double> &poly_z, std::vector<double> &poly_yaw);

void get_polynomial_coefficient_from_list(std::vector<double> &poly, double *c, float index);
void get_polynomial_coefficient_from_list(std::vector<double> &poly, float *c, float index);

#endif
