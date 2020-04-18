#include "math.h"
#include "trajectory.hpp"
#include "ros_thread.hpp"
#include "planner.hpp"

void print_trajectory_polynomial_coeff(std::vector<double> &poly_coeff)
{
	for(int i = 0; i < poly_coeff.size() / 8; i++) {
		float coeff[8];
		coeff[0] = poly_coeff.at(i * 8 + 0);
		coeff[1] = poly_coeff.at(i * 8 + 1);
		coeff[2] = poly_coeff.at(i * 8 + 2);
		coeff[3] = poly_coeff.at(i * 8 + 3);
		coeff[4] = poly_coeff.at(i * 8 + 4);
		coeff[5] = poly_coeff.at(i * 8 + 5);
		coeff[6] = poly_coeff.at(i * 8 + 6);
		coeff[7] = poly_coeff.at(i * 8 + 7);
		printf("#%d: (%f, %f, %f, %f, %f, %f, %f, %f)\n\r", i,
                       coeff[0], coeff[1], coeff[2], coeff[3],
                       coeff[4], coeff[5], coeff[6], coeff[7]);
	}
}

void plan_optimal_trajectory(trajectory_wp_t *wp_list, int waypoint_count)
{
	/* solve quadratic programming problem to get velocity and acceleration */
	path_def path;
	trajectory_profile p_last;

	/* generate input segments */
	p_last.pos << wp_list[0].pos[0],
		      wp_list[0].pos[1],
                      wp_list[0].pos[2];
	p_last.vel << 0.0f, 0.0f, 0.0f;
	p_last.acc << 0.0f, 0.0f, 0.0f;
	//p_last.yaw << 0.0f;
	for(int i = 1; i < waypoint_count; i++) {
		trajectory_profile p_now;
		p_now.pos << wp_list[i].pos[0],
                             wp_list[i].pos[1],
                             wp_list[i].pos[2];
		p_now.vel << 0.0f, 0.0f, 0.0f;
		p_now.acc << 0.0f, 0.0f, 0.0f;
		//p_now.yaw << 0.0f;

		path.push_back(segments(p_last, p_now, 2));
		p_last = p_now;
	}

	/* activate quadratic programming solver */
	std::vector<double> poly_x, poly_y, poly_yaw;
	system("/bin/stty cooked echo");
	qptrajectory plan;
	plan.get_profile(path ,path.size(), 0.02, poly_x, poly_y, poly_yaw);
	system ("/bin/stty raw -echo");

	/* print polynomial coefficients of planned trajectories */
	printf("\n\rpolynomial coefficients of x trajectory:\n\r");
	print_trajectory_polynomial_coeff(poly_x);

	printf("polynomial coefficients of y trajectory:\n\r");
	print_trajectory_polynomial_coeff(poly_y);

	printf("polynomial coefficients of yaw trajectory:\n\r");
	print_trajectory_polynomial_coeff(poly_yaw);
}
