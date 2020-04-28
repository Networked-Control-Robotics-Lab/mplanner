#include "math.h"
#include "trajectory.hpp"
#include "ros_thread.hpp"
#include "planner.hpp"
#include "matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

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

double calc_polynomial(double *c, double t)
{
	return c[0] +
	       (c[1] * pow(t, 1)) +
               (c[2] * pow(t, 2)) +
               (c[3] * pow(t, 3)) +
               (c[4] * pow(t, 4)) +
               (c[5] * pow(t, 5)) +
               (c[6] * pow(t, 6)) +
               (c[7] * pow(t, 7));
}

void get_polynomial_coefficient_from_list(std::vector<double> &poly_x, double *c, float index)
{
	c[0] = poly_x.at(index * 8 + 0);
	c[1] = poly_x.at(index * 8 + 1);
	c[2] = poly_x.at(index * 8 + 2);
	c[3] = poly_x.at(index * 8 + 3);
	c[4] = poly_x.at(index * 8 + 4);
	c[5] = poly_x.at(index * 8 + 5);
	c[6] = poly_x.at(index * 8 + 6);
	c[7] = poly_x.at(index * 8 + 7);
}

void plot_optimal_trajectory(std::vector<double> &poly_x, std::vector<double> &poly_y,
                             std::vector<double> &poly_yaw/*, float *time*/)
{
	double total_time = 8;
        int n = 100; //sample counts
	double period = total_time / n;

        vector<double> x(n), y(n);

	double t;
	double cx[8], cy[8];

	t = 0;
	for(int i = 0; i < 25; i++) {
		get_polynomial_coefficient_from_list(poly_x, cx, 0);
		get_polynomial_coefficient_from_list(poly_y, cy, 0);
		x.at(i) = calc_polynomial(cx, t);
                y.at(i) = calc_polynomial(cy, t);
		t += period;
	}

	t = 0;
	for(int i = 25; i < 50; i++) {
		get_polynomial_coefficient_from_list(poly_x, cx, 1);
		get_polynomial_coefficient_from_list(poly_y, cy, 1);
		x.at(i) = calc_polynomial(cx, t);
                y.at(i) = calc_polynomial(cy, t);
		t += period;
	}

	t = 0;
	for(int i = 50; i < 75; i++) {
		get_polynomial_coefficient_from_list(poly_x, cx, 2);
		get_polynomial_coefficient_from_list(poly_y, cy, 2);
		x.at(i) = calc_polynomial(cx, t);
                y.at(i) = calc_polynomial(cy, t);
		t += period;
	}

	t = 0;
	for(int i = 75; i < 100; i++) {
		get_polynomial_coefficient_from_list(poly_x, cx, 3);
		get_polynomial_coefficient_from_list(poly_y, cy, 3);
		x.at(i) = calc_polynomial(cx, t);
                y.at(i) = calc_polynomial(cy, t);
		t += period;
	}

        plt::plot(x, y);
	plt::show();
}

void plan_optimal_trajectory(trajectory_t *traj, int segment_cnts)
{
	/* solve quadratic programming problem to get velocity and acceleration */
	path_def path;

	for(int i = 0; i < segment_cnts; i++) {
		trajectory_profile p_start;
		p_start.pos << traj[i].start.pos[0],
		               traj[i].start.pos[1],
                               traj[i].start.pos[2];
		p_start.vel << 0.0f, 0.0f, 0.0f;
		p_start.acc << 0.0f, 0.0f, 0.0f;
		//p_start.yaw << 0.0f;

		trajectory_profile p_end;
		p_end.pos << traj[i].end.pos[0],
                             traj[i].end.pos[1],
                             traj[i].end.pos[2];
		p_end.vel << 0.0f, 0.0f, 0.0f;
		p_end.acc << 0.0f, 0.0f, 0.0f;
		//p_end.yaw << 0.0f;

		path.push_back(segments(p_start, p_end, traj[i].flight_time));
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

	plot_optimal_trajectory(poly_x, poly_y, poly_yaw);
}
