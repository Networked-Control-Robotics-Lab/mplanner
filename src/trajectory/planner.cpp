#include <stdio.h>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <Python.h>
#include <iostream>
#include "cvxopt.h"
#include "planner.hpp"

using namespace Eigen;

std::vector<double> qptrajectory::qpsolve8(Eigen::VectorXd waypoint, int number, Eigen::VectorXd duration)
{
	std::vector<double> polynomial;
	polynomial.clear();

	//Hessian Matrix
	Eigen::MatrixXd D(number*8,number*8);
	Eigen::MatrixXd d(4,4);

	//polynomial constraint matrix including all segment
	Eigen::MatrixXd A((number+1)*4+(number-1),number*8);
	Eigen::MatrixXd Ai(4,8);
	Eigen::MatrixXd Af(4,8);

	//waypoint vector
	Eigen::MatrixXd B((number+1)*4+(number-1),1);
	Eigen::MatrixXd G_(number*8,number*8);
	Eigen::MatrixXd h_(number*8,1);

	G_.setZero();
	h_.setZero();
	D.setZero();
	d.setZero();
	A.setZero();
	B.setZero();
	Ai.setZero();
	Af.setZero();

	Py_Initialize();
	if (import_cvxopt() < 0) {
		//fprintf(stderr, "error importing cvxopt");
	}

	//create solver object
	PyObject *solvers = PyImport_ImportModule("cvxopt.solvers");

	if (!solvers) {
		//fprintf(stderr, "error importing cvxopt.solvers");
	}
	PyObject *lp = PyObject_GetAttrString(solvers, "qp");

	if (!lp) {
		fprintf(stderr, "error referencing cvxopt.solvers.lp");
		Py_DECREF(solvers);
	}

	PyObject *Q = (PyObject*)Matrix_New(number * 8, number * 8, DOUBLE);
	PyObject *p = (PyObject*)Matrix_New(number * 8, 1, DOUBLE);
	PyObject *G = (PyObject*)Matrix_New(number * 8, number * 8, DOUBLE);
	PyObject *h = (PyObject*)Matrix_New(number * 8, 1, DOUBLE);
	PyObject *A_ = (PyObject*)Matrix_New((number+1)*4 + (number-1), number*8, DOUBLE);
	PyObject *B_ = (PyObject*)Matrix_New((number+1)*4 + (number-1), 1, DOUBLE);

	PyObject *pArgs = PyTuple_New(6);
	if(!G || !Q || !pArgs) {
		fprintf(stderr, "error creating matrices");
		Py_DECREF(solvers);
		Py_DECREF(lp);
		Py_XDECREF(G);
		Py_XDECREF(Q);
		Py_XDECREF(h);
		Py_XDECREF(pArgs);
	}

	for(int i=0 ; i < number ; i++) {
		D.block<4,4>(4+i*8,4+i*8) = t8_array(duration(i));
	}

	Ai << 1, 0, 0, 0, 0, 0, 0, 0,
	      0, 1, 0, 0, 0, 0, 0, 0,
	      0, 0, 2, 0, 0, 0, 0, 0,
	      0, 0, 0, 6, 0, 0, 0, 0;

	Af << 0,  0,  0,  0, 0, 0, 0, 0,
	      0, -1,  0,  0, 0, 0, 0, 0,
	      0,  0, -2,  0, 0, 0, 0, 0,
	      0,  0,  0, -6, 0, 0, 0, 0;

	A.block<4,8>(0,0) = Ai;
	for(int i=0 ; i < number ; i++) {
		A.block<4,8>(4+i*4,i*8) = endpoint_array8(duration(i));
	}

	for(int i=0 ; i < (number - 1) ; i++) {
		A.block<4,8>(4*(i + 1), 8 * (i + 1)) = Af;
		A((number+1)*4 + i, 8*(i + 1)) = 1;
	}

	B = waypoint;
	//std::cout<< std::endl << D <<std::endl;
	//std::cout<< std::endl << A <<std::endl;
	//std::cout<< std::endl << B <<std::endl;

	for(int i=0 ; i<(number*8); i++) {
		for(int j=0; j<(i+1); j++) {
			MAT_BUFD(Q)[i*(number*8)+j] = D(j,i)   ;
		}
	}

	for(int i=0; i<(number*8); i++) {
		for(int j=0; j<((number+1)*4+(number-1)); j++) {
			MAT_BUFD(A_)[i*((number+1)*4+(number-1))+j] = A(j,i)   ;
		}
	}

	for(int i=0; i<((number+1)*4+(number-1)); i++) {
		MAT_BUFD(B_)[i] = B(i,0)   ;
	}

	for(int i=0; i<(number*8); i++) {
		MAT_BUFD(p)[i,0] = 0;
	}

	G_(1,1) = 0;
	G_(9,9) = 0;
	G_(17,17) = 0;
	G_(25,25) = 0;
	//std::cout<< G_.block<8,8>(0,0) << std::endl;
	for(int i=0; i<(number*8); i++) {
		for(int j=0; j<(i+1); j++) {
			MAT_BUFD(G)[i*number*8+j] = G_(j,i);
		}
	}

	h_(1,0) = 0;
	h_(9,0) = 0;
	h_(17,0) = 0;
	h_(25,0) = 0;
	//std::cout<< h_ << std::endl;
	for(int i=0; i<(number*8); i++) {
		MAT_BUFD(h)[i] = h_(i,0);
	}

	PyTuple_SetItem(pArgs, 0, Q);
	PyTuple_SetItem(pArgs, 1, p);
	PyTuple_SetItem(pArgs, 2, G);
	PyTuple_SetItem(pArgs, 3, h);
	PyTuple_SetItem(pArgs, 4, A_);
	PyTuple_SetItem(pArgs, 5, B_);

	PyObject *sol = PyObject_CallObject(lp, pArgs);

	if (!sol) {
		PyErr_Print();
		Py_DECREF(solvers);
		Py_DECREF(lp);
		Py_DECREF(pArgs);
	}

	PyObject *x = PyDict_GetItemString(sol, "x");

	for(int i=0; i<number*8; i++) {
		polynomial.push_back(MAT_BUFD(x)[i]);

	}

	Py_DECREF(solvers);
	Py_DECREF(lp);
	Py_DECREF(pArgs);
	Py_DECREF(sol);
	Py_Finalize();

	return polynomial;
}

std::vector<double> qptrajectory::qpsolve4(Eigen::VectorXd waypoint, int number, Eigen::VectorXd duration)
{
	std::vector<double> polynomial;
	polynomial.clear();
	//double t = 0;
	//Hessian Matrix
	Eigen::MatrixXd D(number*4,number*4);
	Eigen::MatrixXd d(2,2);
	//polynomial constraint matrix including all segment
	Eigen::MatrixXd A((number+1)*2+(number-1),number*4);
	Eigen::MatrixXd Ai(2,4);
	Eigen::MatrixXd Af(2,4);
	//waypoint vector
	Eigen::MatrixXd B((number+1)*2+(number-1),1);

	D.setZero();
	d.setZero();
	A.setZero();
	B.setZero();
	Ai.setZero();
	Af.setZero();

	Py_Initialize();
	if (import_cvxopt() < 0) {
		//fprintf(stderr, "error importing cvxopt");
	}

	//create solver object
	PyObject *solvers = PyImport_ImportModule("cvxopt.solvers");

	if (!solvers) {
		// fprintf(stderr, "error importing cvxopt.solvers");
	}
	PyObject *lp = PyObject_GetAttrString(solvers, "qp");

	if (!lp) {
		fprintf(stderr, "error referencing cvxopt.solvers.lp");
		Py_DECREF(solvers);

	}

	PyObject *G = (PyObject*)Matrix_New(1,(number*4),DOUBLE);
	PyObject *p = (PyObject*)Matrix_New((number*4),1,DOUBLE);
	PyObject *Q = (PyObject*)Matrix_New((number*4),(number*4),DOUBLE);
	PyObject *h = (PyObject*)Matrix_New(1,1,DOUBLE);

	PyObject *A_ = (PyObject*)Matrix_New((number+1)*2+(number-1),(number*4),DOUBLE); //ok
	PyObject *B_ = (PyObject*)Matrix_New((number+1)*2+(number-1),1,DOUBLE);  //ok

	PyObject *pArgs = PyTuple_New(6);
	if(!G || !Q || !pArgs) {
		fprintf(stderr, "error creating matrices");
		Py_DECREF(solvers);
		Py_DECREF(lp);
		Py_XDECREF(G);
		Py_XDECREF(Q);
		Py_XDECREF(h);
		Py_XDECREF(pArgs);
	}

	for(int i=0 ; i < number ; i++) {
		D.block<2,2>(2+i*4,2+i*4) = t4_array(duration(i));
	}

	Ai << 1, 0, 0, 0,
	      0, 1, 0, 0;

	Af << 0,  0, 0, 0,
	      0, -1, 0, 0;

	A.block<2,4>(0,0) = Ai;

	for(int i=0 ; i < number ; i++) {
		A.block<2,4>(2+i*2,i*4) = endpoint_array4(duration(i));
	}

	for(int i=0 ; i < (number-1) ; i++) {
		A.block<2,4>(2*(i+1),4*(i+1)) = Af;
		A((number+1)*2+i,4*(i+1)) = 1;
	}

	//B = waypoint;
	for(int i=0 ; i < (number+1) ; i++) {
		B(i*2,0) = waypoint(i*4);
		B(i*2+1,0) = waypoint(i*4+1);
	}

	for(int i=0 ; i < (number-1) ; i++) {
		B((number+1)*2+i,0) = waypoint((number+1)*4+i);
	}

	for(int i=0 ; i<(number*4); i++) {
		for(int j=0; j<(i+1); j++) {
			MAT_BUFD(Q)[i*(number*4)+j] = D(j,i)   ;
		}
	}

	for(int i=0; i<(number*4); i++) {
		for(int j=0; j<((number+1)*2+(number-1)); j++) {
			MAT_BUFD(A_)[i*((number+1)*2+(number-1))+j] = A(j,i)   ;
		}
	}

	for(int i=0; i<((number+1)*2+(number-1)); i++) {
		MAT_BUFD(B_)[i] = B(i,0)   ;
	}

	PyTuple_SetItem(pArgs, 0, Q);
	PyTuple_SetItem(pArgs, 1, p);
	PyTuple_SetItem(pArgs, 2, G);
	PyTuple_SetItem(pArgs, 3, h);
	PyTuple_SetItem(pArgs, 4, A_);
	PyTuple_SetItem(pArgs, 5, B_);

	PyObject *sol = PyObject_CallObject(lp, pArgs);

	if (!sol) {
		PyErr_Print();
		Py_DECREF(solvers);
		Py_DECREF(lp);
		Py_DECREF(pArgs);
	}

	PyObject *x = PyDict_GetItemString(sol, "x");

	for(int i=0; i<number*4; i++) {
		polynomial.push_back(MAT_BUFD(x)[i]);

	}

	Py_DECREF(solvers);
	Py_DECREF(lp);
	Py_DECREF(pArgs);
	Py_DECREF(sol);
	Py_Finalize();

	return polynomial;
}

Eigen::MatrixXd qptrajectory::endpoint_array8( double t)
{
	Eigen::MatrixXd A;
	A.setZero(4,8);
	A<< 1, 1*t, 1*t*t, 1*t*t*t, 1*t*t*t*t, 1*t*t*t*t*t, 1*t*t*t*t*t*t, 1*t*t*t*t*t*t*t,
	0,  1,   2*t,  3*t*t,   4*t*t*t,   5*t*t*t*t,   6*t*t*t*t*t,   7*t*t*t*t*t*t,
	0,  0,    2,   6*t,     12*t*t,    20*t*t*t,    30*t*t*t*t,    42*t*t*t*t*t,
	0,  0,    0,   6,24*t,  60*t*t,    120*t*t*t,   210*t*t*t*t;
	return A;
}

Eigen::MatrixXd qptrajectory::endpoint_array4( double t)
{
	Eigen::MatrixXd A;
	A.setZero(2,4);
	A<<1, 1*t, 1*t*t, 1*t*t*t,
	0,  1,   2*t, 3*t*t  ;
	return A;
}

Eigen::MatrixXd qptrajectory::t8_array( double t)
{
	double b0 = 24,
	       b1 = 120,
	       b2 = 360,
	       b3 = 840;
	double d11 = b0*b0*t, d12 = b0*b1*t*t, d13 = b0*b2*t*t*t, d14 = b0*b3*t*t*t*t ;
	double d21 = b0*b1*t*t, d22 = b1*b1*t*t*t, d23 = b1*b2*t*t*t*t, d24 = b1*b3*t*t*t*t*t ;
	double d31 = b0*b2*t*t*t, d32 = b2*b1*t*t*t*t, d33 = b2*b2*t*t*t*t*t, d34 = b2*b3*t*t*t*t*t*t;
	double d41 = b3*b0*t*t*t*t, d42 = b3*b1*t*t*t*t*t, d43 = b3*b2*t*t*t*t*t*t, d44 = b3*b3*t*t*t*t*t*t*t ;
	Eigen::MatrixXd d;
	d.setZero(4,4);
	d << (1/1.0)*d11*1.0, (1/2.0)*d21*1.0,(1/3.0)*d31*1.0, (1/4.0)*d41*1.0,
	(1/2.0)*d21*1.0, (1/3.0)*d22*1.0,(1/4.0)*d32*1.0, (1/5.0)*d42*1.0,
	(1/3.0)*d31*1.0, (1/4.0)*d32*1.0, (1/5.0)*d33*1.0, (1/6.0)*d43*1.0,
	(1/4.0)*d41*1.0, (1/5.0)*d42*1.0, (1/6.0)*d43*1.0, (1/7.0)*d44*1.0 ;
	return d;
}

Eigen::MatrixXd qptrajectory::t4_array( double t)
{
	double b0 = 2,
	       b1 = 6;
	double d11 = b0*b0*t, d12 = b0*b1*t*t;
	double d21 = b0*b1*t*t, d22 = b1*b1*t*t*t;

	Eigen::MatrixXd d;
	d.setZero(2,2);
	d << (1/1.0)*d11*1.0, (1/2.0)*d21*1.0,
	(1/2.0)*d21*1.0, (1/3.0)*d22*1.0 ;
	return d;
}

std::vector<double> qptrajectory::compress_time(Eigen::VectorXd waypointx,Eigen::VectorXd waypointy,int number, Eigen::VectorXd duration,std::vector<double> poly_x,std::vector<double> poly_y)
{
	double compress_weight = 1000000;
	double total_t = 0 ;
	double new_total_t = 0 ;
	double minimize_function = 0;
	double h = 0.005;
	double alpha = 0.00000025;
	double jx=0,jy=0;
	double new_jx=0,new_jy=0;
	Eigen::VectorXd gradient;
	Eigen::VectorXd new_minimize_set;
	Eigen::MatrixXd new_duration_set(number,number);
	Eigen::VectorXd new_duration(number);
	Eigen::VectorXd polynomialx;
	Eigen::VectorXd polynomialy;
	Eigen::VectorXd new_polynomialx;
	Eigen::VectorXd new_polynomialy;
	Eigen::MatrixXd D(number*8,number*8);
	std::vector<double> new_poly;
	std::vector<double> new_poly_x;
	std::vector<double> new_poly_y;
	gradient.setZero(number);
	new_minimize_set.setZero(number);
	polynomialx.setZero(number*8);
	polynomialy.setZero(number*8);
	new_polynomialx.setZero(number*8);
	new_polynomialy.setZero(number*8);
	new_duration.setZero(number);
	D.setZero();
	new_duration_set.setZero();

	//std to eigen
	for(int i=0 ; i < number*8 ; i++) {
		polynomialx(i)=poly_x[i];
		polynomialy(i)=poly_y[i];
	}
	//Hessian Matrix
	for(int i=0 ; i < number ; i++) {
		D.block<4,4>(4+i*8,4+i*8) = t8_array(duration(i));
		total_t+=duration(i);
	}
	jx = polynomialx.transpose()*D*polynomialx;
	jy = polynomialy.transpose()*D*polynomialy;
	//original cost function
	minimize_function = jx + jy + compress_weight*total_t;
	//std::cout<< std::endl << minimize_function <<std::endl;

	//gradient approach
	for(int j=0 ; j < number ; j++) {
		for(int i=0 ; i < number ; i++) {
			if(i==j) {
				new_duration_set(j,i) = duration(i) + h;
			} else {
				new_duration_set(j,i) = duration(i);
			}
		}
	}

	//obtain each polynomial for each duration
	for(int j=0 ; j < number ; j++) {
		for(int i=0 ; i < number ; i++) {
			new_duration(i)=new_duration_set(j,i);
			new_total_t+=new_duration(i);
		}
		new_poly_x = qpsolve8(waypointx,number,new_duration);
		new_poly_y = qpsolve8(waypointy,number,new_duration);
		for(int i=0 ; i < number*8 ; i++) {
			new_polynomialx(i)=new_poly_x[i];
			new_polynomialy(i)=new_poly_y[i];
		}
		new_jx = new_polynomialx.transpose()*D*new_polynomialx;
		new_jy = new_polynomialy.transpose()*D*new_polynomialy;
		new_minimize_set(j) =  new_jx + new_jy + compress_weight*new_total_t;
		gradient(j) = (new_minimize_set(j) - minimize_function)/h;
		//std::cout<< std::endl << new_polynomialx <<std::endl;
		new_duration.setZero(number);
		new_polynomialx.setZero(number*8);
		new_polynomialy.setZero(number*8);
		new_total_t = 0;
		//std::cout<< std::endl << new_minimize_set <<std::endl;

	}
	//std::cout<< std::endl << gradient <<std::endl;

	for(int j=0 ; j < number ; j++) {
		new_duration(j) = duration(j) - alpha*gradient(j);
	}
	//obtain new polynomial for duration after compress
	new_poly_x = qpsolve8(waypointx,number,new_duration);
	new_poly_y = qpsolve8(waypointy,number,new_duration);

	for(int i=0; i<number*8; i++) {
		new_poly.push_back(new_poly_x[i]);
		//std::cout<< std::endl << new_poly[i] <<std::endl;
	}
	for(int i=0; i<number*8; i++) {
		new_poly.push_back(new_poly_y[i]);
		//std::cout<< std::endl << new_poly[number*8+i] <<std::endl;
	}
	for(int i=0; i<number; i++) {
		new_poly.push_back(new_duration[i]);
	}
	return new_poly;
}

std::vector<double> qptrajectory::adjust_time(Eigen::VectorXd waypointx,
                                              Eigen::VectorXd waypointy, int number,
                                              Eigen::VectorXd duration, int iteration,
                                              std::vector<double> poly_x,std::vector<double> poly_y)
{
	double total_t = 0;
	double new_total_t = 0;
	double minimize_function = 0;
	double new_minimize_function = 0;
	double h = 0.05;
	double alpha = 0.00025;
	double step = 0.6;
	double c = 0.5;
	double gain;
	double jx = 0,jy = 0;
	double new_jx = 0, new_jy = 0;

	Eigen::VectorXd gradient;
	Eigen::VectorXd last_gradient;
	Eigen::VectorXd lastlast_gradient;
	Eigen::VectorXd direction;
	Eigen::VectorXd new_minimize_set;
	Eigen::MatrixXd new_duration_set;
	Eigen::MatrixXd direction_set;
	Eigen::VectorXd new_duration;
	Eigen::VectorXd last_duration;
	Eigen::VectorXd lastlast_duration;
	Eigen::VectorXd duration_vector;
	Eigen::VectorXd gradient_vector;
	Eigen::VectorXd polynomialx;
	Eigen::VectorXd polynomialy;
	Eigen::VectorXd new_polynomialx;
	Eigen::VectorXd new_polynomialy;
	Eigen::MatrixXd D;

	std::vector<double> new_poly;
	std::vector<double> new_poly_x;
	std::vector<double> new_poly_y;

	direction.setZero(number);
	direction_set.setZero(number,number);
	last_duration.setZero(number);
	lastlast_duration.setZero(number);
	duration_vector.setZero(number);
	gradient_vector.setZero(number);
	last_gradient.setZero(number);
	lastlast_gradient.setZero(number);
	gradient.setZero(number);
	new_minimize_set.setZero(number);
	polynomialx.setZero(number*8);
	polynomialy.setZero(number*8);
	new_polynomialx.setZero(number*8);
	new_polynomialy.setZero(number*8);
	new_duration.setZero(number);
	D.setZero(number*8,number*8);
	new_duration_set.setZero(number,number);

	for(int k=0; k < iteration; k++) {
		//std to eigen
		for(int i=0 ; i < number*8 ; i++) {
			polynomialx(i) = poly_x[i];
			polynomialy(i) = poly_y[i];
		}

		//Hessian Matrix
		for(int i=0 ; i < number ; i++) {
			D.block<4,4>(4+i*8,4+i*8) = t8_array(duration(i));
			total_t+=duration(i);
		}

		jx = polynomialx.transpose()*D*polynomialx;
		jy = polynomialy.transpose()*D*polynomialy;

		//original cost function
		minimize_function = jx + jy;
		std::cout<< std::endl << minimize_function <<std::endl;
		std::cout<< std::endl << duration <<std::endl;

		//"Minimum Snap Trajectory Generation and Control for Quadrotors"
		//constrainted gradient approach
		for(int j=0 ; j < number ; j++) {
			for(int i=0 ; i < number ; i++) {
				if(i==j) {
					//F(T+h*gi): gi is vector that total equals zero
					new_duration_set(j,i) = duration(i) + h;
					direction_set(j,i) = h;
				} else {
					new_duration_set(j,i) = duration(i) - h/(number-1);
					direction_set(j,i) = -h/(number-1);
				}
			}
		}

		//backtracking line search
		h = step*h;
		//obtain each polynomial for each duration
		for(int j=0 ; j < number ; j++) {
			for(int i=0 ; i < number ; i++) {
				new_duration(i) = new_duration_set(j,i);
				new_total_t += new_duration(i);
				direction(i) = direction_set(j,i);
			}
			new_poly_x = qpsolve8(waypointx,number,new_duration);
			new_poly_y = qpsolve8(waypointy,number,new_duration);
			for(int i=0 ; i < number*8 ; i++) {
				new_polynomialx(i)=new_poly_x[i];
				new_polynomialy(i)=new_poly_y[i];
			}
			new_jx = new_polynomialx.transpose()*D*new_polynomialx;
			new_jy = new_polynomialy.transpose()*D*new_polynomialy;
			new_minimize_set(j) =  new_jx + new_jy;
			gradient(j) = (new_minimize_set(j) - minimize_function)/h;

			new_duration.setZero(number);
			new_polynomialx.setZero(number*8);
			new_polynomialy.setZero(number*8);
			new_total_t = 0;
		}

		double temp = 0;
		for(int j=0 ; j < number ; j++) {
			int scale=1;
			new_duration(j) = (duration(j) - alpha*gradient(j))*scale;
			temp = (direction.transpose()*gradient);
			new_minimize_function += new_minimize_set(j);
		}
		double tempp;
		tempp = (new_minimize_function - minimize_function*number) - (h*c*temp*number);
		std::cout<< std::endl<< tempp <<std::endl;

		//obtain new polynomial for duration after compress
		new_poly_x = qpsolve8(waypointx,number,new_duration);
		new_poly_y = qpsolve8(waypointy,number,new_duration);
		duration.setZero(number);
		polynomialx.setZero(number*8);
		polynomialy.setZero(number*8);
		poly_x.clear();
		poly_y.clear();
		new_minimize_function = 0;
		duration = new_duration;
		poly_x = new_poly_x;
		poly_y = new_poly_y;
		//check if converge
		if(tempp<=1) {
			std::cout<< std::endl<< k << ":converge!" <<std::endl;
			break;
		}
	}

	for(int i=0; i<number*8; i++) {
		new_poly.push_back(new_poly_x[i]);
	}
	for(int i=0; i<number*8; i++) {
		new_poly.push_back(new_poly_y[i]);
	}
	for(int i=0; i<number; i++) {
		new_poly.push_back(new_duration[i]);
	}
	return new_poly;
}

/* 
 * input: waypoint, segment size, duration
 * output: polynominal coefficient of x, y position and yaw
 */
void qptrajectory::get_profile(std::vector<segments> seg, int number, double dt,
                               std::vector<double> &poly_x,
                               std::vector<double> &poly_y,
                               std::vector<double> &poly_yaw)
{
	double total_t = 0.0;
	int iteration = 10;
	std::vector<trajectory_profile> tprofile;
	tprofile.clear();
	Eigen::Vector3d d(0, 0, 0);
	trajectory_profile data(d, d, d, d, d, 0.005);
	Eigen::VectorXd waypointx;
	Eigen::VectorXd waypointy;
	Eigen::VectorXd waypointa;
	Eigen::VectorXd duration;
	Eigen::VectorXd new_duration;
	//four element for single waypoint, additional with inner starting point
	waypointx.setZero((number+1)*4+(number-1));
	waypointy.setZero((number+1)*4+(number-1));
	waypointa.setZero((number+1)*4+(number-1));
	duration.setZero(number);
	new_duration.setZero(number);

	for(int i=0 ; i < seg.size() ; i++) {
		//all end point
		waypointx((i+1)*4) = seg[i].t_c.pos[0];
		waypointx((i+1)*4+1) = seg[i].t_c.vel[0];
		waypointx((i+1)*4+2) = seg[i].t_c.acc[0];
		waypointx((i+1)*4+3) = 0;
		waypointy((i+1)*4) = seg[i].t_c.pos[1];
		waypointy((i+1)*4+1) = seg[i].t_c.vel[1];
		waypointy((i+1)*4+2) = seg[i].t_c.acc[1];
		waypointy((i+1)*4+3) = 0;
		waypointa((i+1)*4) = seg[i].t_c.yaw[0];
	}

	//first starting point
	waypointx(0) = seg[0].b_c.pos[0] ;
	waypointx(1) = seg[0].b_c.vel[0] ;
	waypointx(2) = seg[0].b_c.acc[0] ;
	waypointx(3) = 0;
	waypointy(0) = seg[0].b_c.pos[1] ;
	waypointy(1) = seg[0].b_c.vel[1] ;
	waypointy(2) = seg[0].b_c.acc[1] ;
	waypointy(3) = 0;
	waypointa(0) = seg[0].b_c.yaw[0] ;

	//other starting point
	for(int i=0 ; i < (number-1) ; i++) {
		waypointx((number+1)*4+i) =  seg[i+1].b_c.pos[0] ;
		waypointy((number+1)*4+i) =  seg[i+1].b_c.pos[1] ;
		waypointa((number+1)*4+i) =  seg[i+1].b_c.yaw[0] ;
	}

	for(int i=0 ; i < seg.size() ; i++) {
		duration(i)=seg[i].time_interval;
		total_t+=seg[i].time_interval;
	}

	poly_x = qpsolve8(waypointx, number, duration);
	poly_y = qpsolve8(waypointy, number, duration);
	poly_yaw = qpsolve4(waypointa, number, duration);
}

double qptrajectory::polynomial(std::vector<double> data,double t)
{
	double sum = 0.0, var = 1;
	for(int i =0; i < data.size(); i++) {
		sum += data[i]*var;
		var *= t;
	}
	return sum;
}
double cpow(double t,int times)
{
	double var = 1.0;
	for(int i = 0; i<times ; i++) {
		var *= t;
	}
	return var;
}

double qptrajectory::polynomial_d1(std::vector<double> data,double t)
{
	double sum = 0.0;
	for(int i=1; i < data.size(); i++) {
		sum += (double)data[i] * i * cpow(t, i-1);
	}
	return sum;
}

double qptrajectory::polynomial_d2(std::vector<double> data,double t)
{
	double sum =0.0, var =1.0;
	for(int i=2 ; i<data.size(); i++) {
		sum += data[i] * i *(i-1)* cpow(t, i-2);
	}
	return sum;
}

double qptrajectory::polynomial_d3(std::vector<double> data,double t)
{
	double sum =0.0, var =1.0;

	for(int i = 3 ; i<data.size(); i++) {
		sum += data[i] * i *(i-1)* (i-2) * cpow(t, i-3);
	}

	return sum;
}
