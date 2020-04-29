#include <ros/ros.h>
#include "../lib/mavlink_v2/ncrl_mavlink/mavlink.h"
#include "ncrl_mavlink.h"
#include "pose.hpp"

uav_pose_t uav_pose;

bool received_traj_ack = false;
uint8_t traj_ack_val;

void mavlink_attitude_quaternion_handler(mavlink_message_t *received_msg)
{
	//ROS_INFO("[mavlink]received attitude message.");

	mavlink_attitude_quaternion_t attitude_quaternion;
	mavlink_msg_attitude_quaternion_decode(received_msg, &attitude_quaternion);

	uav_pose.q[0] = attitude_quaternion.q1;
	uav_pose.q[1] = attitude_quaternion.q2;
	uav_pose.q[2] = attitude_quaternion.q3;
	uav_pose.q[3] = attitude_quaternion.q4;
}

void mavlink_local_position_ned_handler(mavlink_message_t *received_msg)
{
	//ROS_INFO("[mavlink]received local position message.");

	mavlink_local_position_ned_t position_ned;
	mavlink_msg_local_position_ned_decode(received_msg, &position_ned);

	uav_pose.pos_ned[0] = position_ned.x;
	uav_pose.pos_ned[1] = position_ned.y;
	uav_pose.pos_ned[2] = position_ned.z;
	uav_pose.vel_ned[0] = position_ned.vx;
	uav_pose.vel_ned[1] = position_ned.vy;
	uav_pose.vel_ned[2] = position_ned.vz;
}

void mavlink_polynomial_trajectory_ack_handler(mavlink_message_t *received_msg)
{
	mavlink_polynomial_trajectory_ack_t traj_ack;
	mavlink_msg_polynomial_trajectory_ack_decode(received_msg, &traj_ack);

	traj_ack_val = traj_ack.ack_val;

	received_traj_ack = true;
}

bool wait_mavlink_polynomial_trajectory_ack(uint8_t *ack_val)
{
	double start_time = ros::Time::now().toSec();

	//TODO: replace with spinlock?
	while(received_traj_ack == false) {
		double current_time = ros::Time::now().toSec();
		double elapsed_time = current_time - start_time;

		/* three seconds timeout */
		if(elapsed_time >= 3) {
			return false;
		}
	}

	received_traj_ack = false;

	*ack_val = traj_ack_val;
	return true;
}
