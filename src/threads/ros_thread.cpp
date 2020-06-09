#include <math.h>
#include <thread>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include "pose.hpp"
#include <mutex>

extern uav_pose_t uav_pose; //receive new uav pose via mavlink

nav_msgs::Path path;

std_msgs::Float32 curr_px, curr_py, curr_pz;
std_msgs::Float32 curr_vx, curr_vy, curr_vz;
std_msgs::Float32 des_px, des_py, des_pz;
std_msgs::Float32 des_vx, des_vy, des_vz;
std_msgs::Float32 ax_ff, ay_ff, az_ff;

std::mutex trajectory_msg_mutex;

void ros_trajectory_waypoint_push_back(float x, float y, float z)
{
	/* using mutex to protect trajectory publisher */
	trajectory_msg_mutex.lock();
	ros::Time current_time = ros::Time::now();

	geometry_msgs::PoseStamped new_point;
	new_point.pose.position.x = x;
	new_point.pose.position.y = y;
	new_point.pose.position.z = z;

	//geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(th);
	geometry_msgs::Quaternion quat;
	new_point.pose.orientation.w = 1.0f; //quat.w;
	new_point.pose.orientation.x = 0.0f; //quat.x;
	new_point.pose.orientation.y = 0.0f; //quat.y;
	new_point.pose.orientation.z = 0.0f; //quat.z;

	new_point.header.stamp = current_time;
	new_point.header.frame_id = "origin";

	path.poses.push_back(new_point);
	trajectory_msg_mutex.unlock();
}

void update_uav_trajectory_position_debug(float *curr_pos, float *des_pos)
{
	curr_px.data = curr_pos[0];
	curr_py.data = curr_pos[1];
	curr_pz.data = curr_pos[2];
	des_px.data = des_pos[0];
	des_py.data = des_pos[1];
	des_pz.data = des_pos[2];
}

void update_uav_trajectory_velocity_debug(float *curr_vel, float *des_vel)
{
	curr_vx.data = curr_vel[0];
	curr_vy.data = curr_vel[1];
	curr_vz.data = curr_vel[2];
	des_vx.data = des_vel[0];
	des_vy.data = des_vel[1];
	des_vz.data = des_vel[2];
}

void update_uav_trajectory_acceleration_debug(float *accel_ff)
{
	ax_ff.data = accel_ff[0];
	ay_ff.data = accel_ff[1];
	az_ff.data = accel_ff[2];
}

void ros_thread_entry()
{
	ros::NodeHandle node;
	ros::Publisher path_pub = node.advertise<nav_msgs::Path>("trajectory", 1, true);

	curr_px.data = 0.0f;
	curr_py.data = 0.0f;
	curr_pz.data = 0.0f;
	curr_vx.data = 0.0f;
	curr_vy.data = 0.0f;
	curr_vz.data = 0.0f;
	des_px.data = 0.0f;
	des_py.data = 0.0f;
	des_pz.data = 0.0f;
	des_vx.data = 0.0f;
	des_vy.data = 0.0f;
	des_vz.data = 0.0f;
	ax_ff.data = 0.0f;
	ay_ff.data = 0.0f;
	az_ff.data = 0.0f;

	/* current position and velocity publishers */
	ros::Publisher px_pub = node.advertise<std_msgs::Float32>("/uav/x", 100);
	ros::Publisher py_pub = node.advertise<std_msgs::Float32>("/uav/y", 100);
	ros::Publisher pz_pub = node.advertise<std_msgs::Float32>("/uav/z", 100);
	ros::Publisher vx_pub = node.advertise<std_msgs::Float32>("/uav/vx", 100);
	ros::Publisher vy_pub = node.advertise<std_msgs::Float32>("/uav/vy", 100);
	ros::Publisher vz_pub = node.advertise<std_msgs::Float32>("/uav/vz", 100);

	/* current desired position/velocity and acceleration feedforward publishers */
	ros::Publisher px_des_pub = node.advertise<std_msgs::Float32>("/uav/x_des", 100);
	ros::Publisher py_des_pub = node.advertise<std_msgs::Float32>("/uav/y_des", 100);
	ros::Publisher pz_des_pub = node.advertise<std_msgs::Float32>("/uav/z_des", 100);
	ros::Publisher vx_des_pub = node.advertise<std_msgs::Float32>("/uav/vx_des", 100);
	ros::Publisher vy_des_pub = node.advertise<std_msgs::Float32>("/uav/vy_des", 100);
	ros::Publisher vz_des_pub = node.advertise<std_msgs::Float32>("/uav/vz_des", 100);
	ros::Publisher ax_ff_pub = node.advertise<std_msgs::Float32>("/uav/ax_ff", 100);
	ros::Publisher ay_ff_pub = node.advertise<std_msgs::Float32>("/uav/ay_ff", 100);
	ros::Publisher az_ff_pub = node.advertise<std_msgs::Float32>("/uav/az_ff", 100);

	tf::TransformBroadcaster tf_broadcaster;
	tf::Transform transform;

	path.header.stamp = ros::Time::now();
	path.header.frame_id = "origin";

	ros::Rate ros_timer(120);

	while(ros::ok()) {
		/* using mutex to protect trajectory publisher */
		trajectory_msg_mutex.lock();
		path_pub.publish(path);
		trajectory_msg_mutex.unlock();

		/* send current uav pose message */
		tf::Matrix3x3 R;
		tf::Quaternion q(uav_pose.q[1], uav_pose.q[2], uav_pose.q[3], uav_pose.q[0]);

		transform.setOrigin(tf::Vector3(
		                            uav_pose.pos_ned[0], uav_pose.pos_ned[1], uav_pose.pos_ned[2]));
		transform.setRotation(q);

		tf_broadcaster.sendTransform(
		        tf::StampedTransform(transform, ros::Time::now(),"origin", "uav"));

		px_pub.publish(curr_px);
		py_pub.publish(curr_py);
		pz_pub.publish(curr_pz);
		vx_pub.publish(curr_vx);
		vy_pub.publish(curr_vy);
		vz_pub.publish(curr_vz);
	
		px_des_pub.publish(des_px);
		py_des_pub.publish(des_py);
		pz_des_pub.publish(des_pz);
		vx_des_pub.publish(des_vx);
		vy_des_pub.publish(des_vy);
		vz_des_pub.publish(des_vz);
		ax_ff_pub.publish(ax_ff);
		ay_ff_pub.publish(ay_ff);
		az_ff_pub.publish(az_ff);

		ros_timer.sleep();
	}
}

