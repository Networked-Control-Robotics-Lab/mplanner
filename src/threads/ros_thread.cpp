#include <math.h>
#include <thread>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include "pose.hpp"

extern uav_pose_t uav_pose; //receive new uav pose via mavlink

nav_msgs::Path path;

void ros_trajectory_waypoint_push_back(float x, float y, float z)
{
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
	new_point.header.frame_id = "world";

	path.poses.push_back(new_point);
}

void ros_thread_entry()
{
	ros::NodeHandle node;
	ros::Publisher path_pub = node.advertise<nav_msgs::Path>("trajectory", 1, true);

	tf::TransformBroadcaster tf_broadcaster;
	tf::Transform transform;

	path.header.stamp = ros::Time::now();
	path.header.frame_id = "world";

	double x = 0.0f;
	double y = 0.0f;
	double th = 0.0f;
	double vx = 0.1f;
	double vy = -0.1f;
	double vth = 0.1f;

	ros::Rate ros_timer(120);

	while(ros::ok()) {
		/* generate new point of trajectory */
		double dt = 0.1;
		double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
		double delta_y = (vx * sin(th) + vy * cos(th)) * dt;

		x += delta_x;
		y += delta_y;
		th += vth;

		ros_trajectory_waypoint_push_back(x, y , 1.5);
		path_pub.publish(path);

		/* send current uav pose message */
		tf::Matrix3x3 R;
		tf::Quaternion q(uav_pose.q[1], uav_pose.q[2], uav_pose.q[3], uav_pose.q[0]);

		transform.setOrigin(tf::Vector3(
			uav_pose.pos_ned[0], uav_pose.pos_ned[1], uav_pose.pos_ned[2]));
		transform.setRotation(q);

		tf_broadcaster.sendTransform(
			tf::StampedTransform(transform, ros::Time::now(),"origin", "world"));

		ros_timer.sleep();
	}
}

