#include <ros/ros.h>
#include "mavlink.h"

void mavlink_attitude_handler(mavlink_message_t *received_msg)
{
	ROS_INFO("[mavlink]received attitude message.");
}

void mavlink_local_position_ned_handler(mavlink_message_t *received_msg)
{
	ROS_INFO("[mavlink]received local position message.");
}
