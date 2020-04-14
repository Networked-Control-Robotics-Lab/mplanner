#include "pose.hpp"

void init_uav_pose(uav_pose_t *uav_pose)
{
	uav_pose->q[0] = 1.0f;
	uav_pose->q[1] = 0.0f;
	uav_pose->q[2] = 0.0f;
	uav_pose->q[3] = 0.0f;

	uav_pose->pos_ned[0] = 0.0f;
	uav_pose->pos_ned[1] = 0.0f;
	uav_pose->pos_ned[2] = 0.0f;

	uav_pose->vel_ned[0] = 0.0f;
	uav_pose->vel_ned[1] = 0.0f;
	uav_pose->vel_ned[2] = 0.0f;
}
