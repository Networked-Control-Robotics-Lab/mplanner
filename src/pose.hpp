#ifndef __POSE_HPP__
#define __POSE_HPP__

typedef struct {
	float q[4]; //attitude quaternion
	float pos_ned[3]; //position in ned frame
	float vel_ned[3]; //velocity in ned frame
} uav_pose_t;

void init_uav_pose(uav_pose_t *uav_pose);

#endif
