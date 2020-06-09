#ifndef __ROS_THREAD_HPP__
#define __ROS_THREAD_HPP__

void update_uav_trajectory_position_debug(float *curr_pos, float *des_pos);
void update_uav_trajectory_velocity_debug(float *curr_vel, float *des_vel);
void update_uav_trajectory_acceleration_debug(float *accel_ff);

void ros_trajectory_waypoint_push_back(float x, float y, float z);

void ros_thread_entry();

#endif
