#ifndef __MAVLINK_PUBLISHER_HPP__
#define __MAVLINL_PUBLISHER_HPP__

void send_mavlink_takeoff_cmd(void);
void send_mavlink_land_cmd(void);
void send_mavlink_trajectory_waypoint(float *pos, float *vel, float *acc, float yaw, float yaw_rate);

#endif
