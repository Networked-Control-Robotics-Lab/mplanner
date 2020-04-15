#ifndef __MAVLINK_PUBLISHER_HPP__
#define __MAVLINL_PUBLISHER_HPP__

void send_mavlink_takeoff_cmd(void);
void send_mavlink_land_cmd(void);
void send_mavlink_position_target(float *pos_enu, float *vel_enu, float *acc_enu,
                                  float yaw, float yaw_rate);

#endif
