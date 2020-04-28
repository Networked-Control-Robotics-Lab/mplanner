#ifndef __MAVLINK_PUBLISHER_HPP__
#define __MAVLINL_PUBLISHER_HPP__

void send_mavlink_trajectory_following_cmd(bool enabled);
void send_mavlink_takeoff_cmd(void);
void send_mavlink_land_cmd(void);
void send_mavlink_position_target(float *pos_enu, float *vel_enu, float *acc_enu,
                                  float yaw, float yaw_rate);


void send_mavlink_polynomial_trajectory_start(bool altitude_fixed);
void send_mavlink_polynomial_trajectory_stop();
void send_mavlink_polynomial_trajectory_write(uint8_t list_size);
void send_mavlink_polynomial_trajectory_item(float *x_coeff, float *y_coeff, float *z_coeff,
                                             float *yaw_coeff);

#endif
