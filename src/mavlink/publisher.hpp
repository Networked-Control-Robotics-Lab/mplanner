#ifndef __MAVLINK_PUBLISHER_HPP__
#define __MAVLINL_PUBLISHER_HPP__

void send_mavlink_takeoff_cmd(uint8_t sys_id);
void send_mavlink_land_cmd(uint8_t sys_id);

void send_mavlink_goto_cmd(uint8_t sys_id, float *pos, float yaw);
void send_mavlink_halt_cmd(uint8_t sys_id);
void send_mavlink_mission_resume_cmd(uint8_t sys_id);

void send_mavlink_polynomial_trajectory_start(bool loop);
void send_mavlink_polynomial_trajectory_stop();
void send_mavlink_polynomial_trajectory_write(uint8_t list_size, bool z_enabled, bool yaw_enabled);
void send_mavlink_polynomial_trajectory_item(uint8_t index, uint8_t type, float *traj_poly_coeff,
                                             float flight_time);

#endif
