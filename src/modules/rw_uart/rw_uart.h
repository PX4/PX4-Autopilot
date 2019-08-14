#ifndef RW_UART_H
#define RW_UART_H

#include <px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <systemlib/err.h>
#include <string.h>
#include <poll.h>

#include <arch/board/board.h>

#include <math.h>
#include <float.h>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <parameters/param.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/cpuload.h>

#pragma  pack(1)

typedef struct {
    char head[4]; //"$STP"
    float_t gps_vehicle_latitude;
    float_t gps_vehicle_longitude;
    float_t gps_wp_longitude;
    float_t gps_wp_latitude;
    float_t gps_yaw;
    uint8_t gps_num;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t wp_num;
    uint8_t rc_yaw;
    uint8_t rc_y;
    uint8_t rc_x;
    uint8_t rc_z;
    uint8_t sp_yaw;
    uint8_t sp_y;
    uint8_t sp_x;
    uint8_t sp_z;
    int8_t local_vx_high8;
    int8_t local_vx_low8;
    uint16_t total_time;
    int16_t local_vz_sp;
    uint8_t distance_high8;
    uint8_t rc_throttle_mid;
    int16_t local_z_pressure;
    int16_t gps_vx;
    uint8_t distance_low8;
    uint8_t receiver_status;
    uint8_t evh;
    uint8_t photo_num;
    uint8_t evv;
    int8_t temprature;
    int16_t acc_right;
    int16_t acc_back;
    int32_t local_pitch;
    int32_t local_roll;
    uint16_t battery_voltage;
    int16_t acc_down;
    uint8_t mission_num;
    uint8_t control_status;
    uint16_t battery_usage;
    uint8_t warnning;
    int8_t local_vz_high8;
    uint8_t rc_yaw_mid;
    uint8_t rc_roll_mid;
    uint8_t rc_pitch_mid;
    int8_t local_vy_high8;
    int16_t local_z_sp;
    uint8_t remain; //0xff
    uint16_t magnet_yaw;
    int8_t local_vz_low8;
    uint16_t flight_time;
    uint8_t battery_current;
    int8_t local_vy_low8;
    int16_t gps_vy;
    uint16_t version; //0xffff
    uint8_t sum_check;
} STP;

typedef struct {
    char head[5]; //$YFPA
    uint8_t buflen;
    uint8_t commad;
    uint8_t commad_re;
    uint8_t roll_p;
    uint8_t roll_i; // not used
    uint8_t roll_d; // not used
    uint8_t pitch_p;
    uint8_t pitch_i; // not used
    uint8_t pitch_d; // not used
    uint8_t yaw_p;
    uint8_t yaw_i; // not used
    uint8_t yaw_d; // not used
    uint8_t z_p;
    uint8_t remain_1;
    uint8_t remain_2;
    uint8_t remain_3;
    uint8_t up_vel_max;
    uint8_t xy_vel_max;
    uint8_t roll_rate;
    uint8_t pitch_rate;
    uint8_t yaw_rate; //adjust to max yawrate on auto/posctrl
    uint8_t acc_up_max;
    uint8_t remain_4;
    uint8_t yaw_max; //adjust to max yawrate on mannual
    uint8_t roll_max; //adjust to max tilt on mannual
    uint8_t pitch_max; //adjust to max tilt on auto/posctrl
    uint16_t higt_max;
    uint8_t acc_hor_max;
    uint16_t dist_max;
    uint8_t contol_style; // not set
    uint8_t calibration; // not set
    uint8_t mav_type;
    uint8_t battery_num;
    uint8_t battery_warn; // adjust  12% ----40%
    uint8_t slope_climb; //not set
    uint8_t remain_5;
    uint8_t remain_6;
    uint8_t mount_roll; // not set
    uint8_t mount_pitch; // not set
    uint8_t flight_time; // not set
    uint8_t pump_exist; //not set
    uint8_t bettery_fail; // adjust 1: rtl  2: land 3: disable
    uint8_t rc_lost_act; // adjust 1: rtl  2: land 3: disable
    uint8_t agri_fly_speed;
    uint8_t agri_spray_speed;
    uint8_t agri_spray_hor;
    uint8_t agri_flight_dist;
    uint8_t sbus_type; // 0:futaba 1:dji
    uint8_t CH9_set;
    uint8_t CH10_set;
    uint8_t CH11_set;
    uint8_t CH12_set;
    uint8_t dn_vel_max;
    uint16_t CRC_test;
}YFPA_param;

typedef struct {
char head[5];
uint8_t empty;
uint16_t waypoint_num;
int32_t lat;
int32_t lon;
int32_t alt;
uint16_t loiter_time;
uint8_t cruise_speed;
uint8_t photo_set;
uint8_t photo_dis;
uint8_t turn_mode;
uint8_t sum_check;
}SETD;

typedef struct {
   STP stp;
}MSG_send;

typedef struct {
    YFPA_param yfpa_param;
    SETD setd;
}MSG_response;

#pragma  pack()

typedef struct {
    uint8_t name;
    uint8_t command;
}MSG_type;

typedef struct {
    int arm_fd;
    int gps_fd;
    int command_fd;
    int mission_fd;
    int manual_fd;
    int status_fd;
    int local_position_sp_fd;
    int local_position_fd;
    int air_data_fd;
    int attitude_fd;
    int battery_fd;
    int geofence_fd;
    //int cpu_fd;
}MSG_orb_sub;

typedef struct {
    orb_advert_t command_pd;
}MSG_orb_pub;

typedef struct {
    struct actuator_armed_s arm_data;
    struct vehicle_gps_position_s gps_data;
    struct vehicle_command_s command_data;
    struct mission_result_s mission_data;
    struct manual_control_setpoint_s manual_data;
    struct vehicle_status_s status_data;
    struct vehicle_local_position_setpoint_s local_position_sp_data;
    struct vehicle_local_position_s local_position_data;
    struct vehicle_air_data_s air_data;
    struct vehicle_attitude_setpoint_s attitude_data;
    struct battery_status_s battery_data;
    struct geofence_result_s geofence_data;
    //struct cpuload_s cpu_data;
}MSG_orb_data;

typedef struct {
    param_t roll_p_hd;
    //param_t roll_i_hd;
    //param_t roll_d_hd;
    param_t pitch_p_hd;
    //param_t pitch_i_hd;
    //param_t pitch_d_hd;
    param_t yaw_p_hd;
    //param_t yaw_i_hd;
    //param_t yaw_d_hd;
    param_t z_p_hd;
    param_t up_vel_max_hd;
    param_t xy_vel_max_hd;
    param_t roll_rate_hd;
    param_t pitch_rate_hd;
    param_t yaw_rate_hd;
    param_t acc_up_max_hd;
    param_t yaw_max_hd;
    param_t roll_max_hd;
    param_t pitch_max_hd;
    param_t higt_max_hd;
    param_t acc_hor_max_hd;
    param_t dist_max_hd;
    //param_t contol_style_hd; // not set
    //param_t calibration_hd; // not set
    param_t mav_type_hd;
    param_t battery_n_cells_hd;
    param_t battery_warn_hd;
    //param_t slope_climb_hd; //not set
    //param_t mount_roll_hd; // not set
    //param_t mount_pitch_hd; // not set
    //param_t mount_yaw_hd; // not set
    //param_t flight_time_hd; // not set
    //param_t pump_exist_hd; //not set
    param_t battery_fail_hd;
    param_t rc_lost_act_hd;
    //param_t agri_fly_speed_hd;
    //param_t agri_spray_speed_hd;
    //param_t agri_spray_hor_hd;
    //param_t agri_flight_dist_hd;
    //param_t sbus_type_hd; // 0:futaba 1:dji
    //param_t CH9_set_hd;
    //param_t CH10_set_hd;
    //param_t CH11_set_hd;
    //param_t CH12_set_hd;
    param_t dn_vel_max_hd;
}MSG_param_hd;


extern void stp_pack (STP *stp, MSG_orb_data stp_data);

extern void wifi_pack(const uint8_t *buffer, MSG_orb_data *msg_data, MSG_type msg_type);

extern bool check_command_repeat(const uint8_t *buffer, MSG_type msg_type);

extern bool compare_buffer_n(const uint8_t *buffer1, const uint8_t *buffer2, int n);

extern bool yfwi_param_set(const uint8_t *buffer, MSG_param_hd msg_hd, uint8_t *param_saved);

extern void yfpa_param_pack(YFPA_param *yfpa_param, MSG_param_hd msg_hd);

extern uint8_t calculate_sum_check (const uint8_t *send_message);

extern uint16_t crc16_ccitt(uint8_t data, uint16_t crc);

extern uint16_t check_crc(const uint8_t *buffer, uint8_t buflen, uint8_t offset);

extern void msg_pack_send(MSG_orb_data msg_data, int uart_read);

extern void msg_pack_response(MSG_param_hd msg_hd, MSG_type msg_type, int uart_read, uint8_t *param_saved);

extern void find_r_type(uint8_t *buffer, MSG_orb_data *msg_data, MSG_orb_pub *msg_pd,
                        MSG_param_hd msg_hd, int uart_read, uint8_t *param_saved);

extern void msg_orb_pub(MSG_orb_pub *msg_pd, MSG_orb_data *msg_data, MSG_type msg_type);

extern void msg_param_saved_get(MSG_param_hd msg_hd, int uart_read, uint8_t *param_saved);

#endif // RW_UART_H
