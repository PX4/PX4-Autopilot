#ifndef RW_UART_DEFINE_H
#define RW_UART_DEFINE_H

#include <px4_posix.h>

#define MSG_NAME_WIFI (uint8_t)0
#define MSG_NAME_IWFI (uint8_t)1
#define MSG_NAME_HFMR (uint8_t)2
#define MSG_NAME_YFWI (uint8_t)3
#define MSG_NAME_EXYF (uint8_t)4
#define MSG_NAME_EXEX (uint8_t) 5

#define WIFI_COMM_WAYPOINT (uint8_t)81
#define WIFI_COMM_WP_DOWNLOAD (uint8_t)82
#define WIFI_COMM_WP_UPLOAD (uint8_t)83
#define WIFI_COMM_WP_UPLOAD_NUM (uint8_t)84
#define WIFI_COMM_AUTO_LAND (uint8_t)87
#define WIFI_COMM_AUTO_TAKEOFF (uint8_t)88
#define WIFI_COMM_RECEIVER_ON (uint8_t)89
#define WIFI_COMM_RECEIVER_OFF (uint8_t)90
#define WIFI_COMM_GYRO_CLEAR (uint8_t)100
#define WIFI_COMM_GET_MID (uint8_t)102
#define WIFI_COMM_WP_CHAGE (uint8_t)103
#define WIFI_COMM_PARAM_GET (uint8_t)104
#define WIFI_COMM_MAG_CALI (uint8_t)105
#define WIFI_COMM_HIGHT_CHANGE (uint8_t)106
#define WIFI_COMM_RC_POS (uint8_t)117
#define WIFI_COMM_ESC_CALI_ON (uint8_t)120
#define WIFI_COMM_CALI_QUIT (uint8_t)121
#define WIFI_COMM_AUTO_FLIGHT_ON (uint8_t)122
#define WIFI_COMM_AUTO_FLIGHT_OFF (uint8_t)123
#define WIFI_COMM_DISARMED (uint8_t)124
#define WIFI_COMM_ARMED (uint8_t)137
#define WIFI_COMM_NOHEAD_ON (uint8_t)149
#define WIFI_COMM_NOHEAD_OFF (uint8_t)150
#define WIFI_COMM_RTK_RATIO_ON (uint8_t)151
#define WIFI_COMM_RTK_RATIO_OFF (uint8_t)152
#define WIFI_COMM_ONE_MINUTE (uint8_t)201
//#define WIFI_COMM_TOUCH_CONTROL (uint8_t)255

#define YFWI_COMM_CHANGE_PARAM (uint8_t)116
#define YFWI_COMM_YAW_FORCE (uint8_t)118

#define EXYF_COMM_LOITER_YAW (uint8_t)4
#define EXYF_COMM_LOITER_MOVE (uint8_t)5
#define EXYF_COMM_PWM_SET (uint8_t)6 //not used
#define EXYF_COMM_PWM_GET (uint8_t)7 //not used
#define EXYF_COMM_IDLE_SPEED_SET (uint8_t)9
#define EXYF_COMM_IDLE_SPEED_GET (uint8_t)10
#define EXYF_COMM_RC_SET (uint8_t)13  //not used
#define EXYF_COMM_RC_GET (uint8_t)14 // not used
#define EXYF_COMM_PLANE_SET (uint8_t)18 //MAV_TYPE
#define EXYF_COMM_PLANE_GET (uint8_t)19 //MAV_TYPE

#define EXEX_COMM_HIGHT_CHANGE (uint8_t)1

#endif // RW_UART_DEFINE_H
