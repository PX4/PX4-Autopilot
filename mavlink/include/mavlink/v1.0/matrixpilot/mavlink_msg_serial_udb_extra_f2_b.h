// MESSAGE SERIAL_UDB_EXTRA_F2_B PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B 171

typedef struct __mavlink_serial_udb_extra_f2_b_t
{
 uint32_t sue_time; ///< Serial UDB Extra Time
 uint32_t sue_flags; ///< Serial UDB Extra Status Flags
 int16_t sue_pwm_input_1; ///< Serial UDB Extra PWM Input Channel 1
 int16_t sue_pwm_input_2; ///< Serial UDB Extra PWM Input Channel 2
 int16_t sue_pwm_input_3; ///< Serial UDB Extra PWM Input Channel 3
 int16_t sue_pwm_input_4; ///< Serial UDB Extra PWM Input Channel 4
 int16_t sue_pwm_input_5; ///< Serial UDB Extra PWM Input Channel 5
 int16_t sue_pwm_input_6; ///< Serial UDB Extra PWM Input Channel 6
 int16_t sue_pwm_input_7; ///< Serial UDB Extra PWM Input Channel 7
 int16_t sue_pwm_input_8; ///< Serial UDB Extra PWM Input Channel 8
 int16_t sue_pwm_input_9; ///< Serial UDB Extra PWM Input Channel 9
 int16_t sue_pwm_input_10; ///< Serial UDB Extra PWM Input Channel 10
 int16_t sue_pwm_output_1; ///< Serial UDB Extra PWM Output Channel 1
 int16_t sue_pwm_output_2; ///< Serial UDB Extra PWM Output Channel 2
 int16_t sue_pwm_output_3; ///< Serial UDB Extra PWM Output Channel 3
 int16_t sue_pwm_output_4; ///< Serial UDB Extra PWM Output Channel 4
 int16_t sue_pwm_output_5; ///< Serial UDB Extra PWM Output Channel 5
 int16_t sue_pwm_output_6; ///< Serial UDB Extra PWM Output Channel 6
 int16_t sue_pwm_output_7; ///< Serial UDB Extra PWM Output Channel 7
 int16_t sue_pwm_output_8; ///< Serial UDB Extra PWM Output Channel 8
 int16_t sue_pwm_output_9; ///< Serial UDB Extra PWM Output Channel 9
 int16_t sue_pwm_output_10; ///< Serial UDB Extra PWM Output Channel 10
 int16_t sue_imu_location_x; ///< Serial UDB Extra IMU Location X
 int16_t sue_imu_location_y; ///< Serial UDB Extra IMU Location Y
 int16_t sue_imu_location_z; ///< Serial UDB Extra IMU Location Z
 int16_t sue_osc_fails; ///< Serial UDB Extra Oscillator Failure Count
 int16_t sue_imu_velocity_x; ///< Serial UDB Extra IMU Velocity X
 int16_t sue_imu_velocity_y; ///< Serial UDB Extra IMU Velocity Y
 int16_t sue_imu_velocity_z; ///< Serial UDB Extra IMU Velocity Z
 int16_t sue_waypoint_goal_x; ///< Serial UDB Extra Current Waypoint Goal X
 int16_t sue_waypoint_goal_y; ///< Serial UDB Extra Current Waypoint Goal Y
 int16_t sue_waypoint_goal_z; ///< Serial UDB Extra Current Waypoint Goal Z
 int16_t sue_memory_stack_free; ///< Serial UDB Extra Stack Memory Free
} mavlink_serial_udb_extra_f2_b_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN 70
#define MAVLINK_MSG_ID_171_LEN 70

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_CRC 169
#define MAVLINK_MSG_ID_171_CRC 169



#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F2_B { \
	"SERIAL_UDB_EXTRA_F2_B", \
	33, \
	{  { "sue_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_time) }, \
         { "sue_flags", NULL, MAVLINK_TYPE_UINT32_T, 0, 4, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_flags) }, \
         { "sue_pwm_input_1", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_input_1) }, \
         { "sue_pwm_input_2", NULL, MAVLINK_TYPE_INT16_T, 0, 10, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_input_2) }, \
         { "sue_pwm_input_3", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_input_3) }, \
         { "sue_pwm_input_4", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_input_4) }, \
         { "sue_pwm_input_5", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_input_5) }, \
         { "sue_pwm_input_6", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_input_6) }, \
         { "sue_pwm_input_7", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_input_7) }, \
         { "sue_pwm_input_8", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_input_8) }, \
         { "sue_pwm_input_9", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_input_9) }, \
         { "sue_pwm_input_10", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_input_10) }, \
         { "sue_pwm_output_1", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_output_1) }, \
         { "sue_pwm_output_2", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_output_2) }, \
         { "sue_pwm_output_3", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_output_3) }, \
         { "sue_pwm_output_4", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_output_4) }, \
         { "sue_pwm_output_5", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_output_5) }, \
         { "sue_pwm_output_6", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_output_6) }, \
         { "sue_pwm_output_7", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_output_7) }, \
         { "sue_pwm_output_8", NULL, MAVLINK_TYPE_INT16_T, 0, 42, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_output_8) }, \
         { "sue_pwm_output_9", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_output_9) }, \
         { "sue_pwm_output_10", NULL, MAVLINK_TYPE_INT16_T, 0, 46, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_pwm_output_10) }, \
         { "sue_imu_location_x", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_imu_location_x) }, \
         { "sue_imu_location_y", NULL, MAVLINK_TYPE_INT16_T, 0, 50, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_imu_location_y) }, \
         { "sue_imu_location_z", NULL, MAVLINK_TYPE_INT16_T, 0, 52, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_imu_location_z) }, \
         { "sue_osc_fails", NULL, MAVLINK_TYPE_INT16_T, 0, 54, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_osc_fails) }, \
         { "sue_imu_velocity_x", NULL, MAVLINK_TYPE_INT16_T, 0, 56, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_imu_velocity_x) }, \
         { "sue_imu_velocity_y", NULL, MAVLINK_TYPE_INT16_T, 0, 58, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_imu_velocity_y) }, \
         { "sue_imu_velocity_z", NULL, MAVLINK_TYPE_INT16_T, 0, 60, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_imu_velocity_z) }, \
         { "sue_waypoint_goal_x", NULL, MAVLINK_TYPE_INT16_T, 0, 62, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_waypoint_goal_x) }, \
         { "sue_waypoint_goal_y", NULL, MAVLINK_TYPE_INT16_T, 0, 64, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_waypoint_goal_y) }, \
         { "sue_waypoint_goal_z", NULL, MAVLINK_TYPE_INT16_T, 0, 66, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_waypoint_goal_z) }, \
         { "sue_memory_stack_free", NULL, MAVLINK_TYPE_INT16_T, 0, 68, offsetof(mavlink_serial_udb_extra_f2_b_t, sue_memory_stack_free) }, \
         } \
}


/**
 * @brief Pack a serial_udb_extra_f2_b message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_time Serial UDB Extra Time
 * @param sue_pwm_input_1 Serial UDB Extra PWM Input Channel 1
 * @param sue_pwm_input_2 Serial UDB Extra PWM Input Channel 2
 * @param sue_pwm_input_3 Serial UDB Extra PWM Input Channel 3
 * @param sue_pwm_input_4 Serial UDB Extra PWM Input Channel 4
 * @param sue_pwm_input_5 Serial UDB Extra PWM Input Channel 5
 * @param sue_pwm_input_6 Serial UDB Extra PWM Input Channel 6
 * @param sue_pwm_input_7 Serial UDB Extra PWM Input Channel 7
 * @param sue_pwm_input_8 Serial UDB Extra PWM Input Channel 8
 * @param sue_pwm_input_9 Serial UDB Extra PWM Input Channel 9
 * @param sue_pwm_input_10 Serial UDB Extra PWM Input Channel 10
 * @param sue_pwm_output_1 Serial UDB Extra PWM Output Channel 1
 * @param sue_pwm_output_2 Serial UDB Extra PWM Output Channel 2
 * @param sue_pwm_output_3 Serial UDB Extra PWM Output Channel 3
 * @param sue_pwm_output_4 Serial UDB Extra PWM Output Channel 4
 * @param sue_pwm_output_5 Serial UDB Extra PWM Output Channel 5
 * @param sue_pwm_output_6 Serial UDB Extra PWM Output Channel 6
 * @param sue_pwm_output_7 Serial UDB Extra PWM Output Channel 7
 * @param sue_pwm_output_8 Serial UDB Extra PWM Output Channel 8
 * @param sue_pwm_output_9 Serial UDB Extra PWM Output Channel 9
 * @param sue_pwm_output_10 Serial UDB Extra PWM Output Channel 10
 * @param sue_imu_location_x Serial UDB Extra IMU Location X
 * @param sue_imu_location_y Serial UDB Extra IMU Location Y
 * @param sue_imu_location_z Serial UDB Extra IMU Location Z
 * @param sue_flags Serial UDB Extra Status Flags
 * @param sue_osc_fails Serial UDB Extra Oscillator Failure Count
 * @param sue_imu_velocity_x Serial UDB Extra IMU Velocity X
 * @param sue_imu_velocity_y Serial UDB Extra IMU Velocity Y
 * @param sue_imu_velocity_z Serial UDB Extra IMU Velocity Z
 * @param sue_waypoint_goal_x Serial UDB Extra Current Waypoint Goal X
 * @param sue_waypoint_goal_y Serial UDB Extra Current Waypoint Goal Y
 * @param sue_waypoint_goal_z Serial UDB Extra Current Waypoint Goal Z
 * @param sue_memory_stack_free Serial UDB Extra Stack Memory Free
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_b_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t sue_time, int16_t sue_pwm_input_1, int16_t sue_pwm_input_2, int16_t sue_pwm_input_3, int16_t sue_pwm_input_4, int16_t sue_pwm_input_5, int16_t sue_pwm_input_6, int16_t sue_pwm_input_7, int16_t sue_pwm_input_8, int16_t sue_pwm_input_9, int16_t sue_pwm_input_10, int16_t sue_pwm_output_1, int16_t sue_pwm_output_2, int16_t sue_pwm_output_3, int16_t sue_pwm_output_4, int16_t sue_pwm_output_5, int16_t sue_pwm_output_6, int16_t sue_pwm_output_7, int16_t sue_pwm_output_8, int16_t sue_pwm_output_9, int16_t sue_pwm_output_10, int16_t sue_imu_location_x, int16_t sue_imu_location_y, int16_t sue_imu_location_z, uint32_t sue_flags, int16_t sue_osc_fails, int16_t sue_imu_velocity_x, int16_t sue_imu_velocity_y, int16_t sue_imu_velocity_z, int16_t sue_waypoint_goal_x, int16_t sue_waypoint_goal_y, int16_t sue_waypoint_goal_z, int16_t sue_memory_stack_free)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN];
	_mav_put_uint32_t(buf, 0, sue_time);
	_mav_put_uint32_t(buf, 4, sue_flags);
	_mav_put_int16_t(buf, 8, sue_pwm_input_1);
	_mav_put_int16_t(buf, 10, sue_pwm_input_2);
	_mav_put_int16_t(buf, 12, sue_pwm_input_3);
	_mav_put_int16_t(buf, 14, sue_pwm_input_4);
	_mav_put_int16_t(buf, 16, sue_pwm_input_5);
	_mav_put_int16_t(buf, 18, sue_pwm_input_6);
	_mav_put_int16_t(buf, 20, sue_pwm_input_7);
	_mav_put_int16_t(buf, 22, sue_pwm_input_8);
	_mav_put_int16_t(buf, 24, sue_pwm_input_9);
	_mav_put_int16_t(buf, 26, sue_pwm_input_10);
	_mav_put_int16_t(buf, 28, sue_pwm_output_1);
	_mav_put_int16_t(buf, 30, sue_pwm_output_2);
	_mav_put_int16_t(buf, 32, sue_pwm_output_3);
	_mav_put_int16_t(buf, 34, sue_pwm_output_4);
	_mav_put_int16_t(buf, 36, sue_pwm_output_5);
	_mav_put_int16_t(buf, 38, sue_pwm_output_6);
	_mav_put_int16_t(buf, 40, sue_pwm_output_7);
	_mav_put_int16_t(buf, 42, sue_pwm_output_8);
	_mav_put_int16_t(buf, 44, sue_pwm_output_9);
	_mav_put_int16_t(buf, 46, sue_pwm_output_10);
	_mav_put_int16_t(buf, 48, sue_imu_location_x);
	_mav_put_int16_t(buf, 50, sue_imu_location_y);
	_mav_put_int16_t(buf, 52, sue_imu_location_z);
	_mav_put_int16_t(buf, 54, sue_osc_fails);
	_mav_put_int16_t(buf, 56, sue_imu_velocity_x);
	_mav_put_int16_t(buf, 58, sue_imu_velocity_y);
	_mav_put_int16_t(buf, 60, sue_imu_velocity_z);
	_mav_put_int16_t(buf, 62, sue_waypoint_goal_x);
	_mav_put_int16_t(buf, 64, sue_waypoint_goal_y);
	_mav_put_int16_t(buf, 66, sue_waypoint_goal_z);
	_mav_put_int16_t(buf, 68, sue_memory_stack_free);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN);
#else
	mavlink_serial_udb_extra_f2_b_t packet;
	packet.sue_time = sue_time;
	packet.sue_flags = sue_flags;
	packet.sue_pwm_input_1 = sue_pwm_input_1;
	packet.sue_pwm_input_2 = sue_pwm_input_2;
	packet.sue_pwm_input_3 = sue_pwm_input_3;
	packet.sue_pwm_input_4 = sue_pwm_input_4;
	packet.sue_pwm_input_5 = sue_pwm_input_5;
	packet.sue_pwm_input_6 = sue_pwm_input_6;
	packet.sue_pwm_input_7 = sue_pwm_input_7;
	packet.sue_pwm_input_8 = sue_pwm_input_8;
	packet.sue_pwm_input_9 = sue_pwm_input_9;
	packet.sue_pwm_input_10 = sue_pwm_input_10;
	packet.sue_pwm_output_1 = sue_pwm_output_1;
	packet.sue_pwm_output_2 = sue_pwm_output_2;
	packet.sue_pwm_output_3 = sue_pwm_output_3;
	packet.sue_pwm_output_4 = sue_pwm_output_4;
	packet.sue_pwm_output_5 = sue_pwm_output_5;
	packet.sue_pwm_output_6 = sue_pwm_output_6;
	packet.sue_pwm_output_7 = sue_pwm_output_7;
	packet.sue_pwm_output_8 = sue_pwm_output_8;
	packet.sue_pwm_output_9 = sue_pwm_output_9;
	packet.sue_pwm_output_10 = sue_pwm_output_10;
	packet.sue_imu_location_x = sue_imu_location_x;
	packet.sue_imu_location_y = sue_imu_location_y;
	packet.sue_imu_location_z = sue_imu_location_z;
	packet.sue_osc_fails = sue_osc_fails;
	packet.sue_imu_velocity_x = sue_imu_velocity_x;
	packet.sue_imu_velocity_y = sue_imu_velocity_y;
	packet.sue_imu_velocity_z = sue_imu_velocity_z;
	packet.sue_waypoint_goal_x = sue_waypoint_goal_x;
	packet.sue_waypoint_goal_y = sue_waypoint_goal_y;
	packet.sue_waypoint_goal_z = sue_waypoint_goal_z;
	packet.sue_memory_stack_free = sue_memory_stack_free;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN);
#endif
}

/**
 * @brief Pack a serial_udb_extra_f2_b message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_time Serial UDB Extra Time
 * @param sue_pwm_input_1 Serial UDB Extra PWM Input Channel 1
 * @param sue_pwm_input_2 Serial UDB Extra PWM Input Channel 2
 * @param sue_pwm_input_3 Serial UDB Extra PWM Input Channel 3
 * @param sue_pwm_input_4 Serial UDB Extra PWM Input Channel 4
 * @param sue_pwm_input_5 Serial UDB Extra PWM Input Channel 5
 * @param sue_pwm_input_6 Serial UDB Extra PWM Input Channel 6
 * @param sue_pwm_input_7 Serial UDB Extra PWM Input Channel 7
 * @param sue_pwm_input_8 Serial UDB Extra PWM Input Channel 8
 * @param sue_pwm_input_9 Serial UDB Extra PWM Input Channel 9
 * @param sue_pwm_input_10 Serial UDB Extra PWM Input Channel 10
 * @param sue_pwm_output_1 Serial UDB Extra PWM Output Channel 1
 * @param sue_pwm_output_2 Serial UDB Extra PWM Output Channel 2
 * @param sue_pwm_output_3 Serial UDB Extra PWM Output Channel 3
 * @param sue_pwm_output_4 Serial UDB Extra PWM Output Channel 4
 * @param sue_pwm_output_5 Serial UDB Extra PWM Output Channel 5
 * @param sue_pwm_output_6 Serial UDB Extra PWM Output Channel 6
 * @param sue_pwm_output_7 Serial UDB Extra PWM Output Channel 7
 * @param sue_pwm_output_8 Serial UDB Extra PWM Output Channel 8
 * @param sue_pwm_output_9 Serial UDB Extra PWM Output Channel 9
 * @param sue_pwm_output_10 Serial UDB Extra PWM Output Channel 10
 * @param sue_imu_location_x Serial UDB Extra IMU Location X
 * @param sue_imu_location_y Serial UDB Extra IMU Location Y
 * @param sue_imu_location_z Serial UDB Extra IMU Location Z
 * @param sue_flags Serial UDB Extra Status Flags
 * @param sue_osc_fails Serial UDB Extra Oscillator Failure Count
 * @param sue_imu_velocity_x Serial UDB Extra IMU Velocity X
 * @param sue_imu_velocity_y Serial UDB Extra IMU Velocity Y
 * @param sue_imu_velocity_z Serial UDB Extra IMU Velocity Z
 * @param sue_waypoint_goal_x Serial UDB Extra Current Waypoint Goal X
 * @param sue_waypoint_goal_y Serial UDB Extra Current Waypoint Goal Y
 * @param sue_waypoint_goal_z Serial UDB Extra Current Waypoint Goal Z
 * @param sue_memory_stack_free Serial UDB Extra Stack Memory Free
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_b_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t sue_time,int16_t sue_pwm_input_1,int16_t sue_pwm_input_2,int16_t sue_pwm_input_3,int16_t sue_pwm_input_4,int16_t sue_pwm_input_5,int16_t sue_pwm_input_6,int16_t sue_pwm_input_7,int16_t sue_pwm_input_8,int16_t sue_pwm_input_9,int16_t sue_pwm_input_10,int16_t sue_pwm_output_1,int16_t sue_pwm_output_2,int16_t sue_pwm_output_3,int16_t sue_pwm_output_4,int16_t sue_pwm_output_5,int16_t sue_pwm_output_6,int16_t sue_pwm_output_7,int16_t sue_pwm_output_8,int16_t sue_pwm_output_9,int16_t sue_pwm_output_10,int16_t sue_imu_location_x,int16_t sue_imu_location_y,int16_t sue_imu_location_z,uint32_t sue_flags,int16_t sue_osc_fails,int16_t sue_imu_velocity_x,int16_t sue_imu_velocity_y,int16_t sue_imu_velocity_z,int16_t sue_waypoint_goal_x,int16_t sue_waypoint_goal_y,int16_t sue_waypoint_goal_z,int16_t sue_memory_stack_free)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN];
	_mav_put_uint32_t(buf, 0, sue_time);
	_mav_put_uint32_t(buf, 4, sue_flags);
	_mav_put_int16_t(buf, 8, sue_pwm_input_1);
	_mav_put_int16_t(buf, 10, sue_pwm_input_2);
	_mav_put_int16_t(buf, 12, sue_pwm_input_3);
	_mav_put_int16_t(buf, 14, sue_pwm_input_4);
	_mav_put_int16_t(buf, 16, sue_pwm_input_5);
	_mav_put_int16_t(buf, 18, sue_pwm_input_6);
	_mav_put_int16_t(buf, 20, sue_pwm_input_7);
	_mav_put_int16_t(buf, 22, sue_pwm_input_8);
	_mav_put_int16_t(buf, 24, sue_pwm_input_9);
	_mav_put_int16_t(buf, 26, sue_pwm_input_10);
	_mav_put_int16_t(buf, 28, sue_pwm_output_1);
	_mav_put_int16_t(buf, 30, sue_pwm_output_2);
	_mav_put_int16_t(buf, 32, sue_pwm_output_3);
	_mav_put_int16_t(buf, 34, sue_pwm_output_4);
	_mav_put_int16_t(buf, 36, sue_pwm_output_5);
	_mav_put_int16_t(buf, 38, sue_pwm_output_6);
	_mav_put_int16_t(buf, 40, sue_pwm_output_7);
	_mav_put_int16_t(buf, 42, sue_pwm_output_8);
	_mav_put_int16_t(buf, 44, sue_pwm_output_9);
	_mav_put_int16_t(buf, 46, sue_pwm_output_10);
	_mav_put_int16_t(buf, 48, sue_imu_location_x);
	_mav_put_int16_t(buf, 50, sue_imu_location_y);
	_mav_put_int16_t(buf, 52, sue_imu_location_z);
	_mav_put_int16_t(buf, 54, sue_osc_fails);
	_mav_put_int16_t(buf, 56, sue_imu_velocity_x);
	_mav_put_int16_t(buf, 58, sue_imu_velocity_y);
	_mav_put_int16_t(buf, 60, sue_imu_velocity_z);
	_mav_put_int16_t(buf, 62, sue_waypoint_goal_x);
	_mav_put_int16_t(buf, 64, sue_waypoint_goal_y);
	_mav_put_int16_t(buf, 66, sue_waypoint_goal_z);
	_mav_put_int16_t(buf, 68, sue_memory_stack_free);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN);
#else
	mavlink_serial_udb_extra_f2_b_t packet;
	packet.sue_time = sue_time;
	packet.sue_flags = sue_flags;
	packet.sue_pwm_input_1 = sue_pwm_input_1;
	packet.sue_pwm_input_2 = sue_pwm_input_2;
	packet.sue_pwm_input_3 = sue_pwm_input_3;
	packet.sue_pwm_input_4 = sue_pwm_input_4;
	packet.sue_pwm_input_5 = sue_pwm_input_5;
	packet.sue_pwm_input_6 = sue_pwm_input_6;
	packet.sue_pwm_input_7 = sue_pwm_input_7;
	packet.sue_pwm_input_8 = sue_pwm_input_8;
	packet.sue_pwm_input_9 = sue_pwm_input_9;
	packet.sue_pwm_input_10 = sue_pwm_input_10;
	packet.sue_pwm_output_1 = sue_pwm_output_1;
	packet.sue_pwm_output_2 = sue_pwm_output_2;
	packet.sue_pwm_output_3 = sue_pwm_output_3;
	packet.sue_pwm_output_4 = sue_pwm_output_4;
	packet.sue_pwm_output_5 = sue_pwm_output_5;
	packet.sue_pwm_output_6 = sue_pwm_output_6;
	packet.sue_pwm_output_7 = sue_pwm_output_7;
	packet.sue_pwm_output_8 = sue_pwm_output_8;
	packet.sue_pwm_output_9 = sue_pwm_output_9;
	packet.sue_pwm_output_10 = sue_pwm_output_10;
	packet.sue_imu_location_x = sue_imu_location_x;
	packet.sue_imu_location_y = sue_imu_location_y;
	packet.sue_imu_location_z = sue_imu_location_z;
	packet.sue_osc_fails = sue_osc_fails;
	packet.sue_imu_velocity_x = sue_imu_velocity_x;
	packet.sue_imu_velocity_y = sue_imu_velocity_y;
	packet.sue_imu_velocity_z = sue_imu_velocity_z;
	packet.sue_waypoint_goal_x = sue_waypoint_goal_x;
	packet.sue_waypoint_goal_y = sue_waypoint_goal_y;
	packet.sue_waypoint_goal_z = sue_waypoint_goal_z;
	packet.sue_memory_stack_free = sue_memory_stack_free;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN);
#endif
}

/**
 * @brief Encode a serial_udb_extra_f2_b struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f2_b C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_b_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f2_b_t* serial_udb_extra_f2_b)
{
	return mavlink_msg_serial_udb_extra_f2_b_pack(system_id, component_id, msg, serial_udb_extra_f2_b->sue_time, serial_udb_extra_f2_b->sue_pwm_input_1, serial_udb_extra_f2_b->sue_pwm_input_2, serial_udb_extra_f2_b->sue_pwm_input_3, serial_udb_extra_f2_b->sue_pwm_input_4, serial_udb_extra_f2_b->sue_pwm_input_5, serial_udb_extra_f2_b->sue_pwm_input_6, serial_udb_extra_f2_b->sue_pwm_input_7, serial_udb_extra_f2_b->sue_pwm_input_8, serial_udb_extra_f2_b->sue_pwm_input_9, serial_udb_extra_f2_b->sue_pwm_input_10, serial_udb_extra_f2_b->sue_pwm_output_1, serial_udb_extra_f2_b->sue_pwm_output_2, serial_udb_extra_f2_b->sue_pwm_output_3, serial_udb_extra_f2_b->sue_pwm_output_4, serial_udb_extra_f2_b->sue_pwm_output_5, serial_udb_extra_f2_b->sue_pwm_output_6, serial_udb_extra_f2_b->sue_pwm_output_7, serial_udb_extra_f2_b->sue_pwm_output_8, serial_udb_extra_f2_b->sue_pwm_output_9, serial_udb_extra_f2_b->sue_pwm_output_10, serial_udb_extra_f2_b->sue_imu_location_x, serial_udb_extra_f2_b->sue_imu_location_y, serial_udb_extra_f2_b->sue_imu_location_z, serial_udb_extra_f2_b->sue_flags, serial_udb_extra_f2_b->sue_osc_fails, serial_udb_extra_f2_b->sue_imu_velocity_x, serial_udb_extra_f2_b->sue_imu_velocity_y, serial_udb_extra_f2_b->sue_imu_velocity_z, serial_udb_extra_f2_b->sue_waypoint_goal_x, serial_udb_extra_f2_b->sue_waypoint_goal_y, serial_udb_extra_f2_b->sue_waypoint_goal_z, serial_udb_extra_f2_b->sue_memory_stack_free);
}

/**
 * @brief Send a serial_udb_extra_f2_b message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_time Serial UDB Extra Time
 * @param sue_pwm_input_1 Serial UDB Extra PWM Input Channel 1
 * @param sue_pwm_input_2 Serial UDB Extra PWM Input Channel 2
 * @param sue_pwm_input_3 Serial UDB Extra PWM Input Channel 3
 * @param sue_pwm_input_4 Serial UDB Extra PWM Input Channel 4
 * @param sue_pwm_input_5 Serial UDB Extra PWM Input Channel 5
 * @param sue_pwm_input_6 Serial UDB Extra PWM Input Channel 6
 * @param sue_pwm_input_7 Serial UDB Extra PWM Input Channel 7
 * @param sue_pwm_input_8 Serial UDB Extra PWM Input Channel 8
 * @param sue_pwm_input_9 Serial UDB Extra PWM Input Channel 9
 * @param sue_pwm_input_10 Serial UDB Extra PWM Input Channel 10
 * @param sue_pwm_output_1 Serial UDB Extra PWM Output Channel 1
 * @param sue_pwm_output_2 Serial UDB Extra PWM Output Channel 2
 * @param sue_pwm_output_3 Serial UDB Extra PWM Output Channel 3
 * @param sue_pwm_output_4 Serial UDB Extra PWM Output Channel 4
 * @param sue_pwm_output_5 Serial UDB Extra PWM Output Channel 5
 * @param sue_pwm_output_6 Serial UDB Extra PWM Output Channel 6
 * @param sue_pwm_output_7 Serial UDB Extra PWM Output Channel 7
 * @param sue_pwm_output_8 Serial UDB Extra PWM Output Channel 8
 * @param sue_pwm_output_9 Serial UDB Extra PWM Output Channel 9
 * @param sue_pwm_output_10 Serial UDB Extra PWM Output Channel 10
 * @param sue_imu_location_x Serial UDB Extra IMU Location X
 * @param sue_imu_location_y Serial UDB Extra IMU Location Y
 * @param sue_imu_location_z Serial UDB Extra IMU Location Z
 * @param sue_flags Serial UDB Extra Status Flags
 * @param sue_osc_fails Serial UDB Extra Oscillator Failure Count
 * @param sue_imu_velocity_x Serial UDB Extra IMU Velocity X
 * @param sue_imu_velocity_y Serial UDB Extra IMU Velocity Y
 * @param sue_imu_velocity_z Serial UDB Extra IMU Velocity Z
 * @param sue_waypoint_goal_x Serial UDB Extra Current Waypoint Goal X
 * @param sue_waypoint_goal_y Serial UDB Extra Current Waypoint Goal Y
 * @param sue_waypoint_goal_z Serial UDB Extra Current Waypoint Goal Z
 * @param sue_memory_stack_free Serial UDB Extra Stack Memory Free
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f2_b_send(mavlink_channel_t chan, uint32_t sue_time, int16_t sue_pwm_input_1, int16_t sue_pwm_input_2, int16_t sue_pwm_input_3, int16_t sue_pwm_input_4, int16_t sue_pwm_input_5, int16_t sue_pwm_input_6, int16_t sue_pwm_input_7, int16_t sue_pwm_input_8, int16_t sue_pwm_input_9, int16_t sue_pwm_input_10, int16_t sue_pwm_output_1, int16_t sue_pwm_output_2, int16_t sue_pwm_output_3, int16_t sue_pwm_output_4, int16_t sue_pwm_output_5, int16_t sue_pwm_output_6, int16_t sue_pwm_output_7, int16_t sue_pwm_output_8, int16_t sue_pwm_output_9, int16_t sue_pwm_output_10, int16_t sue_imu_location_x, int16_t sue_imu_location_y, int16_t sue_imu_location_z, uint32_t sue_flags, int16_t sue_osc_fails, int16_t sue_imu_velocity_x, int16_t sue_imu_velocity_y, int16_t sue_imu_velocity_z, int16_t sue_waypoint_goal_x, int16_t sue_waypoint_goal_y, int16_t sue_waypoint_goal_z, int16_t sue_memory_stack_free)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN];
	_mav_put_uint32_t(buf, 0, sue_time);
	_mav_put_uint32_t(buf, 4, sue_flags);
	_mav_put_int16_t(buf, 8, sue_pwm_input_1);
	_mav_put_int16_t(buf, 10, sue_pwm_input_2);
	_mav_put_int16_t(buf, 12, sue_pwm_input_3);
	_mav_put_int16_t(buf, 14, sue_pwm_input_4);
	_mav_put_int16_t(buf, 16, sue_pwm_input_5);
	_mav_put_int16_t(buf, 18, sue_pwm_input_6);
	_mav_put_int16_t(buf, 20, sue_pwm_input_7);
	_mav_put_int16_t(buf, 22, sue_pwm_input_8);
	_mav_put_int16_t(buf, 24, sue_pwm_input_9);
	_mav_put_int16_t(buf, 26, sue_pwm_input_10);
	_mav_put_int16_t(buf, 28, sue_pwm_output_1);
	_mav_put_int16_t(buf, 30, sue_pwm_output_2);
	_mav_put_int16_t(buf, 32, sue_pwm_output_3);
	_mav_put_int16_t(buf, 34, sue_pwm_output_4);
	_mav_put_int16_t(buf, 36, sue_pwm_output_5);
	_mav_put_int16_t(buf, 38, sue_pwm_output_6);
	_mav_put_int16_t(buf, 40, sue_pwm_output_7);
	_mav_put_int16_t(buf, 42, sue_pwm_output_8);
	_mav_put_int16_t(buf, 44, sue_pwm_output_9);
	_mav_put_int16_t(buf, 46, sue_pwm_output_10);
	_mav_put_int16_t(buf, 48, sue_imu_location_x);
	_mav_put_int16_t(buf, 50, sue_imu_location_y);
	_mav_put_int16_t(buf, 52, sue_imu_location_z);
	_mav_put_int16_t(buf, 54, sue_osc_fails);
	_mav_put_int16_t(buf, 56, sue_imu_velocity_x);
	_mav_put_int16_t(buf, 58, sue_imu_velocity_y);
	_mav_put_int16_t(buf, 60, sue_imu_velocity_z);
	_mav_put_int16_t(buf, 62, sue_waypoint_goal_x);
	_mav_put_int16_t(buf, 64, sue_waypoint_goal_y);
	_mav_put_int16_t(buf, 66, sue_waypoint_goal_z);
	_mav_put_int16_t(buf, 68, sue_memory_stack_free);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN);
#endif
#else
	mavlink_serial_udb_extra_f2_b_t packet;
	packet.sue_time = sue_time;
	packet.sue_flags = sue_flags;
	packet.sue_pwm_input_1 = sue_pwm_input_1;
	packet.sue_pwm_input_2 = sue_pwm_input_2;
	packet.sue_pwm_input_3 = sue_pwm_input_3;
	packet.sue_pwm_input_4 = sue_pwm_input_4;
	packet.sue_pwm_input_5 = sue_pwm_input_5;
	packet.sue_pwm_input_6 = sue_pwm_input_6;
	packet.sue_pwm_input_7 = sue_pwm_input_7;
	packet.sue_pwm_input_8 = sue_pwm_input_8;
	packet.sue_pwm_input_9 = sue_pwm_input_9;
	packet.sue_pwm_input_10 = sue_pwm_input_10;
	packet.sue_pwm_output_1 = sue_pwm_output_1;
	packet.sue_pwm_output_2 = sue_pwm_output_2;
	packet.sue_pwm_output_3 = sue_pwm_output_3;
	packet.sue_pwm_output_4 = sue_pwm_output_4;
	packet.sue_pwm_output_5 = sue_pwm_output_5;
	packet.sue_pwm_output_6 = sue_pwm_output_6;
	packet.sue_pwm_output_7 = sue_pwm_output_7;
	packet.sue_pwm_output_8 = sue_pwm_output_8;
	packet.sue_pwm_output_9 = sue_pwm_output_9;
	packet.sue_pwm_output_10 = sue_pwm_output_10;
	packet.sue_imu_location_x = sue_imu_location_x;
	packet.sue_imu_location_y = sue_imu_location_y;
	packet.sue_imu_location_z = sue_imu_location_z;
	packet.sue_osc_fails = sue_osc_fails;
	packet.sue_imu_velocity_x = sue_imu_velocity_x;
	packet.sue_imu_velocity_y = sue_imu_velocity_y;
	packet.sue_imu_velocity_z = sue_imu_velocity_z;
	packet.sue_waypoint_goal_x = sue_waypoint_goal_x;
	packet.sue_waypoint_goal_y = sue_waypoint_goal_y;
	packet.sue_waypoint_goal_z = sue_waypoint_goal_z;
	packet.sue_memory_stack_free = sue_memory_stack_free;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN);
#endif
#endif
}

#endif

// MESSAGE SERIAL_UDB_EXTRA_F2_B UNPACKING


/**
 * @brief Get field sue_time from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra Time
 */
static inline uint32_t mavlink_msg_serial_udb_extra_f2_b_get_sue_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sue_pwm_input_1 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Input Channel 1
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field sue_pwm_input_2 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Input Channel 2
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  10);
}

/**
 * @brief Get field sue_pwm_input_3 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Input Channel 3
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field sue_pwm_input_4 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Input Channel 4
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field sue_pwm_input_5 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Input Channel 5
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field sue_pwm_input_6 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Input Channel 6
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field sue_pwm_input_7 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Input Channel 7
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field sue_pwm_input_8 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Input Channel 8
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_8(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field sue_pwm_input_9 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Input Channel 9
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_9(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field sue_pwm_input_10 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Input Channel 10
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_10(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field sue_pwm_output_1 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Output Channel 1
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field sue_pwm_output_2 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Output Channel 2
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field sue_pwm_output_3 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Output Channel 3
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field sue_pwm_output_4 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Output Channel 4
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field sue_pwm_output_5 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Output Channel 5
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  36);
}

/**
 * @brief Get field sue_pwm_output_6 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Output Channel 6
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field sue_pwm_output_7 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Output Channel 7
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Get field sue_pwm_output_8 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Output Channel 8
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_8(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  42);
}

/**
 * @brief Get field sue_pwm_output_9 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Output Channel 9
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_9(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  44);
}

/**
 * @brief Get field sue_pwm_output_10 from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra PWM Output Channel 10
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_10(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  46);
}

/**
 * @brief Get field sue_imu_location_x from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra IMU Location X
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_location_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  48);
}

/**
 * @brief Get field sue_imu_location_y from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra IMU Location Y
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_location_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  50);
}

/**
 * @brief Get field sue_imu_location_z from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra IMU Location Z
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_location_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  52);
}

/**
 * @brief Get field sue_flags from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra Status Flags
 */
static inline uint32_t mavlink_msg_serial_udb_extra_f2_b_get_sue_flags(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  4);
}

/**
 * @brief Get field sue_osc_fails from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra Oscillator Failure Count
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_osc_fails(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  54);
}

/**
 * @brief Get field sue_imu_velocity_x from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra IMU Velocity X
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_velocity_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  56);
}

/**
 * @brief Get field sue_imu_velocity_y from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra IMU Velocity Y
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_velocity_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  58);
}

/**
 * @brief Get field sue_imu_velocity_z from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra IMU Velocity Z
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_velocity_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  60);
}

/**
 * @brief Get field sue_waypoint_goal_x from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra Current Waypoint Goal X
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_waypoint_goal_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  62);
}

/**
 * @brief Get field sue_waypoint_goal_y from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra Current Waypoint Goal Y
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_waypoint_goal_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  64);
}

/**
 * @brief Get field sue_waypoint_goal_z from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra Current Waypoint Goal Z
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_waypoint_goal_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  66);
}

/**
 * @brief Get field sue_memory_stack_free from serial_udb_extra_f2_b message
 *
 * @return Serial UDB Extra Stack Memory Free
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_b_get_sue_memory_stack_free(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  68);
}

/**
 * @brief Decode a serial_udb_extra_f2_b message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f2_b C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f2_b_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f2_b_t* serial_udb_extra_f2_b)
{
#if MAVLINK_NEED_BYTE_SWAP
	serial_udb_extra_f2_b->sue_time = mavlink_msg_serial_udb_extra_f2_b_get_sue_time(msg);
	serial_udb_extra_f2_b->sue_flags = mavlink_msg_serial_udb_extra_f2_b_get_sue_flags(msg);
	serial_udb_extra_f2_b->sue_pwm_input_1 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_1(msg);
	serial_udb_extra_f2_b->sue_pwm_input_2 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_2(msg);
	serial_udb_extra_f2_b->sue_pwm_input_3 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_3(msg);
	serial_udb_extra_f2_b->sue_pwm_input_4 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_4(msg);
	serial_udb_extra_f2_b->sue_pwm_input_5 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_5(msg);
	serial_udb_extra_f2_b->sue_pwm_input_6 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_6(msg);
	serial_udb_extra_f2_b->sue_pwm_input_7 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_7(msg);
	serial_udb_extra_f2_b->sue_pwm_input_8 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_8(msg);
	serial_udb_extra_f2_b->sue_pwm_input_9 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_9(msg);
	serial_udb_extra_f2_b->sue_pwm_input_10 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_input_10(msg);
	serial_udb_extra_f2_b->sue_pwm_output_1 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_1(msg);
	serial_udb_extra_f2_b->sue_pwm_output_2 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_2(msg);
	serial_udb_extra_f2_b->sue_pwm_output_3 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_3(msg);
	serial_udb_extra_f2_b->sue_pwm_output_4 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_4(msg);
	serial_udb_extra_f2_b->sue_pwm_output_5 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_5(msg);
	serial_udb_extra_f2_b->sue_pwm_output_6 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_6(msg);
	serial_udb_extra_f2_b->sue_pwm_output_7 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_7(msg);
	serial_udb_extra_f2_b->sue_pwm_output_8 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_8(msg);
	serial_udb_extra_f2_b->sue_pwm_output_9 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_9(msg);
	serial_udb_extra_f2_b->sue_pwm_output_10 = mavlink_msg_serial_udb_extra_f2_b_get_sue_pwm_output_10(msg);
	serial_udb_extra_f2_b->sue_imu_location_x = mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_location_x(msg);
	serial_udb_extra_f2_b->sue_imu_location_y = mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_location_y(msg);
	serial_udb_extra_f2_b->sue_imu_location_z = mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_location_z(msg);
	serial_udb_extra_f2_b->sue_osc_fails = mavlink_msg_serial_udb_extra_f2_b_get_sue_osc_fails(msg);
	serial_udb_extra_f2_b->sue_imu_velocity_x = mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_velocity_x(msg);
	serial_udb_extra_f2_b->sue_imu_velocity_y = mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_velocity_y(msg);
	serial_udb_extra_f2_b->sue_imu_velocity_z = mavlink_msg_serial_udb_extra_f2_b_get_sue_imu_velocity_z(msg);
	serial_udb_extra_f2_b->sue_waypoint_goal_x = mavlink_msg_serial_udb_extra_f2_b_get_sue_waypoint_goal_x(msg);
	serial_udb_extra_f2_b->sue_waypoint_goal_y = mavlink_msg_serial_udb_extra_f2_b_get_sue_waypoint_goal_y(msg);
	serial_udb_extra_f2_b->sue_waypoint_goal_z = mavlink_msg_serial_udb_extra_f2_b_get_sue_waypoint_goal_z(msg);
	serial_udb_extra_f2_b->sue_memory_stack_free = mavlink_msg_serial_udb_extra_f2_b_get_sue_memory_stack_free(msg);
#else
	memcpy(serial_udb_extra_f2_b, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_B_LEN);
#endif
}
