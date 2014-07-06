// MESSAGE SERIAL_UDB_EXTRA_F2_A PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A 170

typedef struct __mavlink_serial_udb_extra_f2_a_t
{
 uint32_t sue_time; ///< Serial UDB Extra Time
 int32_t sue_latitude; ///< Serial UDB Extra Latitude
 int32_t sue_longitude; ///< Serial UDB Extra Longitude
 int32_t sue_altitude; ///< Serial UDB Extra Altitude
 uint16_t sue_waypoint_index; ///< Serial UDB Extra Waypoint Index
 int16_t sue_rmat0; ///< Serial UDB Extra Rmat 0
 int16_t sue_rmat1; ///< Serial UDB Extra Rmat 1
 int16_t sue_rmat2; ///< Serial UDB Extra Rmat 2
 int16_t sue_rmat3; ///< Serial UDB Extra Rmat 3
 int16_t sue_rmat4; ///< Serial UDB Extra Rmat 4
 int16_t sue_rmat5; ///< Serial UDB Extra Rmat 5
 int16_t sue_rmat6; ///< Serial UDB Extra Rmat 6
 int16_t sue_rmat7; ///< Serial UDB Extra Rmat 7
 int16_t sue_rmat8; ///< Serial UDB Extra Rmat 8
 uint16_t sue_cog; ///< Serial UDB Extra GPS Course Over Ground
 int16_t sue_sog; ///< Serial UDB Extra Speed Over Ground
 uint16_t sue_cpu_load; ///< Serial UDB Extra CPU Load
 int16_t sue_voltage_milis; ///< Serial UDB Extra Voltage in MilliVolts
 uint16_t sue_air_speed_3DIMU; ///< Serial UDB Extra 3D IMU Air Speed
 int16_t sue_estimated_wind_0; ///< Serial UDB Extra Estimated Wind 0
 int16_t sue_estimated_wind_1; ///< Serial UDB Extra Estimated Wind 1
 int16_t sue_estimated_wind_2; ///< Serial UDB Extra Estimated Wind 2
 int16_t sue_magFieldEarth0; ///< Serial UDB Extra Magnetic Field Earth 0 
 int16_t sue_magFieldEarth1; ///< Serial UDB Extra Magnetic Field Earth 1 
 int16_t sue_magFieldEarth2; ///< Serial UDB Extra Magnetic Field Earth 2 
 int16_t sue_svs; ///< Serial UDB Extra Number of Sattelites in View
 int16_t sue_hdop; ///< Serial UDB Extra GPS Horizontal Dilution of Precision
 uint8_t sue_status; ///< Serial UDB Extra Status
} mavlink_serial_udb_extra_f2_a_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN 63
#define MAVLINK_MSG_ID_170_LEN 63

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_CRC 150
#define MAVLINK_MSG_ID_170_CRC 150



#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F2_A { \
	"SERIAL_UDB_EXTRA_F2_A", \
	28, \
	{  { "sue_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_time) }, \
         { "sue_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_latitude) }, \
         { "sue_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_longitude) }, \
         { "sue_altitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_altitude) }, \
         { "sue_waypoint_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_waypoint_index) }, \
         { "sue_rmat0", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_rmat0) }, \
         { "sue_rmat1", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_rmat1) }, \
         { "sue_rmat2", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_rmat2) }, \
         { "sue_rmat3", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_rmat3) }, \
         { "sue_rmat4", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_rmat4) }, \
         { "sue_rmat5", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_rmat5) }, \
         { "sue_rmat6", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_rmat6) }, \
         { "sue_rmat7", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_rmat7) }, \
         { "sue_rmat8", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_rmat8) }, \
         { "sue_cog", NULL, MAVLINK_TYPE_UINT16_T, 0, 36, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_cog) }, \
         { "sue_sog", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_sog) }, \
         { "sue_cpu_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 40, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_cpu_load) }, \
         { "sue_voltage_milis", NULL, MAVLINK_TYPE_INT16_T, 0, 42, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_voltage_milis) }, \
         { "sue_air_speed_3DIMU", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_air_speed_3DIMU) }, \
         { "sue_estimated_wind_0", NULL, MAVLINK_TYPE_INT16_T, 0, 46, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_estimated_wind_0) }, \
         { "sue_estimated_wind_1", NULL, MAVLINK_TYPE_INT16_T, 0, 48, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_estimated_wind_1) }, \
         { "sue_estimated_wind_2", NULL, MAVLINK_TYPE_INT16_T, 0, 50, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_estimated_wind_2) }, \
         { "sue_magFieldEarth0", NULL, MAVLINK_TYPE_INT16_T, 0, 52, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_magFieldEarth0) }, \
         { "sue_magFieldEarth1", NULL, MAVLINK_TYPE_INT16_T, 0, 54, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_magFieldEarth1) }, \
         { "sue_magFieldEarth2", NULL, MAVLINK_TYPE_INT16_T, 0, 56, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_magFieldEarth2) }, \
         { "sue_svs", NULL, MAVLINK_TYPE_INT16_T, 0, 58, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_svs) }, \
         { "sue_hdop", NULL, MAVLINK_TYPE_INT16_T, 0, 60, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_hdop) }, \
         { "sue_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 62, offsetof(mavlink_serial_udb_extra_f2_a_t, sue_status) }, \
         } \
}


/**
 * @brief Pack a serial_udb_extra_f2_a message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_time Serial UDB Extra Time
 * @param sue_status Serial UDB Extra Status
 * @param sue_latitude Serial UDB Extra Latitude
 * @param sue_longitude Serial UDB Extra Longitude
 * @param sue_altitude Serial UDB Extra Altitude
 * @param sue_waypoint_index Serial UDB Extra Waypoint Index
 * @param sue_rmat0 Serial UDB Extra Rmat 0
 * @param sue_rmat1 Serial UDB Extra Rmat 1
 * @param sue_rmat2 Serial UDB Extra Rmat 2
 * @param sue_rmat3 Serial UDB Extra Rmat 3
 * @param sue_rmat4 Serial UDB Extra Rmat 4
 * @param sue_rmat5 Serial UDB Extra Rmat 5
 * @param sue_rmat6 Serial UDB Extra Rmat 6
 * @param sue_rmat7 Serial UDB Extra Rmat 7
 * @param sue_rmat8 Serial UDB Extra Rmat 8
 * @param sue_cog Serial UDB Extra GPS Course Over Ground
 * @param sue_sog Serial UDB Extra Speed Over Ground
 * @param sue_cpu_load Serial UDB Extra CPU Load
 * @param sue_voltage_milis Serial UDB Extra Voltage in MilliVolts
 * @param sue_air_speed_3DIMU Serial UDB Extra 3D IMU Air Speed
 * @param sue_estimated_wind_0 Serial UDB Extra Estimated Wind 0
 * @param sue_estimated_wind_1 Serial UDB Extra Estimated Wind 1
 * @param sue_estimated_wind_2 Serial UDB Extra Estimated Wind 2
 * @param sue_magFieldEarth0 Serial UDB Extra Magnetic Field Earth 0 
 * @param sue_magFieldEarth1 Serial UDB Extra Magnetic Field Earth 1 
 * @param sue_magFieldEarth2 Serial UDB Extra Magnetic Field Earth 2 
 * @param sue_svs Serial UDB Extra Number of Sattelites in View
 * @param sue_hdop Serial UDB Extra GPS Horizontal Dilution of Precision
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_a_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t sue_time, uint8_t sue_status, int32_t sue_latitude, int32_t sue_longitude, int32_t sue_altitude, uint16_t sue_waypoint_index, int16_t sue_rmat0, int16_t sue_rmat1, int16_t sue_rmat2, int16_t sue_rmat3, int16_t sue_rmat4, int16_t sue_rmat5, int16_t sue_rmat6, int16_t sue_rmat7, int16_t sue_rmat8, uint16_t sue_cog, int16_t sue_sog, uint16_t sue_cpu_load, int16_t sue_voltage_milis, uint16_t sue_air_speed_3DIMU, int16_t sue_estimated_wind_0, int16_t sue_estimated_wind_1, int16_t sue_estimated_wind_2, int16_t sue_magFieldEarth0, int16_t sue_magFieldEarth1, int16_t sue_magFieldEarth2, int16_t sue_svs, int16_t sue_hdop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN];
	_mav_put_uint32_t(buf, 0, sue_time);
	_mav_put_int32_t(buf, 4, sue_latitude);
	_mav_put_int32_t(buf, 8, sue_longitude);
	_mav_put_int32_t(buf, 12, sue_altitude);
	_mav_put_uint16_t(buf, 16, sue_waypoint_index);
	_mav_put_int16_t(buf, 18, sue_rmat0);
	_mav_put_int16_t(buf, 20, sue_rmat1);
	_mav_put_int16_t(buf, 22, sue_rmat2);
	_mav_put_int16_t(buf, 24, sue_rmat3);
	_mav_put_int16_t(buf, 26, sue_rmat4);
	_mav_put_int16_t(buf, 28, sue_rmat5);
	_mav_put_int16_t(buf, 30, sue_rmat6);
	_mav_put_int16_t(buf, 32, sue_rmat7);
	_mav_put_int16_t(buf, 34, sue_rmat8);
	_mav_put_uint16_t(buf, 36, sue_cog);
	_mav_put_int16_t(buf, 38, sue_sog);
	_mav_put_uint16_t(buf, 40, sue_cpu_load);
	_mav_put_int16_t(buf, 42, sue_voltage_milis);
	_mav_put_uint16_t(buf, 44, sue_air_speed_3DIMU);
	_mav_put_int16_t(buf, 46, sue_estimated_wind_0);
	_mav_put_int16_t(buf, 48, sue_estimated_wind_1);
	_mav_put_int16_t(buf, 50, sue_estimated_wind_2);
	_mav_put_int16_t(buf, 52, sue_magFieldEarth0);
	_mav_put_int16_t(buf, 54, sue_magFieldEarth1);
	_mav_put_int16_t(buf, 56, sue_magFieldEarth2);
	_mav_put_int16_t(buf, 58, sue_svs);
	_mav_put_int16_t(buf, 60, sue_hdop);
	_mav_put_uint8_t(buf, 62, sue_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#else
	mavlink_serial_udb_extra_f2_a_t packet;
	packet.sue_time = sue_time;
	packet.sue_latitude = sue_latitude;
	packet.sue_longitude = sue_longitude;
	packet.sue_altitude = sue_altitude;
	packet.sue_waypoint_index = sue_waypoint_index;
	packet.sue_rmat0 = sue_rmat0;
	packet.sue_rmat1 = sue_rmat1;
	packet.sue_rmat2 = sue_rmat2;
	packet.sue_rmat3 = sue_rmat3;
	packet.sue_rmat4 = sue_rmat4;
	packet.sue_rmat5 = sue_rmat5;
	packet.sue_rmat6 = sue_rmat6;
	packet.sue_rmat7 = sue_rmat7;
	packet.sue_rmat8 = sue_rmat8;
	packet.sue_cog = sue_cog;
	packet.sue_sog = sue_sog;
	packet.sue_cpu_load = sue_cpu_load;
	packet.sue_voltage_milis = sue_voltage_milis;
	packet.sue_air_speed_3DIMU = sue_air_speed_3DIMU;
	packet.sue_estimated_wind_0 = sue_estimated_wind_0;
	packet.sue_estimated_wind_1 = sue_estimated_wind_1;
	packet.sue_estimated_wind_2 = sue_estimated_wind_2;
	packet.sue_magFieldEarth0 = sue_magFieldEarth0;
	packet.sue_magFieldEarth1 = sue_magFieldEarth1;
	packet.sue_magFieldEarth2 = sue_magFieldEarth2;
	packet.sue_svs = sue_svs;
	packet.sue_hdop = sue_hdop;
	packet.sue_status = sue_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#endif
}

/**
 * @brief Pack a serial_udb_extra_f2_a message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_time Serial UDB Extra Time
 * @param sue_status Serial UDB Extra Status
 * @param sue_latitude Serial UDB Extra Latitude
 * @param sue_longitude Serial UDB Extra Longitude
 * @param sue_altitude Serial UDB Extra Altitude
 * @param sue_waypoint_index Serial UDB Extra Waypoint Index
 * @param sue_rmat0 Serial UDB Extra Rmat 0
 * @param sue_rmat1 Serial UDB Extra Rmat 1
 * @param sue_rmat2 Serial UDB Extra Rmat 2
 * @param sue_rmat3 Serial UDB Extra Rmat 3
 * @param sue_rmat4 Serial UDB Extra Rmat 4
 * @param sue_rmat5 Serial UDB Extra Rmat 5
 * @param sue_rmat6 Serial UDB Extra Rmat 6
 * @param sue_rmat7 Serial UDB Extra Rmat 7
 * @param sue_rmat8 Serial UDB Extra Rmat 8
 * @param sue_cog Serial UDB Extra GPS Course Over Ground
 * @param sue_sog Serial UDB Extra Speed Over Ground
 * @param sue_cpu_load Serial UDB Extra CPU Load
 * @param sue_voltage_milis Serial UDB Extra Voltage in MilliVolts
 * @param sue_air_speed_3DIMU Serial UDB Extra 3D IMU Air Speed
 * @param sue_estimated_wind_0 Serial UDB Extra Estimated Wind 0
 * @param sue_estimated_wind_1 Serial UDB Extra Estimated Wind 1
 * @param sue_estimated_wind_2 Serial UDB Extra Estimated Wind 2
 * @param sue_magFieldEarth0 Serial UDB Extra Magnetic Field Earth 0 
 * @param sue_magFieldEarth1 Serial UDB Extra Magnetic Field Earth 1 
 * @param sue_magFieldEarth2 Serial UDB Extra Magnetic Field Earth 2 
 * @param sue_svs Serial UDB Extra Number of Sattelites in View
 * @param sue_hdop Serial UDB Extra GPS Horizontal Dilution of Precision
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_a_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t sue_time,uint8_t sue_status,int32_t sue_latitude,int32_t sue_longitude,int32_t sue_altitude,uint16_t sue_waypoint_index,int16_t sue_rmat0,int16_t sue_rmat1,int16_t sue_rmat2,int16_t sue_rmat3,int16_t sue_rmat4,int16_t sue_rmat5,int16_t sue_rmat6,int16_t sue_rmat7,int16_t sue_rmat8,uint16_t sue_cog,int16_t sue_sog,uint16_t sue_cpu_load,int16_t sue_voltage_milis,uint16_t sue_air_speed_3DIMU,int16_t sue_estimated_wind_0,int16_t sue_estimated_wind_1,int16_t sue_estimated_wind_2,int16_t sue_magFieldEarth0,int16_t sue_magFieldEarth1,int16_t sue_magFieldEarth2,int16_t sue_svs,int16_t sue_hdop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN];
	_mav_put_uint32_t(buf, 0, sue_time);
	_mav_put_int32_t(buf, 4, sue_latitude);
	_mav_put_int32_t(buf, 8, sue_longitude);
	_mav_put_int32_t(buf, 12, sue_altitude);
	_mav_put_uint16_t(buf, 16, sue_waypoint_index);
	_mav_put_int16_t(buf, 18, sue_rmat0);
	_mav_put_int16_t(buf, 20, sue_rmat1);
	_mav_put_int16_t(buf, 22, sue_rmat2);
	_mav_put_int16_t(buf, 24, sue_rmat3);
	_mav_put_int16_t(buf, 26, sue_rmat4);
	_mav_put_int16_t(buf, 28, sue_rmat5);
	_mav_put_int16_t(buf, 30, sue_rmat6);
	_mav_put_int16_t(buf, 32, sue_rmat7);
	_mav_put_int16_t(buf, 34, sue_rmat8);
	_mav_put_uint16_t(buf, 36, sue_cog);
	_mav_put_int16_t(buf, 38, sue_sog);
	_mav_put_uint16_t(buf, 40, sue_cpu_load);
	_mav_put_int16_t(buf, 42, sue_voltage_milis);
	_mav_put_uint16_t(buf, 44, sue_air_speed_3DIMU);
	_mav_put_int16_t(buf, 46, sue_estimated_wind_0);
	_mav_put_int16_t(buf, 48, sue_estimated_wind_1);
	_mav_put_int16_t(buf, 50, sue_estimated_wind_2);
	_mav_put_int16_t(buf, 52, sue_magFieldEarth0);
	_mav_put_int16_t(buf, 54, sue_magFieldEarth1);
	_mav_put_int16_t(buf, 56, sue_magFieldEarth2);
	_mav_put_int16_t(buf, 58, sue_svs);
	_mav_put_int16_t(buf, 60, sue_hdop);
	_mav_put_uint8_t(buf, 62, sue_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#else
	mavlink_serial_udb_extra_f2_a_t packet;
	packet.sue_time = sue_time;
	packet.sue_latitude = sue_latitude;
	packet.sue_longitude = sue_longitude;
	packet.sue_altitude = sue_altitude;
	packet.sue_waypoint_index = sue_waypoint_index;
	packet.sue_rmat0 = sue_rmat0;
	packet.sue_rmat1 = sue_rmat1;
	packet.sue_rmat2 = sue_rmat2;
	packet.sue_rmat3 = sue_rmat3;
	packet.sue_rmat4 = sue_rmat4;
	packet.sue_rmat5 = sue_rmat5;
	packet.sue_rmat6 = sue_rmat6;
	packet.sue_rmat7 = sue_rmat7;
	packet.sue_rmat8 = sue_rmat8;
	packet.sue_cog = sue_cog;
	packet.sue_sog = sue_sog;
	packet.sue_cpu_load = sue_cpu_load;
	packet.sue_voltage_milis = sue_voltage_milis;
	packet.sue_air_speed_3DIMU = sue_air_speed_3DIMU;
	packet.sue_estimated_wind_0 = sue_estimated_wind_0;
	packet.sue_estimated_wind_1 = sue_estimated_wind_1;
	packet.sue_estimated_wind_2 = sue_estimated_wind_2;
	packet.sue_magFieldEarth0 = sue_magFieldEarth0;
	packet.sue_magFieldEarth1 = sue_magFieldEarth1;
	packet.sue_magFieldEarth2 = sue_magFieldEarth2;
	packet.sue_svs = sue_svs;
	packet.sue_hdop = sue_hdop;
	packet.sue_status = sue_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#endif
}

/**
 * @brief Encode a serial_udb_extra_f2_a struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f2_a C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_a_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f2_a_t* serial_udb_extra_f2_a)
{
	return mavlink_msg_serial_udb_extra_f2_a_pack(system_id, component_id, msg, serial_udb_extra_f2_a->sue_time, serial_udb_extra_f2_a->sue_status, serial_udb_extra_f2_a->sue_latitude, serial_udb_extra_f2_a->sue_longitude, serial_udb_extra_f2_a->sue_altitude, serial_udb_extra_f2_a->sue_waypoint_index, serial_udb_extra_f2_a->sue_rmat0, serial_udb_extra_f2_a->sue_rmat1, serial_udb_extra_f2_a->sue_rmat2, serial_udb_extra_f2_a->sue_rmat3, serial_udb_extra_f2_a->sue_rmat4, serial_udb_extra_f2_a->sue_rmat5, serial_udb_extra_f2_a->sue_rmat6, serial_udb_extra_f2_a->sue_rmat7, serial_udb_extra_f2_a->sue_rmat8, serial_udb_extra_f2_a->sue_cog, serial_udb_extra_f2_a->sue_sog, serial_udb_extra_f2_a->sue_cpu_load, serial_udb_extra_f2_a->sue_voltage_milis, serial_udb_extra_f2_a->sue_air_speed_3DIMU, serial_udb_extra_f2_a->sue_estimated_wind_0, serial_udb_extra_f2_a->sue_estimated_wind_1, serial_udb_extra_f2_a->sue_estimated_wind_2, serial_udb_extra_f2_a->sue_magFieldEarth0, serial_udb_extra_f2_a->sue_magFieldEarth1, serial_udb_extra_f2_a->sue_magFieldEarth2, serial_udb_extra_f2_a->sue_svs, serial_udb_extra_f2_a->sue_hdop);
}

/**
 * @brief Encode a serial_udb_extra_f2_a struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f2_a C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_a_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_serial_udb_extra_f2_a_t* serial_udb_extra_f2_a)
{
	return mavlink_msg_serial_udb_extra_f2_a_pack_chan(system_id, component_id, chan, msg, serial_udb_extra_f2_a->sue_time, serial_udb_extra_f2_a->sue_status, serial_udb_extra_f2_a->sue_latitude, serial_udb_extra_f2_a->sue_longitude, serial_udb_extra_f2_a->sue_altitude, serial_udb_extra_f2_a->sue_waypoint_index, serial_udb_extra_f2_a->sue_rmat0, serial_udb_extra_f2_a->sue_rmat1, serial_udb_extra_f2_a->sue_rmat2, serial_udb_extra_f2_a->sue_rmat3, serial_udb_extra_f2_a->sue_rmat4, serial_udb_extra_f2_a->sue_rmat5, serial_udb_extra_f2_a->sue_rmat6, serial_udb_extra_f2_a->sue_rmat7, serial_udb_extra_f2_a->sue_rmat8, serial_udb_extra_f2_a->sue_cog, serial_udb_extra_f2_a->sue_sog, serial_udb_extra_f2_a->sue_cpu_load, serial_udb_extra_f2_a->sue_voltage_milis, serial_udb_extra_f2_a->sue_air_speed_3DIMU, serial_udb_extra_f2_a->sue_estimated_wind_0, serial_udb_extra_f2_a->sue_estimated_wind_1, serial_udb_extra_f2_a->sue_estimated_wind_2, serial_udb_extra_f2_a->sue_magFieldEarth0, serial_udb_extra_f2_a->sue_magFieldEarth1, serial_udb_extra_f2_a->sue_magFieldEarth2, serial_udb_extra_f2_a->sue_svs, serial_udb_extra_f2_a->sue_hdop);
}

/**
 * @brief Send a serial_udb_extra_f2_a message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_time Serial UDB Extra Time
 * @param sue_status Serial UDB Extra Status
 * @param sue_latitude Serial UDB Extra Latitude
 * @param sue_longitude Serial UDB Extra Longitude
 * @param sue_altitude Serial UDB Extra Altitude
 * @param sue_waypoint_index Serial UDB Extra Waypoint Index
 * @param sue_rmat0 Serial UDB Extra Rmat 0
 * @param sue_rmat1 Serial UDB Extra Rmat 1
 * @param sue_rmat2 Serial UDB Extra Rmat 2
 * @param sue_rmat3 Serial UDB Extra Rmat 3
 * @param sue_rmat4 Serial UDB Extra Rmat 4
 * @param sue_rmat5 Serial UDB Extra Rmat 5
 * @param sue_rmat6 Serial UDB Extra Rmat 6
 * @param sue_rmat7 Serial UDB Extra Rmat 7
 * @param sue_rmat8 Serial UDB Extra Rmat 8
 * @param sue_cog Serial UDB Extra GPS Course Over Ground
 * @param sue_sog Serial UDB Extra Speed Over Ground
 * @param sue_cpu_load Serial UDB Extra CPU Load
 * @param sue_voltage_milis Serial UDB Extra Voltage in MilliVolts
 * @param sue_air_speed_3DIMU Serial UDB Extra 3D IMU Air Speed
 * @param sue_estimated_wind_0 Serial UDB Extra Estimated Wind 0
 * @param sue_estimated_wind_1 Serial UDB Extra Estimated Wind 1
 * @param sue_estimated_wind_2 Serial UDB Extra Estimated Wind 2
 * @param sue_magFieldEarth0 Serial UDB Extra Magnetic Field Earth 0 
 * @param sue_magFieldEarth1 Serial UDB Extra Magnetic Field Earth 1 
 * @param sue_magFieldEarth2 Serial UDB Extra Magnetic Field Earth 2 
 * @param sue_svs Serial UDB Extra Number of Sattelites in View
 * @param sue_hdop Serial UDB Extra GPS Horizontal Dilution of Precision
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f2_a_send(mavlink_channel_t chan, uint32_t sue_time, uint8_t sue_status, int32_t sue_latitude, int32_t sue_longitude, int32_t sue_altitude, uint16_t sue_waypoint_index, int16_t sue_rmat0, int16_t sue_rmat1, int16_t sue_rmat2, int16_t sue_rmat3, int16_t sue_rmat4, int16_t sue_rmat5, int16_t sue_rmat6, int16_t sue_rmat7, int16_t sue_rmat8, uint16_t sue_cog, int16_t sue_sog, uint16_t sue_cpu_load, int16_t sue_voltage_milis, uint16_t sue_air_speed_3DIMU, int16_t sue_estimated_wind_0, int16_t sue_estimated_wind_1, int16_t sue_estimated_wind_2, int16_t sue_magFieldEarth0, int16_t sue_magFieldEarth1, int16_t sue_magFieldEarth2, int16_t sue_svs, int16_t sue_hdop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN];
	_mav_put_uint32_t(buf, 0, sue_time);
	_mav_put_int32_t(buf, 4, sue_latitude);
	_mav_put_int32_t(buf, 8, sue_longitude);
	_mav_put_int32_t(buf, 12, sue_altitude);
	_mav_put_uint16_t(buf, 16, sue_waypoint_index);
	_mav_put_int16_t(buf, 18, sue_rmat0);
	_mav_put_int16_t(buf, 20, sue_rmat1);
	_mav_put_int16_t(buf, 22, sue_rmat2);
	_mav_put_int16_t(buf, 24, sue_rmat3);
	_mav_put_int16_t(buf, 26, sue_rmat4);
	_mav_put_int16_t(buf, 28, sue_rmat5);
	_mav_put_int16_t(buf, 30, sue_rmat6);
	_mav_put_int16_t(buf, 32, sue_rmat7);
	_mav_put_int16_t(buf, 34, sue_rmat8);
	_mav_put_uint16_t(buf, 36, sue_cog);
	_mav_put_int16_t(buf, 38, sue_sog);
	_mav_put_uint16_t(buf, 40, sue_cpu_load);
	_mav_put_int16_t(buf, 42, sue_voltage_milis);
	_mav_put_uint16_t(buf, 44, sue_air_speed_3DIMU);
	_mav_put_int16_t(buf, 46, sue_estimated_wind_0);
	_mav_put_int16_t(buf, 48, sue_estimated_wind_1);
	_mav_put_int16_t(buf, 50, sue_estimated_wind_2);
	_mav_put_int16_t(buf, 52, sue_magFieldEarth0);
	_mav_put_int16_t(buf, 54, sue_magFieldEarth1);
	_mav_put_int16_t(buf, 56, sue_magFieldEarth2);
	_mav_put_int16_t(buf, 58, sue_svs);
	_mav_put_int16_t(buf, 60, sue_hdop);
	_mav_put_uint8_t(buf, 62, sue_status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#endif
#else
	mavlink_serial_udb_extra_f2_a_t packet;
	packet.sue_time = sue_time;
	packet.sue_latitude = sue_latitude;
	packet.sue_longitude = sue_longitude;
	packet.sue_altitude = sue_altitude;
	packet.sue_waypoint_index = sue_waypoint_index;
	packet.sue_rmat0 = sue_rmat0;
	packet.sue_rmat1 = sue_rmat1;
	packet.sue_rmat2 = sue_rmat2;
	packet.sue_rmat3 = sue_rmat3;
	packet.sue_rmat4 = sue_rmat4;
	packet.sue_rmat5 = sue_rmat5;
	packet.sue_rmat6 = sue_rmat6;
	packet.sue_rmat7 = sue_rmat7;
	packet.sue_rmat8 = sue_rmat8;
	packet.sue_cog = sue_cog;
	packet.sue_sog = sue_sog;
	packet.sue_cpu_load = sue_cpu_load;
	packet.sue_voltage_milis = sue_voltage_milis;
	packet.sue_air_speed_3DIMU = sue_air_speed_3DIMU;
	packet.sue_estimated_wind_0 = sue_estimated_wind_0;
	packet.sue_estimated_wind_1 = sue_estimated_wind_1;
	packet.sue_estimated_wind_2 = sue_estimated_wind_2;
	packet.sue_magFieldEarth0 = sue_magFieldEarth0;
	packet.sue_magFieldEarth1 = sue_magFieldEarth1;
	packet.sue_magFieldEarth2 = sue_magFieldEarth2;
	packet.sue_svs = sue_svs;
	packet.sue_hdop = sue_hdop;
	packet.sue_status = sue_status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_serial_udb_extra_f2_a_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t sue_time, uint8_t sue_status, int32_t sue_latitude, int32_t sue_longitude, int32_t sue_altitude, uint16_t sue_waypoint_index, int16_t sue_rmat0, int16_t sue_rmat1, int16_t sue_rmat2, int16_t sue_rmat3, int16_t sue_rmat4, int16_t sue_rmat5, int16_t sue_rmat6, int16_t sue_rmat7, int16_t sue_rmat8, uint16_t sue_cog, int16_t sue_sog, uint16_t sue_cpu_load, int16_t sue_voltage_milis, uint16_t sue_air_speed_3DIMU, int16_t sue_estimated_wind_0, int16_t sue_estimated_wind_1, int16_t sue_estimated_wind_2, int16_t sue_magFieldEarth0, int16_t sue_magFieldEarth1, int16_t sue_magFieldEarth2, int16_t sue_svs, int16_t sue_hdop)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, sue_time);
	_mav_put_int32_t(buf, 4, sue_latitude);
	_mav_put_int32_t(buf, 8, sue_longitude);
	_mav_put_int32_t(buf, 12, sue_altitude);
	_mav_put_uint16_t(buf, 16, sue_waypoint_index);
	_mav_put_int16_t(buf, 18, sue_rmat0);
	_mav_put_int16_t(buf, 20, sue_rmat1);
	_mav_put_int16_t(buf, 22, sue_rmat2);
	_mav_put_int16_t(buf, 24, sue_rmat3);
	_mav_put_int16_t(buf, 26, sue_rmat4);
	_mav_put_int16_t(buf, 28, sue_rmat5);
	_mav_put_int16_t(buf, 30, sue_rmat6);
	_mav_put_int16_t(buf, 32, sue_rmat7);
	_mav_put_int16_t(buf, 34, sue_rmat8);
	_mav_put_uint16_t(buf, 36, sue_cog);
	_mav_put_int16_t(buf, 38, sue_sog);
	_mav_put_uint16_t(buf, 40, sue_cpu_load);
	_mav_put_int16_t(buf, 42, sue_voltage_milis);
	_mav_put_uint16_t(buf, 44, sue_air_speed_3DIMU);
	_mav_put_int16_t(buf, 46, sue_estimated_wind_0);
	_mav_put_int16_t(buf, 48, sue_estimated_wind_1);
	_mav_put_int16_t(buf, 50, sue_estimated_wind_2);
	_mav_put_int16_t(buf, 52, sue_magFieldEarth0);
	_mav_put_int16_t(buf, 54, sue_magFieldEarth1);
	_mav_put_int16_t(buf, 56, sue_magFieldEarth2);
	_mav_put_int16_t(buf, 58, sue_svs);
	_mav_put_int16_t(buf, 60, sue_hdop);
	_mav_put_uint8_t(buf, 62, sue_status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#endif
#else
	mavlink_serial_udb_extra_f2_a_t *packet = (mavlink_serial_udb_extra_f2_a_t *)msgbuf;
	packet->sue_time = sue_time;
	packet->sue_latitude = sue_latitude;
	packet->sue_longitude = sue_longitude;
	packet->sue_altitude = sue_altitude;
	packet->sue_waypoint_index = sue_waypoint_index;
	packet->sue_rmat0 = sue_rmat0;
	packet->sue_rmat1 = sue_rmat1;
	packet->sue_rmat2 = sue_rmat2;
	packet->sue_rmat3 = sue_rmat3;
	packet->sue_rmat4 = sue_rmat4;
	packet->sue_rmat5 = sue_rmat5;
	packet->sue_rmat6 = sue_rmat6;
	packet->sue_rmat7 = sue_rmat7;
	packet->sue_rmat8 = sue_rmat8;
	packet->sue_cog = sue_cog;
	packet->sue_sog = sue_sog;
	packet->sue_cpu_load = sue_cpu_load;
	packet->sue_voltage_milis = sue_voltage_milis;
	packet->sue_air_speed_3DIMU = sue_air_speed_3DIMU;
	packet->sue_estimated_wind_0 = sue_estimated_wind_0;
	packet->sue_estimated_wind_1 = sue_estimated_wind_1;
	packet->sue_estimated_wind_2 = sue_estimated_wind_2;
	packet->sue_magFieldEarth0 = sue_magFieldEarth0;
	packet->sue_magFieldEarth1 = sue_magFieldEarth1;
	packet->sue_magFieldEarth2 = sue_magFieldEarth2;
	packet->sue_svs = sue_svs;
	packet->sue_hdop = sue_hdop;
	packet->sue_status = sue_status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A, (const char *)packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SERIAL_UDB_EXTRA_F2_A UNPACKING


/**
 * @brief Get field sue_time from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Time
 */
static inline uint32_t mavlink_msg_serial_udb_extra_f2_a_get_sue_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sue_status from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Status
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f2_a_get_sue_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  62);
}

/**
 * @brief Get field sue_latitude from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Latitude
 */
static inline int32_t mavlink_msg_serial_udb_extra_f2_a_get_sue_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field sue_longitude from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Longitude
 */
static inline int32_t mavlink_msg_serial_udb_extra_f2_a_get_sue_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field sue_altitude from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Altitude
 */
static inline int32_t mavlink_msg_serial_udb_extra_f2_a_get_sue_altitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field sue_waypoint_index from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Waypoint Index
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_waypoint_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field sue_rmat0 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Rmat 0
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat0(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field sue_rmat1 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Rmat 1
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field sue_rmat2 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Rmat 2
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field sue_rmat3 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Rmat 3
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat3(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field sue_rmat4 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Rmat 4
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat4(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field sue_rmat5 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Rmat 5
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat5(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field sue_rmat6 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Rmat 6
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat6(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field sue_rmat7 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Rmat 7
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat7(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field sue_rmat8 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Rmat 8
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat8(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field sue_cog from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra GPS Course Over Ground
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_cog(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  36);
}

/**
 * @brief Get field sue_sog from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Speed Over Ground
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_sog(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field sue_cpu_load from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra CPU Load
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_cpu_load(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  40);
}

/**
 * @brief Get field sue_voltage_milis from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Voltage in MilliVolts
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_voltage_milis(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  42);
}

/**
 * @brief Get field sue_air_speed_3DIMU from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra 3D IMU Air Speed
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_air_speed_3DIMU(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  44);
}

/**
 * @brief Get field sue_estimated_wind_0 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Estimated Wind 0
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_estimated_wind_0(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  46);
}

/**
 * @brief Get field sue_estimated_wind_1 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Estimated Wind 1
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_estimated_wind_1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  48);
}

/**
 * @brief Get field sue_estimated_wind_2 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Estimated Wind 2
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_estimated_wind_2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  50);
}

/**
 * @brief Get field sue_magFieldEarth0 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Magnetic Field Earth 0 
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_magFieldEarth0(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  52);
}

/**
 * @brief Get field sue_magFieldEarth1 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Magnetic Field Earth 1 
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_magFieldEarth1(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  54);
}

/**
 * @brief Get field sue_magFieldEarth2 from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Magnetic Field Earth 2 
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_magFieldEarth2(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  56);
}

/**
 * @brief Get field sue_svs from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra Number of Sattelites in View
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_svs(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  58);
}

/**
 * @brief Get field sue_hdop from serial_udb_extra_f2_a message
 *
 * @return Serial UDB Extra GPS Horizontal Dilution of Precision
 */
static inline int16_t mavlink_msg_serial_udb_extra_f2_a_get_sue_hdop(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  60);
}

/**
 * @brief Decode a serial_udb_extra_f2_a message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f2_a C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f2_a_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f2_a_t* serial_udb_extra_f2_a)
{
#if MAVLINK_NEED_BYTE_SWAP
	serial_udb_extra_f2_a->sue_time = mavlink_msg_serial_udb_extra_f2_a_get_sue_time(msg);
	serial_udb_extra_f2_a->sue_latitude = mavlink_msg_serial_udb_extra_f2_a_get_sue_latitude(msg);
	serial_udb_extra_f2_a->sue_longitude = mavlink_msg_serial_udb_extra_f2_a_get_sue_longitude(msg);
	serial_udb_extra_f2_a->sue_altitude = mavlink_msg_serial_udb_extra_f2_a_get_sue_altitude(msg);
	serial_udb_extra_f2_a->sue_waypoint_index = mavlink_msg_serial_udb_extra_f2_a_get_sue_waypoint_index(msg);
	serial_udb_extra_f2_a->sue_rmat0 = mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat0(msg);
	serial_udb_extra_f2_a->sue_rmat1 = mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat1(msg);
	serial_udb_extra_f2_a->sue_rmat2 = mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat2(msg);
	serial_udb_extra_f2_a->sue_rmat3 = mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat3(msg);
	serial_udb_extra_f2_a->sue_rmat4 = mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat4(msg);
	serial_udb_extra_f2_a->sue_rmat5 = mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat5(msg);
	serial_udb_extra_f2_a->sue_rmat6 = mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat6(msg);
	serial_udb_extra_f2_a->sue_rmat7 = mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat7(msg);
	serial_udb_extra_f2_a->sue_rmat8 = mavlink_msg_serial_udb_extra_f2_a_get_sue_rmat8(msg);
	serial_udb_extra_f2_a->sue_cog = mavlink_msg_serial_udb_extra_f2_a_get_sue_cog(msg);
	serial_udb_extra_f2_a->sue_sog = mavlink_msg_serial_udb_extra_f2_a_get_sue_sog(msg);
	serial_udb_extra_f2_a->sue_cpu_load = mavlink_msg_serial_udb_extra_f2_a_get_sue_cpu_load(msg);
	serial_udb_extra_f2_a->sue_voltage_milis = mavlink_msg_serial_udb_extra_f2_a_get_sue_voltage_milis(msg);
	serial_udb_extra_f2_a->sue_air_speed_3DIMU = mavlink_msg_serial_udb_extra_f2_a_get_sue_air_speed_3DIMU(msg);
	serial_udb_extra_f2_a->sue_estimated_wind_0 = mavlink_msg_serial_udb_extra_f2_a_get_sue_estimated_wind_0(msg);
	serial_udb_extra_f2_a->sue_estimated_wind_1 = mavlink_msg_serial_udb_extra_f2_a_get_sue_estimated_wind_1(msg);
	serial_udb_extra_f2_a->sue_estimated_wind_2 = mavlink_msg_serial_udb_extra_f2_a_get_sue_estimated_wind_2(msg);
	serial_udb_extra_f2_a->sue_magFieldEarth0 = mavlink_msg_serial_udb_extra_f2_a_get_sue_magFieldEarth0(msg);
	serial_udb_extra_f2_a->sue_magFieldEarth1 = mavlink_msg_serial_udb_extra_f2_a_get_sue_magFieldEarth1(msg);
	serial_udb_extra_f2_a->sue_magFieldEarth2 = mavlink_msg_serial_udb_extra_f2_a_get_sue_magFieldEarth2(msg);
	serial_udb_extra_f2_a->sue_svs = mavlink_msg_serial_udb_extra_f2_a_get_sue_svs(msg);
	serial_udb_extra_f2_a->sue_hdop = mavlink_msg_serial_udb_extra_f2_a_get_sue_hdop(msg);
	serial_udb_extra_f2_a->sue_status = mavlink_msg_serial_udb_extra_f2_a_get_sue_status(msg);
#else
	memcpy(serial_udb_extra_f2_a, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F2_A_LEN);
#endif
}
