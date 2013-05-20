// MESSAGE gps_status_var PACKING

#define MAVLINK_MSG_ID_GPS_STATUS_VAR 25

typedef struct __mavlink_gps_status_var_t
{
 uint8_t satellites_visible; ///< Number of satellites visible
 uint8_t satellite_prn[20]; ///< Global satellite ID
 uint8_t satellite_used[20]; ///< 0: Satellite not used, 1: used for localization
 uint8_t satellite_elevation[20]; ///< Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 uint8_t satellite_azimuth[20]; ///< Direction of satellite, 0: 0 deg, 255: 360 deg.
 uint8_t satellite_snr[20]; ///< Signal to noise ratio of satellite
 uint8_t satellite_prn_len; ///< [field length for] Global satellite ID
 uint8_t satellite_used_len; ///< [field length for] 0: Satellite not used, 1: used for localization
 uint8_t satellite_elevation_len; ///< [field length for] Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 uint8_t satellite_azimuth_len; ///< [field length for] Direction of satellite, 0: 0 deg, 255: 360 deg.
 uint8_t satellite_snr_len; ///< [field length for] Signal to noise ratio of satellite
} mavlink_gps_status_var_t;

#define MAVLINK_MSG_ID_GPS_STATUS_VAR_LEN 106
#define MAVLINK_MSG_ID_25_LEN 106

#define MAVLINK_MSG_GPS_STATUS_VAR_FIELD_SATELLITE_PRN_LEN 20
#define MAVLINK_MSG_GPS_STATUS_VAR_FIELD_SATELLITE_USED_LEN 20
#define MAVLINK_MSG_GPS_STATUS_VAR_FIELD_SATELLITE_ELEVATION_LEN 20
#define MAVLINK_MSG_GPS_STATUS_VAR_FIELD_SATELLITE_AZIMUTH_LEN 20
#define MAVLINK_MSG_GPS_STATUS_VAR_FIELD_SATELLITE_SNR_LEN 20

#define MAVLINK_MESSAGE_INFO_GPS_STATUS_VAR { \
	"GPS_STATUS", \
	11, \
	{  { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_status_var_t, satellites_visible) }, \
         { "satellite_prn", NULL, MAVLINK_TYPE_UINT8_T, 20, 1, offsetof(mavlink_gps_status_var_t, satellite_prn) }, \
         { "satellite_used", NULL, MAVLINK_TYPE_UINT8_T, 20, 21, offsetof(mavlink_gps_status_var_t, satellite_used) }, \
         { "satellite_elevation", NULL, MAVLINK_TYPE_UINT8_T, 20, 41, offsetof(mavlink_gps_status_var_t, satellite_elevation) }, \
         { "satellite_azimuth", NULL, MAVLINK_TYPE_UINT8_T, 20, 61, offsetof(mavlink_gps_status_var_t, satellite_azimuth) }, \
         { "satellite_snr", NULL, MAVLINK_TYPE_UINT8_T, 20, 81, offsetof(mavlink_gps_status_var_t, satellite_snr) }, \
         { "satellite_prn_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 82, offsetof(mavlink_gps_status_var_t, satellite_prn_len) }, \
         { "satellite_used_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 83, offsetof(mavlink_gps_status_var_t, satellite_used_len) }, \
         { "satellite_elevation_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 84, offsetof(mavlink_gps_status_var_t, satellite_elevation_len) }, \
         { "satellite_azimuth_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 85, offsetof(mavlink_gps_status_var_t, satellite_azimuth_len) }, \
         { "satellite_snr_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 86, offsetof(mavlink_gps_status_var_t, satellite_snr_len) }, \
         } \
}

/**
 * @brief Pack a gps_status_var message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t satellites_visible, const uint8_t *satellite_prn, uint8_t satellite_prn_len, const uint8_t *satellite_used, uint8_t satellite_used_len, const uint8_t *satellite_elevation, uint8_t satellite_elevation_len, const uint8_t *satellite_azimuth, uint8_t satellite_azimuth_len, const uint8_t *satellite_snr, uint8_t satellite_snr_len)
{
	char buf[106];
	_mav_put_uint8_t(buf, 0, satellites_visible);
	_mav_put_uint8_t_array(buf, 1, satellite_prn, satellite_prn_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len, satellite_used, satellite_used_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len + satellite_used_len, satellite_elevation, satellite_elevation_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len, satellite_azimuth, satellite_azimuth_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len, satellite_snr, satellite_snr_len);
	/* length fields sorted in the same order as the arrays, but at the end of the message */
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len, satellite_prn_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 1, satellite_used_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 2, satellite_elevation_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 3, satellite_azimuth_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 4, satellite_snr_len);
	/* calculate checksum and send */

	msg->msgid = MAVLINK_MSG_ID_GPS_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 106, 23);
}

/**
 * @brief Pack a gps_status_var message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t satellites_visible,const uint8_t *satellite_prn,const uint8_t *satellite_used,const uint8_t *satellite_elevation,const uint8_t *satellite_azimuth,const uint8_t *satellite_snr)
{
	char buf[106];
	_mav_put_uint8_t(buf, 0, satellites_visible);
	_mav_put_uint8_t_array(buf, 1, satellite_prn, satellite_prn_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len, satellite_used, satellite_used_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len + satellite_used_len, satellite_elevation, satellite_elevation_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len, satellite_azimuth, satellite_azimuth_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len, satellite_snr, satellite_snr_len);
	/* length fields sorted in the same order as the arrays, but at the end of the message */
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len, satellite_prn_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 1, satellite_used_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 2, satellite_elevation_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 3, satellite_azimuth_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 4, satellite_snr_len);
	/* calculate checksum and send */

	msg->msgid = MAVLINK_MSG_ID_GPS_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 106, 23);
}

/**
 * @brief Encode a gps_status_var struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_status_var C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_status_var_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_status_var_t* gps_status)
{
	return mavlink_msg_gps_status_pack(system_id, component_id, msg, gps_status->satellites_visible, gps_status->satellites_visible_len, gps_status->satellite_prn, gps_status->satellite_used, gps_status->satellite_elevation, gps_status->satellite_azimuth, gps_status->satellite_snr);
}

/**
 * @brief Send a gps_status_var message
 * @param chan MAVLink channel to send the message
 *
 * @param satellites_visible Number of satellites visible
 * @param satellite_prn Global satellite ID
 * @param satellite_used 0: Satellite not used, 1: used for localization
 * @param satellite_elevation Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 * @param satellite_azimuth Direction of satellite, 0: 0 deg, 255: 360 deg.
 * @param satellite_snr Signal to noise ratio of satellite
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_status_var_send(mavlink_channel_t chan, uint8_t satellites_visible, const uint8_t *satellite_prn, uint8_t satellite_prn_len, const uint8_t *satellite_used, uint8_t satellite_used_len, const uint8_t *satellite_elevation, uint8_t satellite_elevation_len, const uint8_t *satellite_azimuth, uint8_t satellite_azimuth_len, const uint8_t *satellite_snr, uint8_t satellite_snr_len)
{
	char buf[106];
	_mav_put_uint8_t(buf, 0, satellites_visible);
	_mav_put_uint8_t_array(buf, 1, satellite_prn, satellite_prn_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len, satellite_used, satellite_used_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len + satellite_used_len, satellite_elevation, satellite_elevation_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len, satellite_azimuth, satellite_azimuth_len);
	_mav_put_uint8_t_array(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len, satellite_snr, satellite_snr_len);
	/* length fields sorted in the same order as the arrays, but at the end of the message */
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len, satellite_prn_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 1, satellite_used_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 2, satellite_elevation_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 3, satellite_azimuth_len);
	_mav_put_uint8_t(buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 4, satellite_snr_len);
	/* calculate checksum and send */
	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_STATUS, buf, 1 + satellite_prn_len + satellite_used_len + satellite_elevation_len + satellite_azimuth_len + satellite_snr_len + 5, 23);
}

#endif

// MESSAGE gps_status_var UNPACKING


/**
 * @brief Get field satellites_visible from gps_status_var message
 *
 * @return Number of satellites visible
 */
static inline uint8_t mavlink_msg_gps_status_var_get_satellites_visible(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field satellite_prn from gps_status_var message
 *
 * @return Global satellite ID
 */
static inline uint16_t mavlink_msg_gps_status_var_get_satellite_prn(const mavlink_message_t* msg, uint8_t *satellite_prn)
{
	return _MAV_RETURN_uint8_t_array(msg, satellite_prn, mavlink_msg_gps_status_var_get_satellite_prn_len(msg),  1);
}

/**
 * @brief Get field satellite_used from gps_status_var message
 *
 * @return 0: Satellite not used, 1: used for localization
 */
static inline uint16_t mavlink_msg_gps_status_var_get_satellite_used(const mavlink_message_t* msg, uint8_t *satellite_used)
{
	return _MAV_RETURN_uint8_t_array(msg, satellite_used, mavlink_msg_gps_status_var_get_satellite_used_len(msg), 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(msg));
}

/**
 * @brief Get field satellite_elevation from gps_status_var message
 *
 * @return Elevation (0: right on top of receiver, 90: on the horizon) of satellite
 */
static inline uint16_t mavlink_msg_gps_status_var_get_satellite_elevation(const mavlink_message_t* msg, uint8_t *satellite_elevation)
{
	return _MAV_RETURN_uint8_t_array(msg, satellite_elevation, mavlink_msg_gps_status_var_get_satellite_elevation_len(msg), 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(msg) + mavlink_msg_gps_status_var_get_satellite_used_len(msg));
}

/**
 * @brief Get field satellite_azimuth from gps_status_var message
 *
 * @return Direction of satellite, 0: 0 deg, 255: 360 deg.
 */
static inline uint16_t mavlink_msg_gps_status_var_get_satellite_azimuth(const mavlink_message_t* msg, uint8_t *satellite_azimuth)
{
	return _MAV_RETURN_uint8_t_array(msg, satellite_azimuth, mavlink_msg_gps_status_var_get_satellite_azimuth_len(msg),  1 + mavlink_msg_gps_status_var_get_satellite_prn_len(msg) + mavlink_msg_gps_status_var_get_satellite_used_len(msg) + mavlink_msg_gps_status_var_get_satellite_elevation_len(msg);
}

/**
 * @brief Get field satellite_snr from gps_status_var message
 *
 * @return Signal to noise ratio of satellite
 */
static inline uint16_t mavlink_msg_gps_status_var_get_satellite_snr(const mavlink_message_t* msg, uint8_t *satellite_snr)
{
	return _MAV_RETURN_uint8_t_array(msg, satellite_snr, mavlink_msg_gps_status_var_get_satellite_snr_len(msg),  1 + mavlink_msg_gps_status_var_get_satellite_prn_len(msg) + mavlink_msg_gps_status_var_get_satellite_used_len(msg) + mavlink_msg_gps_status_var_get_satellite_elevation_len(msg) + mavlink_msg_gps_status_var_get_satellite_azimuth_len(msg);
}

/**
 * @brief Get the length of field satellite_prn from gps_status_var message
 *
 * @return Number of items in the array
 */
static inline uint8_t mavlink_msg_gps_status_var_get_satellite_prn_len(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  msg->len - 5);
}

/**
 * @brief Get the length of field satellite_used from gps_status_var message
 *
 * @return Number of items in the array
 */
static inline uint8_t mavlink_msg_gps_status_var_get_satellite_used_len(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  msg->len - 4);
}

/**
 * @brief Get the length of field satellite_elevation from gps_status_var message
 *
 * @return Number of items in the array
 */
static inline uint8_t mavlink_msg_gps_status_var_get_satellite_elevation_len(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  msg->len - 3);
}

/**
 * @brief Get the length of field satellite_azimuth from gps_status_var message
 *
 * @return Number of items in the array
 */
static inline uint8_t mavlink_msg_gps_status_var_get_satellite_azimuth_len(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  msg->len - 2);
}

/**
 * @brief Get the length of field satellite_snr from gps_status_var message
 *
 * @return Number of items in the array
 */
static inline uint8_t mavlink_msg_gps_status_var_get_satellite_snr_len(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  msg->len - 1);
}

/**
 * @brief Decode a gps_status_var message into a struct
 *
 * @param msg The message to decode
 * @param gps_status_var C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_status_decode(const mavlink_message_t* msg, mavlink_gps_status_var_t* gps_status)
{
	gps_status->satellites_visible = mavlink_msg_gps_status_var_get_satellites_visible(msg);
	mavlink_msg_gps_status_var_get_satellite_prn(msg, gps_status->satellite_prn);
	mavlink_msg_gps_status_var_get_satellite_used(msg, gps_status->satellite_used);
	mavlink_msg_gps_status_var_get_satellite_elevation(msg, gps_status->satellite_elevation);
	mavlink_msg_gps_status_var_get_satellite_azimuth(msg, gps_status->satellite_azimuth);
	mavlink_msg_gps_status_var_get_satellite_snr(msg, gps_status->satellite_snr);
	gps_status->satellite_prn_len = mavlink_msg_gps_status_var_get_satellite_prn_len(msg);
	gps_status->satellite_used_len = mavlink_msg_gps_status_var_get_satellite_used_len(msg);
	gps_status->satellite_elevation_len = mavlink_msg_gps_status_var_get_satellite_elevation_len(msg);
	gps_status->satellite_azimuth_len = mavlink_msg_gps_status_var_get_satellite_azimuth_len(msg);
	gps_status->satellite_snr_len = mavlink_msg_gps_status_var_get_satellite_snr_len(msg);
}

/**
 * @brief Holds the information necessary to decode the message human-readable
 */
#define MAVLINK_MESSAGE_INFO_GPS_STATUS_VAR(_msg) { \
	"GPS_STATUS", \
	11, \
	{  { "satellites_visible", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gps_status_var_t, satellites_visible) }, \
         { "satellite_prn", NULL, MAVLINK_TYPE_UINT8_T, mavlink_msg_gps_status_var_get_satellite_prn_len(_msg), 1, offsetof(mavlink_gps_status_var_t, satellite_prn) }, \
         { "satellite_used", NULL, MAVLINK_TYPE_UINT8_T, mavlink_msg_gps_status_var_get_satellite_used_len(_msg), 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(_msg), offsetof(mavlink_gps_status_var_t, satellite_used) }, \
         { "satellite_elevation", NULL, MAVLINK_TYPE_UINT8_T, mavlink_msg_gps_status_var_get_satellite_elevation_len(_msg), 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(_msg) + mavlink_msg_gps_status_var_get_satellite_used_len(_msg), offsetof(mavlink_gps_status_var_t, satellite_elevation) }, \
         { "satellite_azimuth", NULL, MAVLINK_TYPE_UINT8_T, mavlink_msg_gps_status_var_get_satellite_azimuth_len(_msg), 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(_msg) + mavlink_msg_gps_status_var_get_satellite_used_len(_msg) + mavlink_msg_gps_status_var_get_satellite_elevation_len(_msg), offsetof(mavlink_gps_status_var_t, satellite_azimuth) }, \
         { "satellite_snr", NULL, MAVLINK_TYPE_UINT8_T, mavlink_msg_gps_status_var_get_satellite_snr_len(_msg), 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(_msg) + mavlink_msg_gps_status_var_get_satellite_used_len(_msg) + mavlink_msg_gps_status_var_get_satellite_elevation_len(_msg) + mavlink_msg_gps_status_var_get_satellite_azimuth_len(_msg), offsetof(mavlink_gps_status_var_t, satellite_snr) }, \
         { "satellite_prn_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(_msg) + mavlink_msg_gps_status_var_get_satellite_used_len(_msg) + mavlink_msg_gps_status_var_get_satellite_elevation_len(_msg) + mavlink_msg_gps_status_var_get_satellite_azimuth_len(_msg) + mavlink_msg_gps_status_var_get_satellite_snr_len(_msg), offsetof(mavlink_gps_status_var_t, satellite_prn_len) }, \
         { "satellite_used_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(_msg) + mavlink_msg_gps_status_var_get_satellite_used_len(_msg) + mavlink_msg_gps_status_var_get_satellite_elevation_len(_msg) + mavlink_msg_gps_status_var_get_satellite_azimuth_len(_msg) + mavlink_msg_gps_status_var_get_satellite_snr_len(_msg) + 1, offsetof(mavlink_gps_status_var_t, satellite_used_len) }, \
         { "satellite_elevation_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(_msg) + mavlink_msg_gps_status_var_get_satellite_used_len(_msg) + mavlink_msg_gps_status_var_get_satellite_elevation_len(_msg) + mavlink_msg_gps_status_var_get_satellite_azimuth_len(_msg) + mavlink_msg_gps_status_var_get_satellite_snr_len(_msg) + 2, offsetof(mavlink_gps_status_var_t, satellite_elevation_len) }, \
         { "satellite_azimuth_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(_msg) + mavlink_msg_gps_status_var_get_satellite_used_len(_msg) + mavlink_msg_gps_status_var_get_satellite_elevation_len(_msg) + mavlink_msg_gps_status_var_get_satellite_azimuth_len(_msg) + mavlink_msg_gps_status_var_get_satellite_snr_len(_msg) + 3, offsetof(mavlink_gps_status_var_t, satellite_azimuth_len) }, \
         { "satellite_snr_len", NULL, MAVLINK_TYPE_UINT8_T, 0, 1 + mavlink_msg_gps_status_var_get_satellite_prn_len(_msg) + mavlink_msg_gps_status_var_get_satellite_used_len(_msg) + mavlink_msg_gps_status_var_get_satellite_elevation_len(_msg) + mavlink_msg_gps_status_var_get_satellite_azimuth_len(_msg) + mavlink_msg_gps_status_var_get_satellite_snr_len(_msg) + 4, offsetof(mavlink_gps_status_var_t, satellite_snr_len) }, \
         } \
}
