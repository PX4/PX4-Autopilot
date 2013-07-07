// MESSAGE SERIAL_UDB_EXTRA_F14 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14 178

typedef struct __mavlink_serial_udb_extra_f14_t
{
 uint32_t sue_TRAP_SOURCE; ///< Serial UDB Extra Type Program Address of Last Trap
 int16_t sue_RCON; ///< Serial UDB Extra Reboot Regitster of DSPIC
 int16_t sue_TRAP_FLAGS; ///< Serial UDB Extra  Last dspic Trap Flags
 int16_t sue_osc_fail_count; ///< Serial UDB Extra Number of Ocillator Failures
 uint8_t sue_WIND_ESTIMATION; ///< Serial UDB Extra Wind Estimation Enabled
 uint8_t sue_GPS_TYPE; ///< Serial UDB Extra Type of GPS Unit
 uint8_t sue_DR; ///< Serial UDB Extra Dead Reckoning Enabled
 uint8_t sue_BOARD_TYPE; ///< Serial UDB Extra Type of UDB Hardware
 uint8_t sue_AIRFRAME; ///< Serial UDB Extra Type of Airframe
 uint8_t sue_CLOCK_CONFIG; ///< Serial UDB Extra UDB Internal Clock Configuration
 uint8_t sue_FLIGHT_PLAN_TYPE; ///< Serial UDB Extra Type of Flight Plan
} mavlink_serial_udb_extra_f14_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN 17
#define MAVLINK_MSG_ID_178_LEN 17

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_CRC 123
#define MAVLINK_MSG_ID_178_CRC 123



#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F14 { \
	"SERIAL_UDB_EXTRA_F14", \
	11, \
	{  { "sue_TRAP_SOURCE", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_serial_udb_extra_f14_t, sue_TRAP_SOURCE) }, \
         { "sue_RCON", NULL, MAVLINK_TYPE_INT16_T, 0, 4, offsetof(mavlink_serial_udb_extra_f14_t, sue_RCON) }, \
         { "sue_TRAP_FLAGS", NULL, MAVLINK_TYPE_INT16_T, 0, 6, offsetof(mavlink_serial_udb_extra_f14_t, sue_TRAP_FLAGS) }, \
         { "sue_osc_fail_count", NULL, MAVLINK_TYPE_INT16_T, 0, 8, offsetof(mavlink_serial_udb_extra_f14_t, sue_osc_fail_count) }, \
         { "sue_WIND_ESTIMATION", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_serial_udb_extra_f14_t, sue_WIND_ESTIMATION) }, \
         { "sue_GPS_TYPE", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_serial_udb_extra_f14_t, sue_GPS_TYPE) }, \
         { "sue_DR", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_serial_udb_extra_f14_t, sue_DR) }, \
         { "sue_BOARD_TYPE", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_serial_udb_extra_f14_t, sue_BOARD_TYPE) }, \
         { "sue_AIRFRAME", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_serial_udb_extra_f14_t, sue_AIRFRAME) }, \
         { "sue_CLOCK_CONFIG", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_serial_udb_extra_f14_t, sue_CLOCK_CONFIG) }, \
         { "sue_FLIGHT_PLAN_TYPE", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_serial_udb_extra_f14_t, sue_FLIGHT_PLAN_TYPE) }, \
         } \
}


/**
 * @brief Pack a serial_udb_extra_f14 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_WIND_ESTIMATION Serial UDB Extra Wind Estimation Enabled
 * @param sue_GPS_TYPE Serial UDB Extra Type of GPS Unit
 * @param sue_DR Serial UDB Extra Dead Reckoning Enabled
 * @param sue_BOARD_TYPE Serial UDB Extra Type of UDB Hardware
 * @param sue_AIRFRAME Serial UDB Extra Type of Airframe
 * @param sue_RCON Serial UDB Extra Reboot Regitster of DSPIC
 * @param sue_TRAP_FLAGS Serial UDB Extra  Last dspic Trap Flags
 * @param sue_TRAP_SOURCE Serial UDB Extra Type Program Address of Last Trap
 * @param sue_osc_fail_count Serial UDB Extra Number of Ocillator Failures
 * @param sue_CLOCK_CONFIG Serial UDB Extra UDB Internal Clock Configuration
 * @param sue_FLIGHT_PLAN_TYPE Serial UDB Extra Type of Flight Plan
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f14_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t sue_WIND_ESTIMATION, uint8_t sue_GPS_TYPE, uint8_t sue_DR, uint8_t sue_BOARD_TYPE, uint8_t sue_AIRFRAME, int16_t sue_RCON, int16_t sue_TRAP_FLAGS, uint32_t sue_TRAP_SOURCE, int16_t sue_osc_fail_count, uint8_t sue_CLOCK_CONFIG, uint8_t sue_FLIGHT_PLAN_TYPE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN];
	_mav_put_uint32_t(buf, 0, sue_TRAP_SOURCE);
	_mav_put_int16_t(buf, 4, sue_RCON);
	_mav_put_int16_t(buf, 6, sue_TRAP_FLAGS);
	_mav_put_int16_t(buf, 8, sue_osc_fail_count);
	_mav_put_uint8_t(buf, 10, sue_WIND_ESTIMATION);
	_mav_put_uint8_t(buf, 11, sue_GPS_TYPE);
	_mav_put_uint8_t(buf, 12, sue_DR);
	_mav_put_uint8_t(buf, 13, sue_BOARD_TYPE);
	_mav_put_uint8_t(buf, 14, sue_AIRFRAME);
	_mav_put_uint8_t(buf, 15, sue_CLOCK_CONFIG);
	_mav_put_uint8_t(buf, 16, sue_FLIGHT_PLAN_TYPE);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN);
#else
	mavlink_serial_udb_extra_f14_t packet;
	packet.sue_TRAP_SOURCE = sue_TRAP_SOURCE;
	packet.sue_RCON = sue_RCON;
	packet.sue_TRAP_FLAGS = sue_TRAP_FLAGS;
	packet.sue_osc_fail_count = sue_osc_fail_count;
	packet.sue_WIND_ESTIMATION = sue_WIND_ESTIMATION;
	packet.sue_GPS_TYPE = sue_GPS_TYPE;
	packet.sue_DR = sue_DR;
	packet.sue_BOARD_TYPE = sue_BOARD_TYPE;
	packet.sue_AIRFRAME = sue_AIRFRAME;
	packet.sue_CLOCK_CONFIG = sue_CLOCK_CONFIG;
	packet.sue_FLIGHT_PLAN_TYPE = sue_FLIGHT_PLAN_TYPE;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN);
#endif
}

/**
 * @brief Pack a serial_udb_extra_f14 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_WIND_ESTIMATION Serial UDB Extra Wind Estimation Enabled
 * @param sue_GPS_TYPE Serial UDB Extra Type of GPS Unit
 * @param sue_DR Serial UDB Extra Dead Reckoning Enabled
 * @param sue_BOARD_TYPE Serial UDB Extra Type of UDB Hardware
 * @param sue_AIRFRAME Serial UDB Extra Type of Airframe
 * @param sue_RCON Serial UDB Extra Reboot Regitster of DSPIC
 * @param sue_TRAP_FLAGS Serial UDB Extra  Last dspic Trap Flags
 * @param sue_TRAP_SOURCE Serial UDB Extra Type Program Address of Last Trap
 * @param sue_osc_fail_count Serial UDB Extra Number of Ocillator Failures
 * @param sue_CLOCK_CONFIG Serial UDB Extra UDB Internal Clock Configuration
 * @param sue_FLIGHT_PLAN_TYPE Serial UDB Extra Type of Flight Plan
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f14_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t sue_WIND_ESTIMATION,uint8_t sue_GPS_TYPE,uint8_t sue_DR,uint8_t sue_BOARD_TYPE,uint8_t sue_AIRFRAME,int16_t sue_RCON,int16_t sue_TRAP_FLAGS,uint32_t sue_TRAP_SOURCE,int16_t sue_osc_fail_count,uint8_t sue_CLOCK_CONFIG,uint8_t sue_FLIGHT_PLAN_TYPE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN];
	_mav_put_uint32_t(buf, 0, sue_TRAP_SOURCE);
	_mav_put_int16_t(buf, 4, sue_RCON);
	_mav_put_int16_t(buf, 6, sue_TRAP_FLAGS);
	_mav_put_int16_t(buf, 8, sue_osc_fail_count);
	_mav_put_uint8_t(buf, 10, sue_WIND_ESTIMATION);
	_mav_put_uint8_t(buf, 11, sue_GPS_TYPE);
	_mav_put_uint8_t(buf, 12, sue_DR);
	_mav_put_uint8_t(buf, 13, sue_BOARD_TYPE);
	_mav_put_uint8_t(buf, 14, sue_AIRFRAME);
	_mav_put_uint8_t(buf, 15, sue_CLOCK_CONFIG);
	_mav_put_uint8_t(buf, 16, sue_FLIGHT_PLAN_TYPE);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN);
#else
	mavlink_serial_udb_extra_f14_t packet;
	packet.sue_TRAP_SOURCE = sue_TRAP_SOURCE;
	packet.sue_RCON = sue_RCON;
	packet.sue_TRAP_FLAGS = sue_TRAP_FLAGS;
	packet.sue_osc_fail_count = sue_osc_fail_count;
	packet.sue_WIND_ESTIMATION = sue_WIND_ESTIMATION;
	packet.sue_GPS_TYPE = sue_GPS_TYPE;
	packet.sue_DR = sue_DR;
	packet.sue_BOARD_TYPE = sue_BOARD_TYPE;
	packet.sue_AIRFRAME = sue_AIRFRAME;
	packet.sue_CLOCK_CONFIG = sue_CLOCK_CONFIG;
	packet.sue_FLIGHT_PLAN_TYPE = sue_FLIGHT_PLAN_TYPE;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN);
#endif
}

/**
 * @brief Encode a serial_udb_extra_f14 struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f14 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f14_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f14_t* serial_udb_extra_f14)
{
	return mavlink_msg_serial_udb_extra_f14_pack(system_id, component_id, msg, serial_udb_extra_f14->sue_WIND_ESTIMATION, serial_udb_extra_f14->sue_GPS_TYPE, serial_udb_extra_f14->sue_DR, serial_udb_extra_f14->sue_BOARD_TYPE, serial_udb_extra_f14->sue_AIRFRAME, serial_udb_extra_f14->sue_RCON, serial_udb_extra_f14->sue_TRAP_FLAGS, serial_udb_extra_f14->sue_TRAP_SOURCE, serial_udb_extra_f14->sue_osc_fail_count, serial_udb_extra_f14->sue_CLOCK_CONFIG, serial_udb_extra_f14->sue_FLIGHT_PLAN_TYPE);
}

/**
 * @brief Send a serial_udb_extra_f14 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_WIND_ESTIMATION Serial UDB Extra Wind Estimation Enabled
 * @param sue_GPS_TYPE Serial UDB Extra Type of GPS Unit
 * @param sue_DR Serial UDB Extra Dead Reckoning Enabled
 * @param sue_BOARD_TYPE Serial UDB Extra Type of UDB Hardware
 * @param sue_AIRFRAME Serial UDB Extra Type of Airframe
 * @param sue_RCON Serial UDB Extra Reboot Regitster of DSPIC
 * @param sue_TRAP_FLAGS Serial UDB Extra  Last dspic Trap Flags
 * @param sue_TRAP_SOURCE Serial UDB Extra Type Program Address of Last Trap
 * @param sue_osc_fail_count Serial UDB Extra Number of Ocillator Failures
 * @param sue_CLOCK_CONFIG Serial UDB Extra UDB Internal Clock Configuration
 * @param sue_FLIGHT_PLAN_TYPE Serial UDB Extra Type of Flight Plan
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f14_send(mavlink_channel_t chan, uint8_t sue_WIND_ESTIMATION, uint8_t sue_GPS_TYPE, uint8_t sue_DR, uint8_t sue_BOARD_TYPE, uint8_t sue_AIRFRAME, int16_t sue_RCON, int16_t sue_TRAP_FLAGS, uint32_t sue_TRAP_SOURCE, int16_t sue_osc_fail_count, uint8_t sue_CLOCK_CONFIG, uint8_t sue_FLIGHT_PLAN_TYPE)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN];
	_mav_put_uint32_t(buf, 0, sue_TRAP_SOURCE);
	_mav_put_int16_t(buf, 4, sue_RCON);
	_mav_put_int16_t(buf, 6, sue_TRAP_FLAGS);
	_mav_put_int16_t(buf, 8, sue_osc_fail_count);
	_mav_put_uint8_t(buf, 10, sue_WIND_ESTIMATION);
	_mav_put_uint8_t(buf, 11, sue_GPS_TYPE);
	_mav_put_uint8_t(buf, 12, sue_DR);
	_mav_put_uint8_t(buf, 13, sue_BOARD_TYPE);
	_mav_put_uint8_t(buf, 14, sue_AIRFRAME);
	_mav_put_uint8_t(buf, 15, sue_CLOCK_CONFIG);
	_mav_put_uint8_t(buf, 16, sue_FLIGHT_PLAN_TYPE);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN);
#endif
#else
	mavlink_serial_udb_extra_f14_t packet;
	packet.sue_TRAP_SOURCE = sue_TRAP_SOURCE;
	packet.sue_RCON = sue_RCON;
	packet.sue_TRAP_FLAGS = sue_TRAP_FLAGS;
	packet.sue_osc_fail_count = sue_osc_fail_count;
	packet.sue_WIND_ESTIMATION = sue_WIND_ESTIMATION;
	packet.sue_GPS_TYPE = sue_GPS_TYPE;
	packet.sue_DR = sue_DR;
	packet.sue_BOARD_TYPE = sue_BOARD_TYPE;
	packet.sue_AIRFRAME = sue_AIRFRAME;
	packet.sue_CLOCK_CONFIG = sue_CLOCK_CONFIG;
	packet.sue_FLIGHT_PLAN_TYPE = sue_FLIGHT_PLAN_TYPE;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN);
#endif
#endif
}

#endif

// MESSAGE SERIAL_UDB_EXTRA_F14 UNPACKING


/**
 * @brief Get field sue_WIND_ESTIMATION from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra Wind Estimation Enabled
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f14_get_sue_WIND_ESTIMATION(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field sue_GPS_TYPE from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra Type of GPS Unit
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f14_get_sue_GPS_TYPE(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field sue_DR from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra Dead Reckoning Enabled
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f14_get_sue_DR(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field sue_BOARD_TYPE from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra Type of UDB Hardware
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f14_get_sue_BOARD_TYPE(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field sue_AIRFRAME from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra Type of Airframe
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f14_get_sue_AIRFRAME(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field sue_RCON from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra Reboot Regitster of DSPIC
 */
static inline int16_t mavlink_msg_serial_udb_extra_f14_get_sue_RCON(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  4);
}

/**
 * @brief Get field sue_TRAP_FLAGS from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra  Last dspic Trap Flags
 */
static inline int16_t mavlink_msg_serial_udb_extra_f14_get_sue_TRAP_FLAGS(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  6);
}

/**
 * @brief Get field sue_TRAP_SOURCE from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra Type Program Address of Last Trap
 */
static inline uint32_t mavlink_msg_serial_udb_extra_f14_get_sue_TRAP_SOURCE(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field sue_osc_fail_count from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra Number of Ocillator Failures
 */
static inline int16_t mavlink_msg_serial_udb_extra_f14_get_sue_osc_fail_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  8);
}

/**
 * @brief Get field sue_CLOCK_CONFIG from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra UDB Internal Clock Configuration
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f14_get_sue_CLOCK_CONFIG(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field sue_FLIGHT_PLAN_TYPE from serial_udb_extra_f14 message
 *
 * @return Serial UDB Extra Type of Flight Plan
 */
static inline uint8_t mavlink_msg_serial_udb_extra_f14_get_sue_FLIGHT_PLAN_TYPE(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a serial_udb_extra_f14 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f14 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f14_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f14_t* serial_udb_extra_f14)
{
#if MAVLINK_NEED_BYTE_SWAP
	serial_udb_extra_f14->sue_TRAP_SOURCE = mavlink_msg_serial_udb_extra_f14_get_sue_TRAP_SOURCE(msg);
	serial_udb_extra_f14->sue_RCON = mavlink_msg_serial_udb_extra_f14_get_sue_RCON(msg);
	serial_udb_extra_f14->sue_TRAP_FLAGS = mavlink_msg_serial_udb_extra_f14_get_sue_TRAP_FLAGS(msg);
	serial_udb_extra_f14->sue_osc_fail_count = mavlink_msg_serial_udb_extra_f14_get_sue_osc_fail_count(msg);
	serial_udb_extra_f14->sue_WIND_ESTIMATION = mavlink_msg_serial_udb_extra_f14_get_sue_WIND_ESTIMATION(msg);
	serial_udb_extra_f14->sue_GPS_TYPE = mavlink_msg_serial_udb_extra_f14_get_sue_GPS_TYPE(msg);
	serial_udb_extra_f14->sue_DR = mavlink_msg_serial_udb_extra_f14_get_sue_DR(msg);
	serial_udb_extra_f14->sue_BOARD_TYPE = mavlink_msg_serial_udb_extra_f14_get_sue_BOARD_TYPE(msg);
	serial_udb_extra_f14->sue_AIRFRAME = mavlink_msg_serial_udb_extra_f14_get_sue_AIRFRAME(msg);
	serial_udb_extra_f14->sue_CLOCK_CONFIG = mavlink_msg_serial_udb_extra_f14_get_sue_CLOCK_CONFIG(msg);
	serial_udb_extra_f14->sue_FLIGHT_PLAN_TYPE = mavlink_msg_serial_udb_extra_f14_get_sue_FLIGHT_PLAN_TYPE(msg);
#else
	memcpy(serial_udb_extra_f14, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F14_LEN);
#endif
}
