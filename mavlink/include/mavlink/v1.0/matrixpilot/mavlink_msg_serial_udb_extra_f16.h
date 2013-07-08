// MESSAGE SERIAL_UDB_EXTRA_F16 PACKING

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16 180

typedef struct __mavlink_serial_udb_extra_f16_t
{
 uint8_t sue_ID_LEAD_PILOT[40]; ///< Serial UDB Extra Name of Expected Lead Pilot
 uint8_t sue_ID_DIY_DRONES_URL[70]; ///< Serial UDB Extra URL of Lead Pilot or Team
} mavlink_serial_udb_extra_f16_t;

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN 110
#define MAVLINK_MSG_ID_180_LEN 110

#define MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_CRC 222
#define MAVLINK_MSG_ID_180_CRC 222

#define MAVLINK_MSG_SERIAL_UDB_EXTRA_F16_FIELD_SUE_ID_LEAD_PILOT_LEN 40
#define MAVLINK_MSG_SERIAL_UDB_EXTRA_F16_FIELD_SUE_ID_DIY_DRONES_URL_LEN 70

#define MAVLINK_MESSAGE_INFO_SERIAL_UDB_EXTRA_F16 { \
	"SERIAL_UDB_EXTRA_F16", \
	2, \
	{  { "sue_ID_LEAD_PILOT", NULL, MAVLINK_TYPE_UINT8_T, 40, 0, offsetof(mavlink_serial_udb_extra_f16_t, sue_ID_LEAD_PILOT) }, \
         { "sue_ID_DIY_DRONES_URL", NULL, MAVLINK_TYPE_UINT8_T, 70, 40, offsetof(mavlink_serial_udb_extra_f16_t, sue_ID_DIY_DRONES_URL) }, \
         } \
}


/**
 * @brief Pack a serial_udb_extra_f16 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sue_ID_LEAD_PILOT Serial UDB Extra Name of Expected Lead Pilot
 * @param sue_ID_DIY_DRONES_URL Serial UDB Extra URL of Lead Pilot or Team
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f16_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const uint8_t *sue_ID_LEAD_PILOT, const uint8_t *sue_ID_DIY_DRONES_URL)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN];

	_mav_put_uint8_t_array(buf, 0, sue_ID_LEAD_PILOT, 40);
	_mav_put_uint8_t_array(buf, 40, sue_ID_DIY_DRONES_URL, 70);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN);
#else
	mavlink_serial_udb_extra_f16_t packet;

	mav_array_memcpy(packet.sue_ID_LEAD_PILOT, sue_ID_LEAD_PILOT, sizeof(uint8_t)*40);
	mav_array_memcpy(packet.sue_ID_DIY_DRONES_URL, sue_ID_DIY_DRONES_URL, sizeof(uint8_t)*70);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN);
#endif
}

/**
 * @brief Pack a serial_udb_extra_f16 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param sue_ID_LEAD_PILOT Serial UDB Extra Name of Expected Lead Pilot
 * @param sue_ID_DIY_DRONES_URL Serial UDB Extra URL of Lead Pilot or Team
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f16_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const uint8_t *sue_ID_LEAD_PILOT,const uint8_t *sue_ID_DIY_DRONES_URL)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN];

	_mav_put_uint8_t_array(buf, 0, sue_ID_LEAD_PILOT, 40);
	_mav_put_uint8_t_array(buf, 40, sue_ID_DIY_DRONES_URL, 70);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN);
#else
	mavlink_serial_udb_extra_f16_t packet;

	mav_array_memcpy(packet.sue_ID_LEAD_PILOT, sue_ID_LEAD_PILOT, sizeof(uint8_t)*40);
	mav_array_memcpy(packet.sue_ID_DIY_DRONES_URL, sue_ID_DIY_DRONES_URL, sizeof(uint8_t)*70);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN);
#endif
}

/**
 * @brief Encode a serial_udb_extra_f16 struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param serial_udb_extra_f16 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f16_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_serial_udb_extra_f16_t* serial_udb_extra_f16)
{
	return mavlink_msg_serial_udb_extra_f16_pack(system_id, component_id, msg, serial_udb_extra_f16->sue_ID_LEAD_PILOT, serial_udb_extra_f16->sue_ID_DIY_DRONES_URL);
}

/**
 * @brief Send a serial_udb_extra_f16 message
 * @param chan MAVLink channel to send the message
 *
 * @param sue_ID_LEAD_PILOT Serial UDB Extra Name of Expected Lead Pilot
 * @param sue_ID_DIY_DRONES_URL Serial UDB Extra URL of Lead Pilot or Team
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_serial_udb_extra_f16_send(mavlink_channel_t chan, const uint8_t *sue_ID_LEAD_PILOT, const uint8_t *sue_ID_DIY_DRONES_URL)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN];

	_mav_put_uint8_t_array(buf, 0, sue_ID_LEAD_PILOT, 40);
	_mav_put_uint8_t_array(buf, 40, sue_ID_DIY_DRONES_URL, 70);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16, buf, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN);
#endif
#else
	mavlink_serial_udb_extra_f16_t packet;

	mav_array_memcpy(packet.sue_ID_LEAD_PILOT, sue_ID_LEAD_PILOT, sizeof(uint8_t)*40);
	mav_array_memcpy(packet.sue_ID_DIY_DRONES_URL, sue_ID_DIY_DRONES_URL, sizeof(uint8_t)*70);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16, (const char *)&packet, MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN);
#endif
#endif
}

#endif

// MESSAGE SERIAL_UDB_EXTRA_F16 UNPACKING


/**
 * @brief Get field sue_ID_LEAD_PILOT from serial_udb_extra_f16 message
 *
 * @return Serial UDB Extra Name of Expected Lead Pilot
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f16_get_sue_ID_LEAD_PILOT(const mavlink_message_t* msg, uint8_t *sue_ID_LEAD_PILOT)
{
	return _MAV_RETURN_uint8_t_array(msg, sue_ID_LEAD_PILOT, 40,  0);
}

/**
 * @brief Get field sue_ID_DIY_DRONES_URL from serial_udb_extra_f16 message
 *
 * @return Serial UDB Extra URL of Lead Pilot or Team
 */
static inline uint16_t mavlink_msg_serial_udb_extra_f16_get_sue_ID_DIY_DRONES_URL(const mavlink_message_t* msg, uint8_t *sue_ID_DIY_DRONES_URL)
{
	return _MAV_RETURN_uint8_t_array(msg, sue_ID_DIY_DRONES_URL, 70,  40);
}

/**
 * @brief Decode a serial_udb_extra_f16 message into a struct
 *
 * @param msg The message to decode
 * @param serial_udb_extra_f16 C-struct to decode the message contents into
 */
static inline void mavlink_msg_serial_udb_extra_f16_decode(const mavlink_message_t* msg, mavlink_serial_udb_extra_f16_t* serial_udb_extra_f16)
{
#if MAVLINK_NEED_BYTE_SWAP
	mavlink_msg_serial_udb_extra_f16_get_sue_ID_LEAD_PILOT(msg, serial_udb_extra_f16->sue_ID_LEAD_PILOT);
	mavlink_msg_serial_udb_extra_f16_get_sue_ID_DIY_DRONES_URL(msg, serial_udb_extra_f16->sue_ID_DIY_DRONES_URL);
#else
	memcpy(serial_udb_extra_f16, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SERIAL_UDB_EXTRA_F16_LEN);
#endif
}
