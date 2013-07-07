// MESSAGE FLEXIFUNCTION_READ_REQ PACKING

#define MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ 151

typedef struct __mavlink_flexifunction_read_req_t
{
 int16_t read_req_type; ///< Type of flexifunction data requested
 int16_t data_index; ///< index into data where needed
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
} mavlink_flexifunction_read_req_t;

#define MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN 6
#define MAVLINK_MSG_ID_151_LEN 6

#define MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_CRC 26
#define MAVLINK_MSG_ID_151_CRC 26



#define MAVLINK_MESSAGE_INFO_FLEXIFUNCTION_READ_REQ { \
	"FLEXIFUNCTION_READ_REQ", \
	4, \
	{  { "read_req_type", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_flexifunction_read_req_t, read_req_type) }, \
         { "data_index", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_flexifunction_read_req_t, data_index) }, \
         { "target_system", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_flexifunction_read_req_t, target_system) }, \
         { "target_component", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_flexifunction_read_req_t, target_component) }, \
         } \
}


/**
 * @brief Pack a flexifunction_read_req message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param read_req_type Type of flexifunction data requested
 * @param data_index index into data where needed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_read_req_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t target_system, uint8_t target_component, int16_t read_req_type, int16_t data_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN];
	_mav_put_int16_t(buf, 0, read_req_type);
	_mav_put_int16_t(buf, 2, data_index);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN);
#else
	mavlink_flexifunction_read_req_t packet;
	packet.read_req_type = read_req_type;
	packet.data_index = data_index;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN);
#endif
}

/**
 * @brief Pack a flexifunction_read_req message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param target_system System ID
 * @param target_component Component ID
 * @param read_req_type Type of flexifunction data requested
 * @param data_index index into data where needed
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_flexifunction_read_req_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t target_system,uint8_t target_component,int16_t read_req_type,int16_t data_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN];
	_mav_put_int16_t(buf, 0, read_req_type);
	_mav_put_int16_t(buf, 2, data_index);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN);
#else
	mavlink_flexifunction_read_req_t packet;
	packet.read_req_type = read_req_type;
	packet.data_index = data_index;
	packet.target_system = target_system;
	packet.target_component = target_component;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN);
#endif
}

/**
 * @brief Encode a flexifunction_read_req struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param flexifunction_read_req C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_flexifunction_read_req_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_flexifunction_read_req_t* flexifunction_read_req)
{
	return mavlink_msg_flexifunction_read_req_pack(system_id, component_id, msg, flexifunction_read_req->target_system, flexifunction_read_req->target_component, flexifunction_read_req->read_req_type, flexifunction_read_req->data_index);
}

/**
 * @brief Send a flexifunction_read_req message
 * @param chan MAVLink channel to send the message
 *
 * @param target_system System ID
 * @param target_component Component ID
 * @param read_req_type Type of flexifunction data requested
 * @param data_index index into data where needed
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_flexifunction_read_req_send(mavlink_channel_t chan, uint8_t target_system, uint8_t target_component, int16_t read_req_type, int16_t data_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN];
	_mav_put_int16_t(buf, 0, read_req_type);
	_mav_put_int16_t(buf, 2, data_index);
	_mav_put_uint8_t(buf, 4, target_system);
	_mav_put_uint8_t(buf, 5, target_component);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ, buf, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN);
#endif
#else
	mavlink_flexifunction_read_req_t packet;
	packet.read_req_type = read_req_type;
	packet.data_index = data_index;
	packet.target_system = target_system;
	packet.target_component = target_component;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ, (const char *)&packet, MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN);
#endif
#endif
}

#endif

// MESSAGE FLEXIFUNCTION_READ_REQ UNPACKING


/**
 * @brief Get field target_system from flexifunction_read_req message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_flexifunction_read_req_get_target_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Get field target_component from flexifunction_read_req message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_flexifunction_read_req_get_target_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field read_req_type from flexifunction_read_req message
 *
 * @return Type of flexifunction data requested
 */
static inline int16_t mavlink_msg_flexifunction_read_req_get_read_req_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field data_index from flexifunction_read_req message
 *
 * @return index into data where needed
 */
static inline int16_t mavlink_msg_flexifunction_read_req_get_data_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a flexifunction_read_req message into a struct
 *
 * @param msg The message to decode
 * @param flexifunction_read_req C-struct to decode the message contents into
 */
static inline void mavlink_msg_flexifunction_read_req_decode(const mavlink_message_t* msg, mavlink_flexifunction_read_req_t* flexifunction_read_req)
{
#if MAVLINK_NEED_BYTE_SWAP
	flexifunction_read_req->read_req_type = mavlink_msg_flexifunction_read_req_get_read_req_type(msg);
	flexifunction_read_req->data_index = mavlink_msg_flexifunction_read_req_get_data_index(msg);
	flexifunction_read_req->target_system = mavlink_msg_flexifunction_read_req_get_target_system(msg);
	flexifunction_read_req->target_component = mavlink_msg_flexifunction_read_req_get_target_component(msg);
#else
	memcpy(flexifunction_read_req, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_FLEXIFUNCTION_READ_REQ_LEN);
#endif
}
