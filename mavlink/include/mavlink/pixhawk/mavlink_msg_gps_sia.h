// MESSAGE GPS_SIA PACKING

#define MAVLINK_MSG_ID_GPS_SIA 211

MAVPACKED(
typedef struct __mavlink_gps_sia_t {
 uint64_t timestamp; /*< Timestamp*/
 uint32_t gps_jingdu; /*< jingdu.*/
 uint32_t gps_weidu; /*< weidu.*/
 float gps_haiba; /*< haiba.*/
 float gps_bei; /*< beixiangsudu.*/
 float gps_dong; /*< dongxiangsudu.*/
 float gps_di; /*< dixiangsudu.*/
 uint8_t gps_weixing; /*< weixinggeshu.*/
 uint8_t gps_dingwei; /*< dingweimoshi.*/
}) mavlink_gps_sia_t;

#define MAVLINK_MSG_ID_GPS_SIA_LEN 34
#define MAVLINK_MSG_ID_GPS_SIA_MIN_LEN 34
#define MAVLINK_MSG_ID_211_LEN 34
#define MAVLINK_MSG_ID_211_MIN_LEN 34

#define MAVLINK_MSG_ID_GPS_SIA_CRC 197
#define MAVLINK_MSG_ID_211_CRC 197



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_GPS_SIA { \
	211, \
	"GPS_SIA", \
	9, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_sia_t, timestamp) }, \
         { "gps_jingdu", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gps_sia_t, gps_jingdu) }, \
         { "gps_weidu", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_gps_sia_t, gps_weidu) }, \
         { "gps_haiba", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gps_sia_t, gps_haiba) }, \
         { "gps_bei", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gps_sia_t, gps_bei) }, \
         { "gps_dong", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gps_sia_t, gps_dong) }, \
         { "gps_di", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gps_sia_t, gps_di) }, \
         { "gps_weixing", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_gps_sia_t, gps_weixing) }, \
         { "gps_dingwei", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_gps_sia_t, gps_dingwei) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_GPS_SIA { \
	"GPS_SIA", \
	9, \
	{  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_gps_sia_t, timestamp) }, \
         { "gps_jingdu", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_gps_sia_t, gps_jingdu) }, \
         { "gps_weidu", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_gps_sia_t, gps_weidu) }, \
         { "gps_haiba", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_gps_sia_t, gps_haiba) }, \
         { "gps_bei", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_gps_sia_t, gps_bei) }, \
         { "gps_dong", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_gps_sia_t, gps_dong) }, \
         { "gps_di", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_gps_sia_t, gps_di) }, \
         { "gps_weixing", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_gps_sia_t, gps_weixing) }, \
         { "gps_dingwei", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_gps_sia_t, gps_dingwei) }, \
         } \
}
#endif

/**
 * @brief Pack a gps_sia message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp Timestamp
 * @param gps_jingdu jingdu.
 * @param gps_weidu weidu.
 * @param gps_haiba haiba.
 * @param gps_bei beixiangsudu.
 * @param gps_dong dongxiangsudu.
 * @param gps_di dixiangsudu.
 * @param gps_weixing weixinggeshu.
 * @param gps_dingwei dingweimoshi.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_sia_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint64_t timestamp, uint32_t gps_jingdu, uint32_t gps_weidu, float gps_haiba, float gps_bei, float gps_dong, float gps_di, uint8_t gps_weixing, uint8_t gps_dingwei)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_SIA_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint32_t(buf, 8, gps_jingdu);
	_mav_put_uint32_t(buf, 12, gps_weidu);
	_mav_put_float(buf, 16, gps_haiba);
	_mav_put_float(buf, 20, gps_bei);
	_mav_put_float(buf, 24, gps_dong);
	_mav_put_float(buf, 28, gps_di);
	_mav_put_uint8_t(buf, 32, gps_weixing);
	_mav_put_uint8_t(buf, 33, gps_dingwei);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_SIA_LEN);
#else
	mavlink_gps_sia_t packet;
	packet.timestamp = timestamp;
	packet.gps_jingdu = gps_jingdu;
	packet.gps_weidu = gps_weidu;
	packet.gps_haiba = gps_haiba;
	packet.gps_bei = gps_bei;
	packet.gps_dong = gps_dong;
	packet.gps_di = gps_di;
	packet.gps_weixing = gps_weixing;
	packet.gps_dingwei = gps_dingwei;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_SIA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_SIA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS_SIA_MIN_LEN, MAVLINK_MSG_ID_GPS_SIA_LEN, MAVLINK_MSG_ID_GPS_SIA_CRC);
}

/**
 * @brief Pack a gps_sia message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp Timestamp
 * @param gps_jingdu jingdu.
 * @param gps_weidu weidu.
 * @param gps_haiba haiba.
 * @param gps_bei beixiangsudu.
 * @param gps_dong dongxiangsudu.
 * @param gps_di dixiangsudu.
 * @param gps_weixing weixinggeshu.
 * @param gps_dingwei dingweimoshi.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps_sia_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint64_t timestamp,uint32_t gps_jingdu,uint32_t gps_weidu,float gps_haiba,float gps_bei,float gps_dong,float gps_di,uint8_t gps_weixing,uint8_t gps_dingwei)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_SIA_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint32_t(buf, 8, gps_jingdu);
	_mav_put_uint32_t(buf, 12, gps_weidu);
	_mav_put_float(buf, 16, gps_haiba);
	_mav_put_float(buf, 20, gps_bei);
	_mav_put_float(buf, 24, gps_dong);
	_mav_put_float(buf, 28, gps_di);
	_mav_put_uint8_t(buf, 32, gps_weixing);
	_mav_put_uint8_t(buf, 33, gps_dingwei);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS_SIA_LEN);
#else
	mavlink_gps_sia_t packet;
	packet.timestamp = timestamp;
	packet.gps_jingdu = gps_jingdu;
	packet.gps_weidu = gps_weidu;
	packet.gps_haiba = gps_haiba;
	packet.gps_bei = gps_bei;
	packet.gps_dong = gps_dong;
	packet.gps_di = gps_di;
	packet.gps_weixing = gps_weixing;
	packet.gps_dingwei = gps_dingwei;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS_SIA_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS_SIA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS_SIA_MIN_LEN, MAVLINK_MSG_ID_GPS_SIA_LEN, MAVLINK_MSG_ID_GPS_SIA_CRC);
}

/**
 * @brief Encode a gps_sia struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps_sia C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_sia_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps_sia_t* gps_sia)
{
	return mavlink_msg_gps_sia_pack(system_id, component_id, msg, gps_sia->timestamp, gps_sia->gps_jingdu, gps_sia->gps_weidu, gps_sia->gps_haiba, gps_sia->gps_bei, gps_sia->gps_dong, gps_sia->gps_di, gps_sia->gps_weixing, gps_sia->gps_dingwei);
}

/**
 * @brief Encode a gps_sia struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps_sia C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps_sia_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps_sia_t* gps_sia)
{
	return mavlink_msg_gps_sia_pack_chan(system_id, component_id, chan, msg, gps_sia->timestamp, gps_sia->gps_jingdu, gps_sia->gps_weidu, gps_sia->gps_haiba, gps_sia->gps_bei, gps_sia->gps_dong, gps_sia->gps_di, gps_sia->gps_weixing, gps_sia->gps_dingwei);
}

/**
 * @brief Send a gps_sia message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp Timestamp
 * @param gps_jingdu jingdu.
 * @param gps_weidu weidu.
 * @param gps_haiba haiba.
 * @param gps_bei beixiangsudu.
 * @param gps_dong dongxiangsudu.
 * @param gps_di dixiangsudu.
 * @param gps_weixing weixinggeshu.
 * @param gps_dingwei dingweimoshi.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps_sia_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t gps_jingdu, uint32_t gps_weidu, float gps_haiba, float gps_bei, float gps_dong, float gps_di, uint8_t gps_weixing, uint8_t gps_dingwei)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS_SIA_LEN];
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint32_t(buf, 8, gps_jingdu);
	_mav_put_uint32_t(buf, 12, gps_weidu);
	_mav_put_float(buf, 16, gps_haiba);
	_mav_put_float(buf, 20, gps_bei);
	_mav_put_float(buf, 24, gps_dong);
	_mav_put_float(buf, 28, gps_di);
	_mav_put_uint8_t(buf, 32, gps_weixing);
	_mav_put_uint8_t(buf, 33, gps_dingwei);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_SIA, buf, MAVLINK_MSG_ID_GPS_SIA_MIN_LEN, MAVLINK_MSG_ID_GPS_SIA_LEN, MAVLINK_MSG_ID_GPS_SIA_CRC);
#else
	mavlink_gps_sia_t packet;
	packet.timestamp = timestamp;
	packet.gps_jingdu = gps_jingdu;
	packet.gps_weidu = gps_weidu;
	packet.gps_haiba = gps_haiba;
	packet.gps_bei = gps_bei;
	packet.gps_dong = gps_dong;
	packet.gps_di = gps_di;
	packet.gps_weixing = gps_weixing;
	packet.gps_dingwei = gps_dingwei;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_SIA, (const char *)&packet, MAVLINK_MSG_ID_GPS_SIA_MIN_LEN, MAVLINK_MSG_ID_GPS_SIA_LEN, MAVLINK_MSG_ID_GPS_SIA_CRC);
#endif
}

/**
 * @brief Send a gps_sia message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_gps_sia_send_struct(mavlink_channel_t chan, const mavlink_gps_sia_t* gps_sia)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_gps_sia_send(chan, gps_sia->timestamp, gps_sia->gps_jingdu, gps_sia->gps_weidu, gps_sia->gps_haiba, gps_sia->gps_bei, gps_sia->gps_dong, gps_sia->gps_di, gps_sia->gps_weixing, gps_sia->gps_dingwei);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_SIA, (const char *)gps_sia, MAVLINK_MSG_ID_GPS_SIA_MIN_LEN, MAVLINK_MSG_ID_GPS_SIA_LEN, MAVLINK_MSG_ID_GPS_SIA_CRC);
#endif
}

#if MAVLINK_MSG_ID_GPS_SIA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_gps_sia_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t gps_jingdu, uint32_t gps_weidu, float gps_haiba, float gps_bei, float gps_dong, float gps_di, uint8_t gps_weixing, uint8_t gps_dingwei)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, timestamp);
	_mav_put_uint32_t(buf, 8, gps_jingdu);
	_mav_put_uint32_t(buf, 12, gps_weidu);
	_mav_put_float(buf, 16, gps_haiba);
	_mav_put_float(buf, 20, gps_bei);
	_mav_put_float(buf, 24, gps_dong);
	_mav_put_float(buf, 28, gps_di);
	_mav_put_uint8_t(buf, 32, gps_weixing);
	_mav_put_uint8_t(buf, 33, gps_dingwei);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_SIA, buf, MAVLINK_MSG_ID_GPS_SIA_MIN_LEN, MAVLINK_MSG_ID_GPS_SIA_LEN, MAVLINK_MSG_ID_GPS_SIA_CRC);
#else
	mavlink_gps_sia_t *packet = (mavlink_gps_sia_t *)msgbuf;
	packet->timestamp = timestamp;
	packet->gps_jingdu = gps_jingdu;
	packet->gps_weidu = gps_weidu;
	packet->gps_haiba = gps_haiba;
	packet->gps_bei = gps_bei;
	packet->gps_dong = gps_dong;
	packet->gps_di = gps_di;
	packet->gps_weixing = gps_weixing;
	packet->gps_dingwei = gps_dingwei;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS_SIA, (const char *)packet, MAVLINK_MSG_ID_GPS_SIA_MIN_LEN, MAVLINK_MSG_ID_GPS_SIA_LEN, MAVLINK_MSG_ID_GPS_SIA_CRC);
#endif
}
#endif

#endif

// MESSAGE GPS_SIA UNPACKING


/**
 * @brief Get field timestamp from gps_sia message
 *
 * @return Timestamp
 */
static inline uint64_t mavlink_msg_gps_sia_get_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field gps_jingdu from gps_sia message
 *
 * @return jingdu.
 */
static inline uint32_t mavlink_msg_gps_sia_get_gps_jingdu(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field gps_weidu from gps_sia message
 *
 * @return weidu.
 */
static inline uint32_t mavlink_msg_gps_sia_get_gps_weidu(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field gps_haiba from gps_sia message
 *
 * @return haiba.
 */
static inline float mavlink_msg_gps_sia_get_gps_haiba(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gps_bei from gps_sia message
 *
 * @return beixiangsudu.
 */
static inline float mavlink_msg_gps_sia_get_gps_bei(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gps_dong from gps_sia message
 *
 * @return dongxiangsudu.
 */
static inline float mavlink_msg_gps_sia_get_gps_dong(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field gps_di from gps_sia message
 *
 * @return dixiangsudu.
 */
static inline float mavlink_msg_gps_sia_get_gps_di(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field gps_weixing from gps_sia message
 *
 * @return weixinggeshu.
 */
static inline uint8_t mavlink_msg_gps_sia_get_gps_weixing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field gps_dingwei from gps_sia message
 *
 * @return dingweimoshi.
 */
static inline uint8_t mavlink_msg_gps_sia_get_gps_dingwei(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Decode a gps_sia message into a struct
 *
 * @param msg The message to decode
 * @param gps_sia C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps_sia_decode(const mavlink_message_t* msg, mavlink_gps_sia_t* gps_sia)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	gps_sia->timestamp = mavlink_msg_gps_sia_get_timestamp(msg);
	gps_sia->gps_jingdu = mavlink_msg_gps_sia_get_gps_jingdu(msg);
	gps_sia->gps_weidu = mavlink_msg_gps_sia_get_gps_weidu(msg);
	gps_sia->gps_haiba = mavlink_msg_gps_sia_get_gps_haiba(msg);
	gps_sia->gps_bei = mavlink_msg_gps_sia_get_gps_bei(msg);
	gps_sia->gps_dong = mavlink_msg_gps_sia_get_gps_dong(msg);
	gps_sia->gps_di = mavlink_msg_gps_sia_get_gps_di(msg);
	gps_sia->gps_weixing = mavlink_msg_gps_sia_get_gps_weixing(msg);
	gps_sia->gps_dingwei = mavlink_msg_gps_sia_get_gps_dingwei(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_GPS_SIA_LEN? msg->len : MAVLINK_MSG_ID_GPS_SIA_LEN;
        memset(gps_sia, 0, MAVLINK_MSG_ID_GPS_SIA_LEN);
	memcpy(gps_sia, _MAV_PAYLOAD(msg), len);
#endif
}
