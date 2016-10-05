// MESSAGE MISSION_BUCKETS_POSITION PACKING

#define MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION 212

MAVPACKED(
typedef struct __mavlink_mission_buckets_position_t {
 double bucketA_lat_curr; /*< A -- latitude, in deg*/
 double bucketA_lon_curr; /*< longitude, in deg*/
 double bucketA_lat_next; /*< latitude, in deg*/
 double bucketA_lon_next; /*< longitude, in deg*/
 double bucketB_lat_curr; /*< B -- latitude, in deg*/
 double bucketB_lon_curr; /*< longitude, in deg*/
 double bucketB_lat_next; /*< latitude, in deg*/
 double bucketB_lon_next; /*< longitude, in deg*/
 double bucketC_lat_curr; /*< C -- latitude, in deg*/
 double bucketC_lon_curr; /*< longitude, in deg*/
 double bucketC_lat_next; /*< latitude, in deg*/
 double bucketC_lon_next; /*< longitude, in deg*/
 double bucketD_lat_curr; /*< D -- latitude, in deg*/
 double bucketD_lon_curr; /*< longitude, in deg*/
 double bucketD_lat_next; /*< latitude, in deg*/
 double bucketD_lon_next; /*< longitude, in deg*/
 double copter_lat_home; /*< home -- latitude, in deg*/
 double copter_lon_home; /*< longitude, in deg*/
 double copter_lat_guide; /*< guide -- latitude, in deg*/
 double copter_lon_guide; /*< longitude, in deg*/
 float bucketA_alt_curr; /*< altitude AMSL, in m*/
 float bucketA_alt_next; /*< altitude AMSL, in m*/
 float bucketB_alt_curr; /*< altitude AMSL, in m*/
 float bucketB_alt_next; /*< altitude AMSL, in m*/
 float bucketC_alt_curr; /*< altitude AMSL, in m*/
 float bucketC_alt_next; /*< altitude AMSL, in m*/
 float bucketD_alt_curr; /*< altitude AMSL, in m*/
 float bucketD_alt_next; /*< altitude AMSL, in m*/
 float copter_alt_home; /*< altitude AMSL, in m*/
 float copter_alt_guide; /*< altitude AMSL, in m*/
}) mavlink_mission_buckets_position_t;

#define MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN 200
#define MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_MIN_LEN 200
#define MAVLINK_MSG_ID_212_LEN 200
#define MAVLINK_MSG_ID_212_MIN_LEN 200

#define MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_CRC 239
#define MAVLINK_MSG_ID_212_CRC 239



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MISSION_BUCKETS_POSITION { \
	212, \
	"MISSION_BUCKETS_POSITION", \
	30, \
	{  { "bucketA_lat_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_mission_buckets_position_t, bucketA_lat_curr) }, \
         { "bucketA_lon_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_mission_buckets_position_t, bucketA_lon_curr) }, \
         { "bucketA_lat_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_mission_buckets_position_t, bucketA_lat_next) }, \
         { "bucketA_lon_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_mission_buckets_position_t, bucketA_lon_next) }, \
         { "bucketB_lat_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 32, offsetof(mavlink_mission_buckets_position_t, bucketB_lat_curr) }, \
         { "bucketB_lon_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 40, offsetof(mavlink_mission_buckets_position_t, bucketB_lon_curr) }, \
         { "bucketB_lat_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 48, offsetof(mavlink_mission_buckets_position_t, bucketB_lat_next) }, \
         { "bucketB_lon_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 56, offsetof(mavlink_mission_buckets_position_t, bucketB_lon_next) }, \
         { "bucketC_lat_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 64, offsetof(mavlink_mission_buckets_position_t, bucketC_lat_curr) }, \
         { "bucketC_lon_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 72, offsetof(mavlink_mission_buckets_position_t, bucketC_lon_curr) }, \
         { "bucketC_lat_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 80, offsetof(mavlink_mission_buckets_position_t, bucketC_lat_next) }, \
         { "bucketC_lon_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 88, offsetof(mavlink_mission_buckets_position_t, bucketC_lon_next) }, \
         { "bucketD_lat_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 96, offsetof(mavlink_mission_buckets_position_t, bucketD_lat_curr) }, \
         { "bucketD_lon_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 104, offsetof(mavlink_mission_buckets_position_t, bucketD_lon_curr) }, \
         { "bucketD_lat_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 112, offsetof(mavlink_mission_buckets_position_t, bucketD_lat_next) }, \
         { "bucketD_lon_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 120, offsetof(mavlink_mission_buckets_position_t, bucketD_lon_next) }, \
         { "copter_lat_home", NULL, MAVLINK_TYPE_DOUBLE, 0, 128, offsetof(mavlink_mission_buckets_position_t, copter_lat_home) }, \
         { "copter_lon_home", NULL, MAVLINK_TYPE_DOUBLE, 0, 136, offsetof(mavlink_mission_buckets_position_t, copter_lon_home) }, \
         { "copter_lat_guide", NULL, MAVLINK_TYPE_DOUBLE, 0, 144, offsetof(mavlink_mission_buckets_position_t, copter_lat_guide) }, \
         { "copter_lon_guide", NULL, MAVLINK_TYPE_DOUBLE, 0, 152, offsetof(mavlink_mission_buckets_position_t, copter_lon_guide) }, \
         { "bucketA_alt_curr", NULL, MAVLINK_TYPE_FLOAT, 0, 160, offsetof(mavlink_mission_buckets_position_t, bucketA_alt_curr) }, \
         { "bucketA_alt_next", NULL, MAVLINK_TYPE_FLOAT, 0, 164, offsetof(mavlink_mission_buckets_position_t, bucketA_alt_next) }, \
         { "bucketB_alt_curr", NULL, MAVLINK_TYPE_FLOAT, 0, 168, offsetof(mavlink_mission_buckets_position_t, bucketB_alt_curr) }, \
         { "bucketB_alt_next", NULL, MAVLINK_TYPE_FLOAT, 0, 172, offsetof(mavlink_mission_buckets_position_t, bucketB_alt_next) }, \
         { "bucketC_alt_curr", NULL, MAVLINK_TYPE_FLOAT, 0, 176, offsetof(mavlink_mission_buckets_position_t, bucketC_alt_curr) }, \
         { "bucketC_alt_next", NULL, MAVLINK_TYPE_FLOAT, 0, 180, offsetof(mavlink_mission_buckets_position_t, bucketC_alt_next) }, \
         { "bucketD_alt_curr", NULL, MAVLINK_TYPE_FLOAT, 0, 184, offsetof(mavlink_mission_buckets_position_t, bucketD_alt_curr) }, \
         { "bucketD_alt_next", NULL, MAVLINK_TYPE_FLOAT, 0, 188, offsetof(mavlink_mission_buckets_position_t, bucketD_alt_next) }, \
         { "copter_alt_home", NULL, MAVLINK_TYPE_FLOAT, 0, 192, offsetof(mavlink_mission_buckets_position_t, copter_alt_home) }, \
         { "copter_alt_guide", NULL, MAVLINK_TYPE_FLOAT, 0, 196, offsetof(mavlink_mission_buckets_position_t, copter_alt_guide) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MISSION_BUCKETS_POSITION { \
	"MISSION_BUCKETS_POSITION", \
	30, \
	{  { "bucketA_lat_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_mission_buckets_position_t, bucketA_lat_curr) }, \
         { "bucketA_lon_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_mission_buckets_position_t, bucketA_lon_curr) }, \
         { "bucketA_lat_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_mission_buckets_position_t, bucketA_lat_next) }, \
         { "bucketA_lon_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_mission_buckets_position_t, bucketA_lon_next) }, \
         { "bucketB_lat_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 32, offsetof(mavlink_mission_buckets_position_t, bucketB_lat_curr) }, \
         { "bucketB_lon_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 40, offsetof(mavlink_mission_buckets_position_t, bucketB_lon_curr) }, \
         { "bucketB_lat_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 48, offsetof(mavlink_mission_buckets_position_t, bucketB_lat_next) }, \
         { "bucketB_lon_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 56, offsetof(mavlink_mission_buckets_position_t, bucketB_lon_next) }, \
         { "bucketC_lat_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 64, offsetof(mavlink_mission_buckets_position_t, bucketC_lat_curr) }, \
         { "bucketC_lon_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 72, offsetof(mavlink_mission_buckets_position_t, bucketC_lon_curr) }, \
         { "bucketC_lat_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 80, offsetof(mavlink_mission_buckets_position_t, bucketC_lat_next) }, \
         { "bucketC_lon_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 88, offsetof(mavlink_mission_buckets_position_t, bucketC_lon_next) }, \
         { "bucketD_lat_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 96, offsetof(mavlink_mission_buckets_position_t, bucketD_lat_curr) }, \
         { "bucketD_lon_curr", NULL, MAVLINK_TYPE_DOUBLE, 0, 104, offsetof(mavlink_mission_buckets_position_t, bucketD_lon_curr) }, \
         { "bucketD_lat_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 112, offsetof(mavlink_mission_buckets_position_t, bucketD_lat_next) }, \
         { "bucketD_lon_next", NULL, MAVLINK_TYPE_DOUBLE, 0, 120, offsetof(mavlink_mission_buckets_position_t, bucketD_lon_next) }, \
         { "copter_lat_home", NULL, MAVLINK_TYPE_DOUBLE, 0, 128, offsetof(mavlink_mission_buckets_position_t, copter_lat_home) }, \
         { "copter_lon_home", NULL, MAVLINK_TYPE_DOUBLE, 0, 136, offsetof(mavlink_mission_buckets_position_t, copter_lon_home) }, \
         { "copter_lat_guide", NULL, MAVLINK_TYPE_DOUBLE, 0, 144, offsetof(mavlink_mission_buckets_position_t, copter_lat_guide) }, \
         { "copter_lon_guide", NULL, MAVLINK_TYPE_DOUBLE, 0, 152, offsetof(mavlink_mission_buckets_position_t, copter_lon_guide) }, \
         { "bucketA_alt_curr", NULL, MAVLINK_TYPE_FLOAT, 0, 160, offsetof(mavlink_mission_buckets_position_t, bucketA_alt_curr) }, \
         { "bucketA_alt_next", NULL, MAVLINK_TYPE_FLOAT, 0, 164, offsetof(mavlink_mission_buckets_position_t, bucketA_alt_next) }, \
         { "bucketB_alt_curr", NULL, MAVLINK_TYPE_FLOAT, 0, 168, offsetof(mavlink_mission_buckets_position_t, bucketB_alt_curr) }, \
         { "bucketB_alt_next", NULL, MAVLINK_TYPE_FLOAT, 0, 172, offsetof(mavlink_mission_buckets_position_t, bucketB_alt_next) }, \
         { "bucketC_alt_curr", NULL, MAVLINK_TYPE_FLOAT, 0, 176, offsetof(mavlink_mission_buckets_position_t, bucketC_alt_curr) }, \
         { "bucketC_alt_next", NULL, MAVLINK_TYPE_FLOAT, 0, 180, offsetof(mavlink_mission_buckets_position_t, bucketC_alt_next) }, \
         { "bucketD_alt_curr", NULL, MAVLINK_TYPE_FLOAT, 0, 184, offsetof(mavlink_mission_buckets_position_t, bucketD_alt_curr) }, \
         { "bucketD_alt_next", NULL, MAVLINK_TYPE_FLOAT, 0, 188, offsetof(mavlink_mission_buckets_position_t, bucketD_alt_next) }, \
         { "copter_alt_home", NULL, MAVLINK_TYPE_FLOAT, 0, 192, offsetof(mavlink_mission_buckets_position_t, copter_alt_home) }, \
         { "copter_alt_guide", NULL, MAVLINK_TYPE_FLOAT, 0, 196, offsetof(mavlink_mission_buckets_position_t, copter_alt_guide) }, \
         } \
}
#endif

/**
 * @brief Pack a mission_buckets_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param bucketA_lat_curr A -- latitude, in deg
 * @param bucketA_lon_curr longitude, in deg
 * @param bucketA_alt_curr altitude AMSL, in m
 * @param bucketA_lat_next latitude, in deg
 * @param bucketA_lon_next longitude, in deg
 * @param bucketA_alt_next altitude AMSL, in m
 * @param bucketB_lat_curr B -- latitude, in deg
 * @param bucketB_lon_curr longitude, in deg
 * @param bucketB_alt_curr altitude AMSL, in m
 * @param bucketB_lat_next latitude, in deg
 * @param bucketB_lon_next longitude, in deg
 * @param bucketB_alt_next altitude AMSL, in m
 * @param bucketC_lat_curr C -- latitude, in deg
 * @param bucketC_lon_curr longitude, in deg
 * @param bucketC_alt_curr altitude AMSL, in m
 * @param bucketC_lat_next latitude, in deg
 * @param bucketC_lon_next longitude, in deg
 * @param bucketC_alt_next altitude AMSL, in m
 * @param bucketD_lat_curr D -- latitude, in deg
 * @param bucketD_lon_curr longitude, in deg
 * @param bucketD_alt_curr altitude AMSL, in m
 * @param bucketD_lat_next latitude, in deg
 * @param bucketD_lon_next longitude, in deg
 * @param bucketD_alt_next altitude AMSL, in m
 * @param copter_lat_home home -- latitude, in deg
 * @param copter_lon_home longitude, in deg
 * @param copter_alt_home altitude AMSL, in m
 * @param copter_lat_guide guide -- latitude, in deg
 * @param copter_lon_guide longitude, in deg
 * @param copter_alt_guide altitude AMSL, in m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_buckets_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       double bucketA_lat_curr, double bucketA_lon_curr, float bucketA_alt_curr, double bucketA_lat_next, double bucketA_lon_next, float bucketA_alt_next, double bucketB_lat_curr, double bucketB_lon_curr, float bucketB_alt_curr, double bucketB_lat_next, double bucketB_lon_next, float bucketB_alt_next, double bucketC_lat_curr, double bucketC_lon_curr, float bucketC_alt_curr, double bucketC_lat_next, double bucketC_lon_next, float bucketC_alt_next, double bucketD_lat_curr, double bucketD_lon_curr, float bucketD_alt_curr, double bucketD_lat_next, double bucketD_lon_next, float bucketD_alt_next, double copter_lat_home, double copter_lon_home, float copter_alt_home, double copter_lat_guide, double copter_lon_guide, float copter_alt_guide)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN];
	_mav_put_double(buf, 0, bucketA_lat_curr);
	_mav_put_double(buf, 8, bucketA_lon_curr);
	_mav_put_double(buf, 16, bucketA_lat_next);
	_mav_put_double(buf, 24, bucketA_lon_next);
	_mav_put_double(buf, 32, bucketB_lat_curr);
	_mav_put_double(buf, 40, bucketB_lon_curr);
	_mav_put_double(buf, 48, bucketB_lat_next);
	_mav_put_double(buf, 56, bucketB_lon_next);
	_mav_put_double(buf, 64, bucketC_lat_curr);
	_mav_put_double(buf, 72, bucketC_lon_curr);
	_mav_put_double(buf, 80, bucketC_lat_next);
	_mav_put_double(buf, 88, bucketC_lon_next);
	_mav_put_double(buf, 96, bucketD_lat_curr);
	_mav_put_double(buf, 104, bucketD_lon_curr);
	_mav_put_double(buf, 112, bucketD_lat_next);
	_mav_put_double(buf, 120, bucketD_lon_next);
	_mav_put_double(buf, 128, copter_lat_home);
	_mav_put_double(buf, 136, copter_lon_home);
	_mav_put_double(buf, 144, copter_lat_guide);
	_mav_put_double(buf, 152, copter_lon_guide);
	_mav_put_float(buf, 160, bucketA_alt_curr);
	_mav_put_float(buf, 164, bucketA_alt_next);
	_mav_put_float(buf, 168, bucketB_alt_curr);
	_mav_put_float(buf, 172, bucketB_alt_next);
	_mav_put_float(buf, 176, bucketC_alt_curr);
	_mav_put_float(buf, 180, bucketC_alt_next);
	_mav_put_float(buf, 184, bucketD_alt_curr);
	_mav_put_float(buf, 188, bucketD_alt_next);
	_mav_put_float(buf, 192, copter_alt_home);
	_mav_put_float(buf, 196, copter_alt_guide);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN);
#else
	mavlink_mission_buckets_position_t packet;
	packet.bucketA_lat_curr = bucketA_lat_curr;
	packet.bucketA_lon_curr = bucketA_lon_curr;
	packet.bucketA_lat_next = bucketA_lat_next;
	packet.bucketA_lon_next = bucketA_lon_next;
	packet.bucketB_lat_curr = bucketB_lat_curr;
	packet.bucketB_lon_curr = bucketB_lon_curr;
	packet.bucketB_lat_next = bucketB_lat_next;
	packet.bucketB_lon_next = bucketB_lon_next;
	packet.bucketC_lat_curr = bucketC_lat_curr;
	packet.bucketC_lon_curr = bucketC_lon_curr;
	packet.bucketC_lat_next = bucketC_lat_next;
	packet.bucketC_lon_next = bucketC_lon_next;
	packet.bucketD_lat_curr = bucketD_lat_curr;
	packet.bucketD_lon_curr = bucketD_lon_curr;
	packet.bucketD_lat_next = bucketD_lat_next;
	packet.bucketD_lon_next = bucketD_lon_next;
	packet.copter_lat_home = copter_lat_home;
	packet.copter_lon_home = copter_lon_home;
	packet.copter_lat_guide = copter_lat_guide;
	packet.copter_lon_guide = copter_lon_guide;
	packet.bucketA_alt_curr = bucketA_alt_curr;
	packet.bucketA_alt_next = bucketA_alt_next;
	packet.bucketB_alt_curr = bucketB_alt_curr;
	packet.bucketB_alt_next = bucketB_alt_next;
	packet.bucketC_alt_curr = bucketC_alt_curr;
	packet.bucketC_alt_next = bucketC_alt_next;
	packet.bucketD_alt_curr = bucketD_alt_curr;
	packet.bucketD_alt_next = bucketD_alt_next;
	packet.copter_alt_home = copter_alt_home;
	packet.copter_alt_guide = copter_alt_guide;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_MIN_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_CRC);
}

/**
 * @brief Pack a mission_buckets_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bucketA_lat_curr A -- latitude, in deg
 * @param bucketA_lon_curr longitude, in deg
 * @param bucketA_alt_curr altitude AMSL, in m
 * @param bucketA_lat_next latitude, in deg
 * @param bucketA_lon_next longitude, in deg
 * @param bucketA_alt_next altitude AMSL, in m
 * @param bucketB_lat_curr B -- latitude, in deg
 * @param bucketB_lon_curr longitude, in deg
 * @param bucketB_alt_curr altitude AMSL, in m
 * @param bucketB_lat_next latitude, in deg
 * @param bucketB_lon_next longitude, in deg
 * @param bucketB_alt_next altitude AMSL, in m
 * @param bucketC_lat_curr C -- latitude, in deg
 * @param bucketC_lon_curr longitude, in deg
 * @param bucketC_alt_curr altitude AMSL, in m
 * @param bucketC_lat_next latitude, in deg
 * @param bucketC_lon_next longitude, in deg
 * @param bucketC_alt_next altitude AMSL, in m
 * @param bucketD_lat_curr D -- latitude, in deg
 * @param bucketD_lon_curr longitude, in deg
 * @param bucketD_alt_curr altitude AMSL, in m
 * @param bucketD_lat_next latitude, in deg
 * @param bucketD_lon_next longitude, in deg
 * @param bucketD_alt_next altitude AMSL, in m
 * @param copter_lat_home home -- latitude, in deg
 * @param copter_lon_home longitude, in deg
 * @param copter_alt_home altitude AMSL, in m
 * @param copter_lat_guide guide -- latitude, in deg
 * @param copter_lon_guide longitude, in deg
 * @param copter_alt_guide altitude AMSL, in m
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_mission_buckets_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           double bucketA_lat_curr,double bucketA_lon_curr,float bucketA_alt_curr,double bucketA_lat_next,double bucketA_lon_next,float bucketA_alt_next,double bucketB_lat_curr,double bucketB_lon_curr,float bucketB_alt_curr,double bucketB_lat_next,double bucketB_lon_next,float bucketB_alt_next,double bucketC_lat_curr,double bucketC_lon_curr,float bucketC_alt_curr,double bucketC_lat_next,double bucketC_lon_next,float bucketC_alt_next,double bucketD_lat_curr,double bucketD_lon_curr,float bucketD_alt_curr,double bucketD_lat_next,double bucketD_lon_next,float bucketD_alt_next,double copter_lat_home,double copter_lon_home,float copter_alt_home,double copter_lat_guide,double copter_lon_guide,float copter_alt_guide)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN];
	_mav_put_double(buf, 0, bucketA_lat_curr);
	_mav_put_double(buf, 8, bucketA_lon_curr);
	_mav_put_double(buf, 16, bucketA_lat_next);
	_mav_put_double(buf, 24, bucketA_lon_next);
	_mav_put_double(buf, 32, bucketB_lat_curr);
	_mav_put_double(buf, 40, bucketB_lon_curr);
	_mav_put_double(buf, 48, bucketB_lat_next);
	_mav_put_double(buf, 56, bucketB_lon_next);
	_mav_put_double(buf, 64, bucketC_lat_curr);
	_mav_put_double(buf, 72, bucketC_lon_curr);
	_mav_put_double(buf, 80, bucketC_lat_next);
	_mav_put_double(buf, 88, bucketC_lon_next);
	_mav_put_double(buf, 96, bucketD_lat_curr);
	_mav_put_double(buf, 104, bucketD_lon_curr);
	_mav_put_double(buf, 112, bucketD_lat_next);
	_mav_put_double(buf, 120, bucketD_lon_next);
	_mav_put_double(buf, 128, copter_lat_home);
	_mav_put_double(buf, 136, copter_lon_home);
	_mav_put_double(buf, 144, copter_lat_guide);
	_mav_put_double(buf, 152, copter_lon_guide);
	_mav_put_float(buf, 160, bucketA_alt_curr);
	_mav_put_float(buf, 164, bucketA_alt_next);
	_mav_put_float(buf, 168, bucketB_alt_curr);
	_mav_put_float(buf, 172, bucketB_alt_next);
	_mav_put_float(buf, 176, bucketC_alt_curr);
	_mav_put_float(buf, 180, bucketC_alt_next);
	_mav_put_float(buf, 184, bucketD_alt_curr);
	_mav_put_float(buf, 188, bucketD_alt_next);
	_mav_put_float(buf, 192, copter_alt_home);
	_mav_put_float(buf, 196, copter_alt_guide);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN);
#else
	mavlink_mission_buckets_position_t packet;
	packet.bucketA_lat_curr = bucketA_lat_curr;
	packet.bucketA_lon_curr = bucketA_lon_curr;
	packet.bucketA_lat_next = bucketA_lat_next;
	packet.bucketA_lon_next = bucketA_lon_next;
	packet.bucketB_lat_curr = bucketB_lat_curr;
	packet.bucketB_lon_curr = bucketB_lon_curr;
	packet.bucketB_lat_next = bucketB_lat_next;
	packet.bucketB_lon_next = bucketB_lon_next;
	packet.bucketC_lat_curr = bucketC_lat_curr;
	packet.bucketC_lon_curr = bucketC_lon_curr;
	packet.bucketC_lat_next = bucketC_lat_next;
	packet.bucketC_lon_next = bucketC_lon_next;
	packet.bucketD_lat_curr = bucketD_lat_curr;
	packet.bucketD_lon_curr = bucketD_lon_curr;
	packet.bucketD_lat_next = bucketD_lat_next;
	packet.bucketD_lon_next = bucketD_lon_next;
	packet.copter_lat_home = copter_lat_home;
	packet.copter_lon_home = copter_lon_home;
	packet.copter_lat_guide = copter_lat_guide;
	packet.copter_lon_guide = copter_lon_guide;
	packet.bucketA_alt_curr = bucketA_alt_curr;
	packet.bucketA_alt_next = bucketA_alt_next;
	packet.bucketB_alt_curr = bucketB_alt_curr;
	packet.bucketB_alt_next = bucketB_alt_next;
	packet.bucketC_alt_curr = bucketC_alt_curr;
	packet.bucketC_alt_next = bucketC_alt_next;
	packet.bucketD_alt_curr = bucketD_alt_curr;
	packet.bucketD_alt_next = bucketD_alt_next;
	packet.copter_alt_home = copter_alt_home;
	packet.copter_alt_guide = copter_alt_guide;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_MIN_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_CRC);
}

/**
 * @brief Encode a mission_buckets_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param mission_buckets_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_buckets_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_mission_buckets_position_t* mission_buckets_position)
{
	return mavlink_msg_mission_buckets_position_pack(system_id, component_id, msg, mission_buckets_position->bucketA_lat_curr, mission_buckets_position->bucketA_lon_curr, mission_buckets_position->bucketA_alt_curr, mission_buckets_position->bucketA_lat_next, mission_buckets_position->bucketA_lon_next, mission_buckets_position->bucketA_alt_next, mission_buckets_position->bucketB_lat_curr, mission_buckets_position->bucketB_lon_curr, mission_buckets_position->bucketB_alt_curr, mission_buckets_position->bucketB_lat_next, mission_buckets_position->bucketB_lon_next, mission_buckets_position->bucketB_alt_next, mission_buckets_position->bucketC_lat_curr, mission_buckets_position->bucketC_lon_curr, mission_buckets_position->bucketC_alt_curr, mission_buckets_position->bucketC_lat_next, mission_buckets_position->bucketC_lon_next, mission_buckets_position->bucketC_alt_next, mission_buckets_position->bucketD_lat_curr, mission_buckets_position->bucketD_lon_curr, mission_buckets_position->bucketD_alt_curr, mission_buckets_position->bucketD_lat_next, mission_buckets_position->bucketD_lon_next, mission_buckets_position->bucketD_alt_next, mission_buckets_position->copter_lat_home, mission_buckets_position->copter_lon_home, mission_buckets_position->copter_alt_home, mission_buckets_position->copter_lat_guide, mission_buckets_position->copter_lon_guide, mission_buckets_position->copter_alt_guide);
}

/**
 * @brief Encode a mission_buckets_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mission_buckets_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_mission_buckets_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_mission_buckets_position_t* mission_buckets_position)
{
	return mavlink_msg_mission_buckets_position_pack_chan(system_id, component_id, chan, msg, mission_buckets_position->bucketA_lat_curr, mission_buckets_position->bucketA_lon_curr, mission_buckets_position->bucketA_alt_curr, mission_buckets_position->bucketA_lat_next, mission_buckets_position->bucketA_lon_next, mission_buckets_position->bucketA_alt_next, mission_buckets_position->bucketB_lat_curr, mission_buckets_position->bucketB_lon_curr, mission_buckets_position->bucketB_alt_curr, mission_buckets_position->bucketB_lat_next, mission_buckets_position->bucketB_lon_next, mission_buckets_position->bucketB_alt_next, mission_buckets_position->bucketC_lat_curr, mission_buckets_position->bucketC_lon_curr, mission_buckets_position->bucketC_alt_curr, mission_buckets_position->bucketC_lat_next, mission_buckets_position->bucketC_lon_next, mission_buckets_position->bucketC_alt_next, mission_buckets_position->bucketD_lat_curr, mission_buckets_position->bucketD_lon_curr, mission_buckets_position->bucketD_alt_curr, mission_buckets_position->bucketD_lat_next, mission_buckets_position->bucketD_lon_next, mission_buckets_position->bucketD_alt_next, mission_buckets_position->copter_lat_home, mission_buckets_position->copter_lon_home, mission_buckets_position->copter_alt_home, mission_buckets_position->copter_lat_guide, mission_buckets_position->copter_lon_guide, mission_buckets_position->copter_alt_guide);
}

/**
 * @brief Send a mission_buckets_position message
 * @param chan MAVLink channel to send the message
 *
 * @param bucketA_lat_curr A -- latitude, in deg
 * @param bucketA_lon_curr longitude, in deg
 * @param bucketA_alt_curr altitude AMSL, in m
 * @param bucketA_lat_next latitude, in deg
 * @param bucketA_lon_next longitude, in deg
 * @param bucketA_alt_next altitude AMSL, in m
 * @param bucketB_lat_curr B -- latitude, in deg
 * @param bucketB_lon_curr longitude, in deg
 * @param bucketB_alt_curr altitude AMSL, in m
 * @param bucketB_lat_next latitude, in deg
 * @param bucketB_lon_next longitude, in deg
 * @param bucketB_alt_next altitude AMSL, in m
 * @param bucketC_lat_curr C -- latitude, in deg
 * @param bucketC_lon_curr longitude, in deg
 * @param bucketC_alt_curr altitude AMSL, in m
 * @param bucketC_lat_next latitude, in deg
 * @param bucketC_lon_next longitude, in deg
 * @param bucketC_alt_next altitude AMSL, in m
 * @param bucketD_lat_curr D -- latitude, in deg
 * @param bucketD_lon_curr longitude, in deg
 * @param bucketD_alt_curr altitude AMSL, in m
 * @param bucketD_lat_next latitude, in deg
 * @param bucketD_lon_next longitude, in deg
 * @param bucketD_alt_next altitude AMSL, in m
 * @param copter_lat_home home -- latitude, in deg
 * @param copter_lon_home longitude, in deg
 * @param copter_alt_home altitude AMSL, in m
 * @param copter_lat_guide guide -- latitude, in deg
 * @param copter_lon_guide longitude, in deg
 * @param copter_alt_guide altitude AMSL, in m
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_mission_buckets_position_send(mavlink_channel_t chan, double bucketA_lat_curr, double bucketA_lon_curr, float bucketA_alt_curr, double bucketA_lat_next, double bucketA_lon_next, float bucketA_alt_next, double bucketB_lat_curr, double bucketB_lon_curr, float bucketB_alt_curr, double bucketB_lat_next, double bucketB_lon_next, float bucketB_alt_next, double bucketC_lat_curr, double bucketC_lon_curr, float bucketC_alt_curr, double bucketC_lat_next, double bucketC_lon_next, float bucketC_alt_next, double bucketD_lat_curr, double bucketD_lon_curr, float bucketD_alt_curr, double bucketD_lat_next, double bucketD_lon_next, float bucketD_alt_next, double copter_lat_home, double copter_lon_home, float copter_alt_home, double copter_lat_guide, double copter_lon_guide, float copter_alt_guide)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN];
	_mav_put_double(buf, 0, bucketA_lat_curr);
	_mav_put_double(buf, 8, bucketA_lon_curr);
	_mav_put_double(buf, 16, bucketA_lat_next);
	_mav_put_double(buf, 24, bucketA_lon_next);
	_mav_put_double(buf, 32, bucketB_lat_curr);
	_mav_put_double(buf, 40, bucketB_lon_curr);
	_mav_put_double(buf, 48, bucketB_lat_next);
	_mav_put_double(buf, 56, bucketB_lon_next);
	_mav_put_double(buf, 64, bucketC_lat_curr);
	_mav_put_double(buf, 72, bucketC_lon_curr);
	_mav_put_double(buf, 80, bucketC_lat_next);
	_mav_put_double(buf, 88, bucketC_lon_next);
	_mav_put_double(buf, 96, bucketD_lat_curr);
	_mav_put_double(buf, 104, bucketD_lon_curr);
	_mav_put_double(buf, 112, bucketD_lat_next);
	_mav_put_double(buf, 120, bucketD_lon_next);
	_mav_put_double(buf, 128, copter_lat_home);
	_mav_put_double(buf, 136, copter_lon_home);
	_mav_put_double(buf, 144, copter_lat_guide);
	_mav_put_double(buf, 152, copter_lon_guide);
	_mav_put_float(buf, 160, bucketA_alt_curr);
	_mav_put_float(buf, 164, bucketA_alt_next);
	_mav_put_float(buf, 168, bucketB_alt_curr);
	_mav_put_float(buf, 172, bucketB_alt_next);
	_mav_put_float(buf, 176, bucketC_alt_curr);
	_mav_put_float(buf, 180, bucketC_alt_next);
	_mav_put_float(buf, 184, bucketD_alt_curr);
	_mav_put_float(buf, 188, bucketD_alt_next);
	_mav_put_float(buf, 192, copter_alt_home);
	_mav_put_float(buf, 196, copter_alt_guide);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION, buf, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_MIN_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_CRC);
#else
	mavlink_mission_buckets_position_t packet;
	packet.bucketA_lat_curr = bucketA_lat_curr;
	packet.bucketA_lon_curr = bucketA_lon_curr;
	packet.bucketA_lat_next = bucketA_lat_next;
	packet.bucketA_lon_next = bucketA_lon_next;
	packet.bucketB_lat_curr = bucketB_lat_curr;
	packet.bucketB_lon_curr = bucketB_lon_curr;
	packet.bucketB_lat_next = bucketB_lat_next;
	packet.bucketB_lon_next = bucketB_lon_next;
	packet.bucketC_lat_curr = bucketC_lat_curr;
	packet.bucketC_lon_curr = bucketC_lon_curr;
	packet.bucketC_lat_next = bucketC_lat_next;
	packet.bucketC_lon_next = bucketC_lon_next;
	packet.bucketD_lat_curr = bucketD_lat_curr;
	packet.bucketD_lon_curr = bucketD_lon_curr;
	packet.bucketD_lat_next = bucketD_lat_next;
	packet.bucketD_lon_next = bucketD_lon_next;
	packet.copter_lat_home = copter_lat_home;
	packet.copter_lon_home = copter_lon_home;
	packet.copter_lat_guide = copter_lat_guide;
	packet.copter_lon_guide = copter_lon_guide;
	packet.bucketA_alt_curr = bucketA_alt_curr;
	packet.bucketA_alt_next = bucketA_alt_next;
	packet.bucketB_alt_curr = bucketB_alt_curr;
	packet.bucketB_alt_next = bucketB_alt_next;
	packet.bucketC_alt_curr = bucketC_alt_curr;
	packet.bucketC_alt_next = bucketC_alt_next;
	packet.bucketD_alt_curr = bucketD_alt_curr;
	packet.bucketD_alt_next = bucketD_alt_next;
	packet.copter_alt_home = copter_alt_home;
	packet.copter_alt_guide = copter_alt_guide;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION, (const char *)&packet, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_MIN_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_CRC);
#endif
}

/**
 * @brief Send a mission_buckets_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_mission_buckets_position_send_struct(mavlink_channel_t chan, const mavlink_mission_buckets_position_t* mission_buckets_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_mission_buckets_position_send(chan, mission_buckets_position->bucketA_lat_curr, mission_buckets_position->bucketA_lon_curr, mission_buckets_position->bucketA_alt_curr, mission_buckets_position->bucketA_lat_next, mission_buckets_position->bucketA_lon_next, mission_buckets_position->bucketA_alt_next, mission_buckets_position->bucketB_lat_curr, mission_buckets_position->bucketB_lon_curr, mission_buckets_position->bucketB_alt_curr, mission_buckets_position->bucketB_lat_next, mission_buckets_position->bucketB_lon_next, mission_buckets_position->bucketB_alt_next, mission_buckets_position->bucketC_lat_curr, mission_buckets_position->bucketC_lon_curr, mission_buckets_position->bucketC_alt_curr, mission_buckets_position->bucketC_lat_next, mission_buckets_position->bucketC_lon_next, mission_buckets_position->bucketC_alt_next, mission_buckets_position->bucketD_lat_curr, mission_buckets_position->bucketD_lon_curr, mission_buckets_position->bucketD_alt_curr, mission_buckets_position->bucketD_lat_next, mission_buckets_position->bucketD_lon_next, mission_buckets_position->bucketD_alt_next, mission_buckets_position->copter_lat_home, mission_buckets_position->copter_lon_home, mission_buckets_position->copter_alt_home, mission_buckets_position->copter_lat_guide, mission_buckets_position->copter_lon_guide, mission_buckets_position->copter_alt_guide);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION, (const char *)mission_buckets_position, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_MIN_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_mission_buckets_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double bucketA_lat_curr, double bucketA_lon_curr, float bucketA_alt_curr, double bucketA_lat_next, double bucketA_lon_next, float bucketA_alt_next, double bucketB_lat_curr, double bucketB_lon_curr, float bucketB_alt_curr, double bucketB_lat_next, double bucketB_lon_next, float bucketB_alt_next, double bucketC_lat_curr, double bucketC_lon_curr, float bucketC_alt_curr, double bucketC_lat_next, double bucketC_lon_next, float bucketC_alt_next, double bucketD_lat_curr, double bucketD_lon_curr, float bucketD_alt_curr, double bucketD_lat_next, double bucketD_lon_next, float bucketD_alt_next, double copter_lat_home, double copter_lon_home, float copter_alt_home, double copter_lat_guide, double copter_lon_guide, float copter_alt_guide)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_double(buf, 0, bucketA_lat_curr);
	_mav_put_double(buf, 8, bucketA_lon_curr);
	_mav_put_double(buf, 16, bucketA_lat_next);
	_mav_put_double(buf, 24, bucketA_lon_next);
	_mav_put_double(buf, 32, bucketB_lat_curr);
	_mav_put_double(buf, 40, bucketB_lon_curr);
	_mav_put_double(buf, 48, bucketB_lat_next);
	_mav_put_double(buf, 56, bucketB_lon_next);
	_mav_put_double(buf, 64, bucketC_lat_curr);
	_mav_put_double(buf, 72, bucketC_lon_curr);
	_mav_put_double(buf, 80, bucketC_lat_next);
	_mav_put_double(buf, 88, bucketC_lon_next);
	_mav_put_double(buf, 96, bucketD_lat_curr);
	_mav_put_double(buf, 104, bucketD_lon_curr);
	_mav_put_double(buf, 112, bucketD_lat_next);
	_mav_put_double(buf, 120, bucketD_lon_next);
	_mav_put_double(buf, 128, copter_lat_home);
	_mav_put_double(buf, 136, copter_lon_home);
	_mav_put_double(buf, 144, copter_lat_guide);
	_mav_put_double(buf, 152, copter_lon_guide);
	_mav_put_float(buf, 160, bucketA_alt_curr);
	_mav_put_float(buf, 164, bucketA_alt_next);
	_mav_put_float(buf, 168, bucketB_alt_curr);
	_mav_put_float(buf, 172, bucketB_alt_next);
	_mav_put_float(buf, 176, bucketC_alt_curr);
	_mav_put_float(buf, 180, bucketC_alt_next);
	_mav_put_float(buf, 184, bucketD_alt_curr);
	_mav_put_float(buf, 188, bucketD_alt_next);
	_mav_put_float(buf, 192, copter_alt_home);
	_mav_put_float(buf, 196, copter_alt_guide);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION, buf, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_MIN_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_CRC);
#else
	mavlink_mission_buckets_position_t *packet = (mavlink_mission_buckets_position_t *)msgbuf;
	packet->bucketA_lat_curr = bucketA_lat_curr;
	packet->bucketA_lon_curr = bucketA_lon_curr;
	packet->bucketA_lat_next = bucketA_lat_next;
	packet->bucketA_lon_next = bucketA_lon_next;
	packet->bucketB_lat_curr = bucketB_lat_curr;
	packet->bucketB_lon_curr = bucketB_lon_curr;
	packet->bucketB_lat_next = bucketB_lat_next;
	packet->bucketB_lon_next = bucketB_lon_next;
	packet->bucketC_lat_curr = bucketC_lat_curr;
	packet->bucketC_lon_curr = bucketC_lon_curr;
	packet->bucketC_lat_next = bucketC_lat_next;
	packet->bucketC_lon_next = bucketC_lon_next;
	packet->bucketD_lat_curr = bucketD_lat_curr;
	packet->bucketD_lon_curr = bucketD_lon_curr;
	packet->bucketD_lat_next = bucketD_lat_next;
	packet->bucketD_lon_next = bucketD_lon_next;
	packet->copter_lat_home = copter_lat_home;
	packet->copter_lon_home = copter_lon_home;
	packet->copter_lat_guide = copter_lat_guide;
	packet->copter_lon_guide = copter_lon_guide;
	packet->bucketA_alt_curr = bucketA_alt_curr;
	packet->bucketA_alt_next = bucketA_alt_next;
	packet->bucketB_alt_curr = bucketB_alt_curr;
	packet->bucketB_alt_next = bucketB_alt_next;
	packet->bucketC_alt_curr = bucketC_alt_curr;
	packet->bucketC_alt_next = bucketC_alt_next;
	packet->bucketD_alt_curr = bucketD_alt_curr;
	packet->bucketD_alt_next = bucketD_alt_next;
	packet->copter_alt_home = copter_alt_home;
	packet->copter_alt_guide = copter_alt_guide;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION, (const char *)packet, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_MIN_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE MISSION_BUCKETS_POSITION UNPACKING


/**
 * @brief Get field bucketA_lat_curr from mission_buckets_position message
 *
 * @return A -- latitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketA_lat_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field bucketA_lon_curr from mission_buckets_position message
 *
 * @return longitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketA_lon_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field bucketA_alt_curr from mission_buckets_position message
 *
 * @return altitude AMSL, in m
 */
static inline float mavlink_msg_mission_buckets_position_get_bucketA_alt_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  160);
}

/**
 * @brief Get field bucketA_lat_next from mission_buckets_position message
 *
 * @return latitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketA_lat_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field bucketA_lon_next from mission_buckets_position message
 *
 * @return longitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketA_lon_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  24);
}

/**
 * @brief Get field bucketA_alt_next from mission_buckets_position message
 *
 * @return altitude AMSL, in m
 */
static inline float mavlink_msg_mission_buckets_position_get_bucketA_alt_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  164);
}

/**
 * @brief Get field bucketB_lat_curr from mission_buckets_position message
 *
 * @return B -- latitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketB_lat_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  32);
}

/**
 * @brief Get field bucketB_lon_curr from mission_buckets_position message
 *
 * @return longitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketB_lon_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  40);
}

/**
 * @brief Get field bucketB_alt_curr from mission_buckets_position message
 *
 * @return altitude AMSL, in m
 */
static inline float mavlink_msg_mission_buckets_position_get_bucketB_alt_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  168);
}

/**
 * @brief Get field bucketB_lat_next from mission_buckets_position message
 *
 * @return latitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketB_lat_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  48);
}

/**
 * @brief Get field bucketB_lon_next from mission_buckets_position message
 *
 * @return longitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketB_lon_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  56);
}

/**
 * @brief Get field bucketB_alt_next from mission_buckets_position message
 *
 * @return altitude AMSL, in m
 */
static inline float mavlink_msg_mission_buckets_position_get_bucketB_alt_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  172);
}

/**
 * @brief Get field bucketC_lat_curr from mission_buckets_position message
 *
 * @return C -- latitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketC_lat_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  64);
}

/**
 * @brief Get field bucketC_lon_curr from mission_buckets_position message
 *
 * @return longitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketC_lon_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  72);
}

/**
 * @brief Get field bucketC_alt_curr from mission_buckets_position message
 *
 * @return altitude AMSL, in m
 */
static inline float mavlink_msg_mission_buckets_position_get_bucketC_alt_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  176);
}

/**
 * @brief Get field bucketC_lat_next from mission_buckets_position message
 *
 * @return latitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketC_lat_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  80);
}

/**
 * @brief Get field bucketC_lon_next from mission_buckets_position message
 *
 * @return longitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketC_lon_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  88);
}

/**
 * @brief Get field bucketC_alt_next from mission_buckets_position message
 *
 * @return altitude AMSL, in m
 */
static inline float mavlink_msg_mission_buckets_position_get_bucketC_alt_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  180);
}

/**
 * @brief Get field bucketD_lat_curr from mission_buckets_position message
 *
 * @return D -- latitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketD_lat_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  96);
}

/**
 * @brief Get field bucketD_lon_curr from mission_buckets_position message
 *
 * @return longitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketD_lon_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  104);
}

/**
 * @brief Get field bucketD_alt_curr from mission_buckets_position message
 *
 * @return altitude AMSL, in m
 */
static inline float mavlink_msg_mission_buckets_position_get_bucketD_alt_curr(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  184);
}

/**
 * @brief Get field bucketD_lat_next from mission_buckets_position message
 *
 * @return latitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketD_lat_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  112);
}

/**
 * @brief Get field bucketD_lon_next from mission_buckets_position message
 *
 * @return longitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_bucketD_lon_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  120);
}

/**
 * @brief Get field bucketD_alt_next from mission_buckets_position message
 *
 * @return altitude AMSL, in m
 */
static inline float mavlink_msg_mission_buckets_position_get_bucketD_alt_next(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  188);
}

/**
 * @brief Get field copter_lat_home from mission_buckets_position message
 *
 * @return home -- latitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_copter_lat_home(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  128);
}

/**
 * @brief Get field copter_lon_home from mission_buckets_position message
 *
 * @return longitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_copter_lon_home(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  136);
}

/**
 * @brief Get field copter_alt_home from mission_buckets_position message
 *
 * @return altitude AMSL, in m
 */
static inline float mavlink_msg_mission_buckets_position_get_copter_alt_home(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  192);
}

/**
 * @brief Get field copter_lat_guide from mission_buckets_position message
 *
 * @return guide -- latitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_copter_lat_guide(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  144);
}

/**
 * @brief Get field copter_lon_guide from mission_buckets_position message
 *
 * @return longitude, in deg
 */
static inline double mavlink_msg_mission_buckets_position_get_copter_lon_guide(const mavlink_message_t* msg)
{
	return _MAV_RETURN_double(msg,  152);
}

/**
 * @brief Get field copter_alt_guide from mission_buckets_position message
 *
 * @return altitude AMSL, in m
 */
static inline float mavlink_msg_mission_buckets_position_get_copter_alt_guide(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  196);
}

/**
 * @brief Decode a mission_buckets_position message into a struct
 *
 * @param msg The message to decode
 * @param mission_buckets_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_mission_buckets_position_decode(const mavlink_message_t* msg, mavlink_mission_buckets_position_t* mission_buckets_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	mission_buckets_position->bucketA_lat_curr = mavlink_msg_mission_buckets_position_get_bucketA_lat_curr(msg);
	mission_buckets_position->bucketA_lon_curr = mavlink_msg_mission_buckets_position_get_bucketA_lon_curr(msg);
	mission_buckets_position->bucketA_lat_next = mavlink_msg_mission_buckets_position_get_bucketA_lat_next(msg);
	mission_buckets_position->bucketA_lon_next = mavlink_msg_mission_buckets_position_get_bucketA_lon_next(msg);
	mission_buckets_position->bucketB_lat_curr = mavlink_msg_mission_buckets_position_get_bucketB_lat_curr(msg);
	mission_buckets_position->bucketB_lon_curr = mavlink_msg_mission_buckets_position_get_bucketB_lon_curr(msg);
	mission_buckets_position->bucketB_lat_next = mavlink_msg_mission_buckets_position_get_bucketB_lat_next(msg);
	mission_buckets_position->bucketB_lon_next = mavlink_msg_mission_buckets_position_get_bucketB_lon_next(msg);
	mission_buckets_position->bucketC_lat_curr = mavlink_msg_mission_buckets_position_get_bucketC_lat_curr(msg);
	mission_buckets_position->bucketC_lon_curr = mavlink_msg_mission_buckets_position_get_bucketC_lon_curr(msg);
	mission_buckets_position->bucketC_lat_next = mavlink_msg_mission_buckets_position_get_bucketC_lat_next(msg);
	mission_buckets_position->bucketC_lon_next = mavlink_msg_mission_buckets_position_get_bucketC_lon_next(msg);
	mission_buckets_position->bucketD_lat_curr = mavlink_msg_mission_buckets_position_get_bucketD_lat_curr(msg);
	mission_buckets_position->bucketD_lon_curr = mavlink_msg_mission_buckets_position_get_bucketD_lon_curr(msg);
	mission_buckets_position->bucketD_lat_next = mavlink_msg_mission_buckets_position_get_bucketD_lat_next(msg);
	mission_buckets_position->bucketD_lon_next = mavlink_msg_mission_buckets_position_get_bucketD_lon_next(msg);
	mission_buckets_position->copter_lat_home = mavlink_msg_mission_buckets_position_get_copter_lat_home(msg);
	mission_buckets_position->copter_lon_home = mavlink_msg_mission_buckets_position_get_copter_lon_home(msg);
	mission_buckets_position->copter_lat_guide = mavlink_msg_mission_buckets_position_get_copter_lat_guide(msg);
	mission_buckets_position->copter_lon_guide = mavlink_msg_mission_buckets_position_get_copter_lon_guide(msg);
	mission_buckets_position->bucketA_alt_curr = mavlink_msg_mission_buckets_position_get_bucketA_alt_curr(msg);
	mission_buckets_position->bucketA_alt_next = mavlink_msg_mission_buckets_position_get_bucketA_alt_next(msg);
	mission_buckets_position->bucketB_alt_curr = mavlink_msg_mission_buckets_position_get_bucketB_alt_curr(msg);
	mission_buckets_position->bucketB_alt_next = mavlink_msg_mission_buckets_position_get_bucketB_alt_next(msg);
	mission_buckets_position->bucketC_alt_curr = mavlink_msg_mission_buckets_position_get_bucketC_alt_curr(msg);
	mission_buckets_position->bucketC_alt_next = mavlink_msg_mission_buckets_position_get_bucketC_alt_next(msg);
	mission_buckets_position->bucketD_alt_curr = mavlink_msg_mission_buckets_position_get_bucketD_alt_curr(msg);
	mission_buckets_position->bucketD_alt_next = mavlink_msg_mission_buckets_position_get_bucketD_alt_next(msg);
	mission_buckets_position->copter_alt_home = mavlink_msg_mission_buckets_position_get_copter_alt_home(msg);
	mission_buckets_position->copter_alt_guide = mavlink_msg_mission_buckets_position_get_copter_alt_guide(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN? msg->len : MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN;
        memset(mission_buckets_position, 0, MAVLINK_MSG_ID_MISSION_BUCKETS_POSITION_LEN);
	memcpy(mission_buckets_position, _MAV_PAYLOAD(msg), len);
#endif
}
