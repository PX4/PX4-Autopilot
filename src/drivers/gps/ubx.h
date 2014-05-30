/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ubx.h
 *
 * U-Blox protocol definition. Following u-blox 6/7 Receiver Description
 * including Prototol Specification.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 */

#ifndef UBX_H_
#define UBX_H_

#include "gps_helper.h"

#define UBX_SYNC1 0xB5
#define UBX_SYNC2 0x62

/* ClassIDs (the ones that are used) */
#define UBX_CLASS_NAV 0x01
//#define UBX_CLASS_RXM 0x02
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_CLASS_MON 0x0A

/* MessageIDs (the ones that are used) */
#define UBX_MESSAGE_NAV_POSLLH 0x02
//#define UBX_MESSAGE_NAV_DOP 0x04
#define UBX_MESSAGE_NAV_SOL 0x06
#define UBX_MESSAGE_NAV_VELNED 0x12
//#define UBX_MESSAGE_RXM_SVSI 0x20
#define UBX_MESSAGE_NAV_TIMEUTC 0x21
#define UBX_MESSAGE_NAV_SVINFO 0x30
#define UBX_MESSAGE_ACK_NAK 0x00
#define UBX_MESSAGE_ACK_ACK 0x01
#define UBX_MESSAGE_CFG_PRT 0x00
#define UBX_MESSAGE_CFG_MSG 0x01
#define UBX_MESSAGE_CFG_RATE 0x08
#define UBX_MESSAGE_CFG_NAV5 0x24

#define UBX_MESSAGE_MON_HW	0x09

#define UBX_CFG_PRT_LENGTH 20
#define UBX_CFG_PRT_PAYLOAD_PORTID 0x01			/**< UART1 */
#define UBX_CFG_PRT_PAYLOAD_MODE 0x000008D0		/**< 0b0000100011010000: 8N1 */
#define UBX_CFG_PRT_PAYLOAD_BAUDRATE 38400		/**< choose 38400 as GPS baudrate */
#define UBX_CFG_PRT_PAYLOAD_INPROTOMASK 0x01		/**< UBX in */
#define UBX_CFG_PRT_PAYLOAD_OUTPROTOMASK 0x01		/**< UBX out */

#define UBX_CFG_RATE_LENGTH 6
#define UBX_CFG_RATE_PAYLOAD_MEASINTERVAL 200		/**< 200ms for 5Hz */
#define UBX_CFG_RATE_PAYLOAD_NAVRATE 1			/**< cannot be changed */
#define UBX_CFG_RATE_PAYLOAD_TIMEREF 0			/**< 0: UTC, 1: GPS time */


#define UBX_CFG_NAV5_LENGTH 36
#define UBX_CFG_NAV5_PAYLOAD_MASK 0x0005		/**< XXX only update dynamic model and fix mode */
#define UBX_CFG_NAV5_PAYLOAD_DYNMODEL 7			/**< 0: portable, 2: stationary, 3: pedestrian, 4: automotive, 5: sea, 6: airborne <1g, 7: airborne <2g, 8: airborne <4g */
#define UBX_CFG_NAV5_PAYLOAD_FIXMODE 2			/**< 1: 2D only, 2: 3D only, 3: Auto 2D/3D */

#define UBX_CFG_MSG_LENGTH 8
#define UBX_CFG_MSG_PAYLOAD_RATE1_5HZ 0x01 		/**< {0x00, 0x01, 0x00, 0x00, 0x00, 0x00} the second entry is for UART1 */
#define UBX_CFG_MSG_PAYLOAD_RATE1_1HZ 0x05		/**< {0x00, 0x05, 0x00, 0x00, 0x00, 0x00} the second entry is for UART1 */
#define UBX_CFG_MSG_PAYLOAD_RATE1_05HZ 10

#define UBX_MAX_PAYLOAD_LENGTH 500

// ************
/** the structures of the binary packets */
#pragma pack(push, 1)

struct ubx_header {
	uint8_t sync1;
	uint8_t sync2;
	uint8_t msg_class;
	uint8_t msg_id;
	uint16_t length;
};

typedef struct {
	uint32_t time_milliseconds;		/**<  GPS Millisecond Time of Week */
	int32_t lon;					/**<  Longitude * 1e-7, deg */
	int32_t lat;					/**<  Latitude * 1e-7, deg */
	int32_t height;					/**<  Height above Ellipsoid, mm */
	int32_t height_msl;				/**<  Height above mean sea level, mm */
	uint32_t hAcc;  				/**< Horizontal Accuracy Estimate, mm */
	uint32_t vAcc;  				/**< Vertical Accuracy Estimate, mm */
	uint8_t ck_a;
	uint8_t ck_b;
} gps_bin_nav_posllh_packet_t;

typedef struct {
	uint32_t time_milliseconds; 	/**< GPS Millisecond Time of Week */
	int32_t time_nanoseconds;		/**< Fractional Nanoseconds remainder of rounded ms above, range -500000 .. 500000 */
	int16_t week;					/**< GPS week (GPS time) */
	uint8_t gpsFix;					/**< GPS Fix: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GPS + dead reckoning, 5 = time only fix */
	uint8_t flags;
	int32_t ecefX;
	int32_t ecefY;
	int32_t ecefZ;
	uint32_t pAcc;
	int32_t ecefVX;
	int32_t ecefVY;
	int32_t ecefVZ;
	uint32_t sAcc;
	uint16_t pDOP;
	uint8_t reserved1;
	uint8_t numSV;
	uint32_t reserved2;
	uint8_t ck_a;
	uint8_t ck_b;
} gps_bin_nav_sol_packet_t;

typedef struct {
	uint32_t time_milliseconds;		/**< GPS Millisecond Time of Week */
	uint32_t time_accuracy; 		/**< Time Accuracy Estimate, ns */
	int32_t time_nanoseconds; 		/**< Nanoseconds of second, range -1e9 .. 1e9 (UTC) */
	uint16_t year; 					/**< Year, range 1999..2099 (UTC) */
	uint8_t month; 					/**< Month, range 1..12 (UTC) */
	uint8_t day; 					/**< Day of Month, range 1..31 (UTC) */
	uint8_t hour; 					/**< Hour of Day, range 0..23 (UTC) */
	uint8_t min; 					/**< Minute of Hour, range 0..59 (UTC) */
	uint8_t sec; 					/**< Seconds of Minute, range 0..59 (UTC) */
	uint8_t valid_flag; 			/**< Validity Flags (see ubx documentation) */
	uint8_t ck_a;
	uint8_t ck_b;
} gps_bin_nav_timeutc_packet_t;

//typedef struct {
//	uint32_t time_milliseconds; 	/**<  GPS Millisecond Time of Week */
//	uint16_t gDOP; 					/**< Geometric DOP (scaling 0.01) */
//	uint16_t pDOP; 					/**< Position DOP (scaling 0.01) */
//	uint16_t tDOP; 					/**< Time DOP (scaling 0.01) */
//	uint16_t vDOP; 					/**< Vertical DOP (scaling 0.01) */
//	uint16_t hDOP; 					/**< Horizontal DOP (scaling 0.01) */
//	uint16_t nDOP; 					/**< Northing DOP (scaling 0.01) */
//	uint16_t eDOP; 					/**< Easting DOP (scaling 0.01) */
//	uint8_t ck_a;
//	uint8_t ck_b;
//} gps_bin_nav_dop_packet_t;

typedef struct {
	uint32_t time_milliseconds; 	/**<  GPS Millisecond Time of Week */
	uint8_t numCh; 					/**< Number of channels */
	uint8_t globalFlags;
	uint16_t reserved2;

} gps_bin_nav_svinfo_part1_packet_t;

typedef struct {
	uint8_t chn; 					/**< Channel number, 255 for SVs not assigned to a channel */
	uint8_t svid; 					/**< Satellite ID */
	uint8_t flags;
	uint8_t quality;
	uint8_t cno; 					/**< Carrier to Noise Ratio (Signal Strength), dbHz */
	int8_t elev; 					/**< Elevation in integer degrees */
	int16_t azim; 					/**< Azimuth in integer degrees */
	int32_t prRes; 					/**< Pseudo range residual in centimetres */

} gps_bin_nav_svinfo_part2_packet_t;

typedef struct {
	uint8_t ck_a;
	uint8_t ck_b;
} gps_bin_nav_svinfo_part3_packet_t;

typedef struct {
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	int32_t velN; //NED north velocity, cm/s
	int32_t velE; //NED east velocity, cm/s
	int32_t velD; //NED down velocity, cm/s
	uint32_t speed; //Speed (3-D), cm/s
	uint32_t gSpeed; //Ground Speed (2-D), cm/s
	int32_t heading; //Heading of motion 2-D, deg, scaling: 1e-5
	uint32_t sAcc; //Speed Accuracy Estimate, cm/s
	uint32_t cAcc; //Course / Heading Accuracy Estimate, scaling: 1e-5
	uint8_t ck_a;
	uint8_t ck_b;
} gps_bin_nav_velned_packet_t;

struct gps_bin_mon_hw_packet {
	uint32_t pinSel;
	uint32_t pinBank;
	uint32_t pinDir;
	uint32_t pinVal;
	uint16_t noisePerMS;
	uint16_t agcCnt;
	uint8_t aStatus;
	uint8_t aPower;
	uint8_t flags;
	uint8_t __reserved1;
	uint32_t usedMask;
	uint8_t VP[25];
	uint8_t jamInd;
	uint16_t __reserved3;
	uint32_t pinIrq;
	uint32_t pulLH;
	uint32_t pullL;
};


//typedef struct {
//	int32_t time_milliseconds; 		/**< Measurement integer millisecond GPS time of week */
//	int16_t week; 					/**< Measurement GPS week number */
//	uint8_t numVis;					/**< Number of visible satellites */
//
//	//... rest of package is not used in this implementation
//
//} gps_bin_rxm_svsi_packet_t;

typedef struct {
	uint8_t clsID;
	uint8_t msgID;
	uint8_t ck_a;
	uint8_t ck_b;
} gps_bin_ack_ack_packet_t;

typedef struct {
	uint8_t clsID;
	uint8_t msgID;
	uint8_t ck_a;
	uint8_t ck_b;
} gps_bin_ack_nak_packet_t;

typedef struct {
	uint8_t clsID;
	uint8_t msgID;
	uint16_t length;
	uint8_t portID;
	uint8_t res0;
	uint16_t res1;
	uint32_t mode;
	uint32_t baudRate;
	uint16_t inProtoMask;
	uint16_t outProtoMask;
	uint16_t flags;
	uint16_t pad;
	uint8_t ck_a;
	uint8_t ck_b;
} type_gps_bin_cfg_prt_packet_t;

typedef struct {
	uint8_t clsID;
	uint8_t msgID;
	uint16_t length;
	uint16_t measRate;
	uint16_t navRate;
	uint16_t timeRef;
	uint8_t ck_a;
	uint8_t ck_b;
} type_gps_bin_cfg_rate_packet_t;

typedef struct {
	uint8_t clsID;
	uint8_t msgID;
	uint16_t length;
	uint16_t mask;
	uint8_t dynModel;
	uint8_t fixMode;
	int32_t fixedAlt;
	uint32_t fixedAltVar;
	int8_t minElev;
	uint8_t drLimit;
	uint16_t pDop;
	uint16_t tDop;
	uint16_t pAcc;
	uint16_t tAcc;
	uint8_t staticHoldThresh;
	uint8_t dgpsTimeOut;
	uint32_t reserved2;
	uint32_t reserved3;
	uint32_t reserved4;
	uint8_t ck_a;
	uint8_t ck_b;
} type_gps_bin_cfg_nav5_packet_t;

typedef struct {
	uint8_t clsID;
	uint8_t msgID;
	uint16_t length;
	uint8_t msgClass_payload;
	uint8_t msgID_payload;
	uint8_t rate;
	uint8_t ck_a;
	uint8_t ck_b;
} type_gps_bin_cfg_msg_packet_t;

struct ubx_cfg_msg_rate {
	uint8_t msg_class;
	uint8_t msg_id;
	uint8_t rate;
};


// END the structures of the binary packets
// ************

typedef enum {
	UBX_DECODE_UNINIT = 0,
	UBX_DECODE_GOT_SYNC1,
	UBX_DECODE_GOT_SYNC2,
	UBX_DECODE_GOT_CLASS,
	UBX_DECODE_GOT_MESSAGEID,
	UBX_DECODE_GOT_LENGTH1,
	UBX_DECODE_GOT_LENGTH2
} ubx_decode_state_t;

//typedef type_gps_bin_ubx_state gps_bin_ubx_state_t;
#pragma pack(pop)

#define RECV_BUFFER_SIZE 300 //The NAV-SOL messages really need such a big buffer

class UBX : public GPS_Helper
{
public:
	UBX(const int &fd, struct vehicle_gps_position_s *gps_position);
	~UBX();
	int			receive(unsigned timeout);
	int			configure(unsigned &baudrate);

private:

	/**
	 * Parse the binary MTK packet
	 */
	int			parse_char(uint8_t b);

	/**
	 * Handle the package once it has arrived
	 */
	int			handle_message(void);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void			decode_init(void);

	/**
	 * While parsing add every byte (except the sync bytes) to the checksum
	 */
	void			add_byte_to_checksum(uint8_t);

	/**
	 * Add the two checksum bytes to an outgoing message
	 */
	void			add_checksum_to_message(uint8_t *message, const unsigned length);

	/**
	 * Helper to send a config packet
	 */
	void			send_config_packet(const int &fd, uint8_t *packet, const unsigned length);

	void			configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);

	void			send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size);

	void			add_checksum(uint8_t *message, const unsigned length, uint8_t &ck_a, uint8_t &ck_b);

	int			wait_for_ack(unsigned timeout);

	int			_fd;
	struct vehicle_gps_position_s *_gps_position;
	bool			_configured;
	bool			_waiting_for_ack;
	bool			_got_posllh;
	bool			_got_velned;
	bool			_got_timeutc;
	uint8_t			_message_class_needed;
	uint8_t			_message_id_needed;
	ubx_decode_state_t	_decode_state;
	uint8_t			_rx_buffer[RECV_BUFFER_SIZE];
	unsigned		_rx_count;
	uint8_t			_rx_ck_a;
	uint8_t			_rx_ck_b;
	uint8_t			_message_class;
	uint8_t			_message_id;
	unsigned		_payload_size;
	hrt_abstime		_disable_cmd_last;
};

#endif /* UBX_H_ */
