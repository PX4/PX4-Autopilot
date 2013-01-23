/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
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

/* @file U-Blox protocol definitions */

#ifndef UBX_H_
#define UBX_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>


//internal definitions (not depending on the ubx protocol
#define UBX_NO_OF_MESSAGES 7								/**< Read 7 UBX GPS messages */
#define UBX_WATCHDOG_CRITICAL_TIME_MICROSECONDS 3000000		/**< Allow 3 seconds maximum inter-message time */
#define UBX_WATCHDOG_WAIT_TIME_MICROSECONDS 2000000			/**< Check for current state every two seconds */

#define UBX_CONFIG_TIMEOUT 1000000

#define APPNAME "gps: ubx"

#define UBX_SYNC_1 0xB5
#define UBX_SYNC_2 0x62

//UBX Protocoll definitions (this is the subset of the messages that are parsed)
#define UBX_CLASS_NAV 0x01
#define UBX_CLASS_RXM 0x02
#define UBX_CLASS_ACK 0x05
#define UBX_CLASS_CFG 0x06
#define UBX_MESSAGE_NAV_POSLLH 0x02
#define UBX_MESSAGE_NAV_SOL 0x06
#define UBX_MESSAGE_NAV_TIMEUTC 0x21
#define UBX_MESSAGE_NAV_DOP 0x04
#define UBX_MESSAGE_NAV_SVINFO 0x30
#define UBX_MESSAGE_NAV_VELNED 0x12
#define UBX_MESSAGE_RXM_SVSI 0x20
#define UBX_MESSAGE_ACK_ACK 0x01
#define UBX_MESSAGE_ACK_NAK 0x00
#define UBX_MESSAGE_CFG_PRT 0x00
#define UBX_MESSAGE_CFG_NAV5 0x24
#define UBX_MESSAGE_CFG_MSG 0x01

#define UBX_CFG_PRT_LENGTH 20
#define UBX_CFG_PRT_PAYLOAD_PORTID 0x01         /**< port 1 */
#define UBX_CFG_PRT_PAYLOAD_MODE 0x000008D0     /**< 0b0000100011010000: 8N1 */
#define UBX_CFG_PRT_PAYLOAD_BAUDRATE 38400      /**< always choose 38400 as GPS baudrate */
#define UBX_CFG_PRT_PAYLOAD_INPROTOMASK 0x01    /**< ubx in */
#define UBX_CFG_PRT_PAYLOAD_OUTPROTOMASK 0x01   /**< ubx out */

#define UBX_CFG_NAV5_LENGTH 36
#define UBX_CFG_NAV5_PAYLOAD_MASK 0x0001		/**< only update dynamic model and fix mode */
#define UBX_CFG_NAV5_PAYLOAD_DYNMODEL 7			/**< 0: portable, 2: stationary, 3: pedestrian, 4: automotive, 5: sea, 6: airborne <1g, 7: airborne <2g, 8: airborne <4g */
#define UBX_CFG_NAV5_PAYLOAD_FIXMODE 2          /**< 1: 2D only, 2: 3D only, 3: Auto 2D/3D */

#define UBX_CFG_MSG_LENGTH 8
#define UBX_CFG_MSG_PAYLOAD_RATE {0x00, 0x01, 0x00, 0x00, 0x00, 0x00} /**< UART1 chosen */


// ************
/** the structures of the binary packets */
#pragma pack(push, 1)
typedef struct {
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	int32_t lon;  // Longitude * 1e-7, deg
	int32_t lat; // Latitude * 1e-7, deg
	int32_t height;  // Height above Ellipsoid, mm
	int32_t height_msl;  // Height above mean sea level, mm
	uint32_t hAcc;  // Horizontal Accuracy Estimate, mm
	uint32_t vAcc;  // Vertical Accuracy Estimate, mm

	uint8_t ck_a;
	uint8_t ck_b;
} type_gps_bin_nav_posllh_packet;

typedef type_gps_bin_nav_posllh_packet gps_bin_nav_posllh_packet_t;

typedef struct {
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	int32_t time_nanoseconds; // Fractional Nanoseconds remainder of rounded ms above, range -500000 .. 500000
	int16_t week; // GPS week (GPS time)
	uint8_t gpsFix; //GPS Fix: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GPS + dead reckoning, 5 = time only fix
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
} type_gps_bin_nav_sol_packet;

typedef type_gps_bin_nav_sol_packet gps_bin_nav_sol_packet_t;

typedef struct {
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	uint32_t time_accuracy;  //Time Accuracy Estimate, ns
	int32_t time_nanoseconds; //Nanoseconds of second, range -1e9 .. 1e9 (UTC)
	uint16_t year; //Year, range 1999..2099 (UTC)
	uint8_t month; //Month, range 1..12 (UTC)
	uint8_t day; //Day of Month, range 1..31 (UTC)
	uint8_t hour; //Hour of Day, range 0..23 (UTC)
	uint8_t min; //Minute of Hour, range 0..59 (UTC)
	uint8_t sec; //Seconds of Minute, range 0..59 (UTC)
	uint8_t valid_flag; //Validity Flags (see ubx documentation)


	uint8_t ck_a;
	uint8_t ck_b;
} type_gps_bin_nav_timeutc_packet;

typedef type_gps_bin_nav_timeutc_packet gps_bin_nav_timeutc_packet_t;

typedef struct {
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	uint16_t gDOP; //Geometric DOP (scaling 0.01)
	uint16_t pDOP; //Position DOP (scaling 0.01)
	uint16_t tDOP; //Time DOP (scaling 0.01)
	uint16_t vDOP; //Vertical DOP (scaling 0.01)
	uint16_t hDOP; //Horizontal DOP (scaling 0.01)
	uint16_t nDOP; //Northing DOP (scaling 0.01)
	uint16_t eDOP; //Easting DOP (scaling 0.01)


	uint8_t ck_a;
	uint8_t ck_b;
} type_gps_bin_nav_dop_packet;

typedef type_gps_bin_nav_dop_packet gps_bin_nav_dop_packet_t;

typedef struct {
	uint32_t time_milliseconds; // GPS Millisecond Time of Week
	uint8_t numCh; //Number of channels
	uint8_t globalFlags;
	uint16_t reserved2;

} type_gps_bin_nav_svinfo_part1_packet;

typedef type_gps_bin_nav_svinfo_part1_packet gps_bin_nav_svinfo_part1_packet_t;

typedef struct {
	uint8_t chn; //Channel number, 255 for SVs not assigned to a channel
	uint8_t svid; //Satellite ID
	uint8_t flags;
	uint8_t quality;
	uint8_t cno; //Carrier to Noise Ratio (Signal Strength), dbHz
	int8_t elev; //Elevation in integer degrees
	int16_t azim; //Azimuth in integer degrees
	int32_t prRes; //Pseudo range residual in centimetres

} type_gps_bin_nav_svinfo_part2_packet;

typedef type_gps_bin_nav_svinfo_part2_packet gps_bin_nav_svinfo_part2_packet_t;

typedef struct {
	uint8_t ck_a;
	uint8_t ck_b;

} type_gps_bin_nav_svinfo_part3_packet;

typedef type_gps_bin_nav_svinfo_part3_packet gps_bin_nav_svinfo_part3_packet_t;


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
} type_gps_bin_nav_velned_packet;

typedef type_gps_bin_nav_velned_packet gps_bin_nav_velned_packet_t;

typedef struct {
	int32_t time_milliseconds; // Measurement integer millisecond GPS time of week
	int16_t week; //Measurement GPS week number
	uint8_t numVis; //Number of visible satellites

	//... rest of package is not used in this implementation

} type_gps_bin_rxm_svsi_packet;

typedef type_gps_bin_rxm_svsi_packet gps_bin_rxm_svsi_packet_t;

typedef struct {
	uint8_t clsID;
	uint8_t msgID;

	uint8_t ck_a;
	uint8_t ck_b;
} type_gps_bin_ack_ack_packet;

typedef type_gps_bin_ack_ack_packet gps_bin_ack_ack_packet_t;

typedef struct {
	uint8_t clsID;
	uint8_t msgID;

	uint8_t ck_a;
	uint8_t ck_b;
} type_gps_bin_ack_nak_packet;

typedef type_gps_bin_ack_nak_packet gps_bin_ack_nak_packet_t;

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
} type_gps_bin_cfg_prt_packet;

typedef type_gps_bin_cfg_prt_packet type_gps_bin_cfg_prt_packet_t;

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
} type_gps_bin_cfg_nav5_packet;

typedef type_gps_bin_cfg_nav5_packet type_gps_bin_cfg_nav5_packet_t;

typedef struct {
	uint8_t clsID;
	uint8_t msgID;
	uint16_t length;
	uint8_t msgClass_payload;
	uint8_t msgID_payload;
	uint8_t rate[6];

	uint8_t ck_a;
	uint8_t ck_b;
} type_gps_bin_cfg_msg_packet;

typedef type_gps_bin_cfg_msg_packet type_gps_bin_cfg_msg_packet_t;


// END the structures of the binary packets
// ************

enum UBX_CONFIG_STATE {
	UBX_CONFIG_STATE_NONE = 0,
	UBX_CONFIG_STATE_PRT = 1,
	UBX_CONFIG_STATE_NAV5 = 2,
	UBX_CONFIG_STATE_MSG_NAV_POSLLH = 3,
	UBX_CONFIG_STATE_MSG_NAV_TIMEUTC = 4,
	UBX_CONFIG_STATE_MSG_NAV_DOP = 5,
	UBX_CONFIG_STATE_MSG_NAV_SVINFO = 6,
	UBX_CONFIG_STATE_MSG_NAV_SOL = 7,
	UBX_CONFIG_STATE_MSG_NAV_VELNED = 8,
	UBX_CONFIG_STATE_MSG_RXM_SVSI = 9,
	UBX_CONFIG_STATE_CONFIGURED = 10
};

enum UBX_MESSAGE_CLASSES {
	CLASS_UNKNOWN = 0,
	NAV = 1,
	RXM = 2,
	ACK = 3,
	CFG = 4
};

enum UBX_MESSAGE_IDS {
	//these numbers do NOT correspond to the message id numbers of the ubx protocol
	ID_UNKNOWN = 0,
	NAV_POSLLH = 1,
	NAV_SOL = 2,
	NAV_TIMEUTC = 3,
	NAV_DOP = 4,
	NAV_SVINFO = 5,
	NAV_VELNED = 6,
	RXM_SVSI = 7,
	CFG_NAV5 = 8,
	ACK_ACK = 9,
	ACK_NAK = 10
};

enum UBX_DECODE_STATES {
	UBX_DECODE_UNINIT = 0,
	UBX_DECODE_GOT_SYNC1 = 1,
	UBX_DECODE_GOT_SYNC2 = 2,
	UBX_DECODE_GOT_CLASS = 3,
	UBX_DECODE_GOT_MESSAGEID = 4,
	UBX_DECODE_GOT_LENGTH1 = 5,
	UBX_DECODE_GOT_LENGTH2 = 6
};

typedef struct {
	union {
		uint16_t ck;
		struct {
			uint8_t ck_a;
			uint8_t ck_b;
		};
	};
	enum UBX_DECODE_STATES decode_state;
	bool print_errors;
	int16_t rx_count;
	uint16_t payload_size;

	enum UBX_MESSAGE_CLASSES  message_class;
	enum UBX_MESSAGE_IDS message_id;
	uint64_t last_message_timestamps[UBX_NO_OF_MESSAGES];

} type_gps_bin_ubx_state;

typedef type_gps_bin_ubx_state gps_bin_ubx_state_t;
#pragma pack(pop)

extern pthread_mutex_t *ubx_mutex;
extern gps_bin_ubx_state_t *ubx_state;

void ubx_decode_init(void);

void ubx_checksum(uint8_t b, uint8_t *ck_a, uint8_t *ck_b);



int ubx_parse(uint8_t b,  char *gps_rx_buffer);

int configure_gps_ubx(int *fd);

int read_gps_ubx(int *fd, char *gps_rx_buffer, int buffer_size);

int write_config_message_ubx(const uint8_t *message, const size_t length, const int *fd);

void calculate_ubx_checksum(uint8_t *message, uint8_t length);

void *ubx_watchdog_loop(void *args);

void *ubx_loop(void *args);


#endif /* UBX_H_ */
