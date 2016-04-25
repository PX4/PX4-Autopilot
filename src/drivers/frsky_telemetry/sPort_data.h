/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
 *   Author: Stefan Rado <px4@sradonia.net>
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
 * @file sPort_data.h
 * @author Mark Whitehorn <kd0aij@github.com>
 *
 * FrSky SmartPort telemetry implementation.
 *
 */
#ifndef _SPORT_DATA_H
#define _SPORT_DATA_H

#include <sys/types.h>
#include <stdbool.h>

/* FrSky SmartPort polling IDs captured from X4R */
#define SMARTPORT_POLL_1    0x1B
#define SMARTPORT_POLL_2    0x34
#define SMARTPORT_POLL_3    0x95
#define SMARTPORT_POLL_4    0x16
#define SMARTPORT_POLL_5    0xB7
#define SMARTPORT_POLL_6    0x00
#define SMARTPORT_POLL_7    0x83
#define SMARTPORT_POLL_8    0xBA

/* FrSky SmartPort sensor IDs. See more here: https://github.com/opentx/opentx/blob/master/radio/src/telemetry/frsky.h#L109 */
#define SMARTPORT_ID_RSSI          0xf101
#define SMARTPORT_ID_RXA1          0xf102	// supplied by RX
#define SMARTPORT_ID_RXA2          0xf103	// supplied by RX
#define SMARTPORT_ID_BATV          0xf104
#define SMARTPORT_ID_SWR           0xf105   // Standing Wave Ratio
#define SMARTPORT_ID_T1            0x0400
#define SMARTPORT_ID_T2            0x0410
#define SMARTPORT_ID_RPM           0x0500
#define SMARTPORT_ID_FUEL          0x0600
#define SMARTPORT_ID_ALT           0x0100
#define SMARTPORT_ID_VARIO         0x0110   //VSPEED
#define SMARTPORT_ID_ACCX          0x0700   //Measured in g!
#define SMARTPORT_ID_ACCY          0x0710
#define SMARTPORT_ID_ACCZ          0x0720
#define SMARTPORT_ID_CURR          0x0200
#define SMARTPORT_ID_VFAS          0x0210  //Volt per Cell
#define SMARTPORT_ID_CELLS         0x0300
#define SMARTPORT_ID_GPS_LON_LAT   0x0800
#define SMARTPORT_ID_GPS_ALT       0x0820
#define SMARTPORT_ID_GPS_SPD       0x0830
#define SMARTPORT_ID_GPS_CRS       0x0840
#define SMARTPORT_ID_GPS_TIME      0x0850

/* We have 256 possible ID's for custom values, from 0x5000 to 0x50ff
 * These are offset by 128 for future compability with APM (0-128 for APM, 128-255 for PX4)
 */
#define SMARTPORT_ID_DIY_NAV_STATE  0x5080
#define SMARTPORT_ID_DIY_GPS_FIX    0x5081
#define SMARTPORT_ID_DIY_ARMING_STATE    0x5082
#define SMARTPORT_ID_DIY_ATTITUDE_ROLL  0x5083
#define SMARTPORT_ID_DIY_ATTITUDE_PITCH   0x5084
#define SMARTPORT_ID_DIY_ATTITUDE_YAW   0x5085
#define SMARTPORT_ID_DIY_MISSION_COUNT 0x5086
#define SMARTPORT_ID_DIY_MISSION_SEQUENCE_REACHED 0x5087 //Sequence is an id for a mission item - "a thing to do"
#define SMARTPORT_ID_DIY_MISSION_SEQUENCE_CURRENT 0x5088
#define SMARTPORT_ID_DIY_MISSION_SEQUENCE_STATUS 0x5089

/* Public functions
TODO: mavlink messages, home position?
*/
bool sPort_init(void);
void sPort_deinit(void);
void sPort_update_topics(void);
void sPort_send_data(int uart, uint16_t id, uint32_t data);
void sPort_send_BATV(int uart);
void sPort_send_CUR(int uart);
void sPort_send_ALT(int uart);
void sPort_send_SPD(int uart);
void sPort_send_VSPD(int uart, float speed);
void sPort_send_FUEL(int uart);
void sPort_send_ACCX(int uart);
void sPort_send_ACCY(int uart);
void sPort_send_ACCZ(int uart);
void sPort_send_GPS_LON(int uart);
void sPort_send_GPS_LAT(int uart);
void sPort_send_GPS_ALT(int uart);
void sPort_send_GPS_SPD(int uart);
void sPort_send_GPS_CRS(int uart);
void sPort_send_GPS_DATE(int uart);
void sPort_send_GPS_TIME(int uart);
void sPort_send_GPS_FIX(int uart);
void sPort_send_NAV_STATE(int uart);
void sPort_send_ARMING_STATE(int uart);
void sPort_send_ATTITUDE_ROLL(int uart);
void sPort_send_ATTITUDE_PITCH(int uart);
void sPort_send_ATTITUDE_YAW(int uart);
void sPort_send_MISSION_SEQUENCE_CURRENT(int uart);
void sPort_send_MISSION_SEQUENCE_REACHED(int uart);
void sPort_send_MISSION_SEQUENCE_STATUS(int uart);

#endif /* _SPORT_TELEMETRY_H */
