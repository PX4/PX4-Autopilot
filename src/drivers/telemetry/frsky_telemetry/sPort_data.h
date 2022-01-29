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
#define SMARTPORT_SENSOR_ID_SP2UR     0xC6 // Sensor ID  6

/* FrSky SmartPort sensor IDs. See more here: https://github.com/opentx/opentx/blob/2.2/radio/src/telemetry/frsky.h */
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
#define SMARTPORT_ID_ACCX          0x0700
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
#define SMARTPORT_ID_DIY_FIRST     0x5000
#define SMARTPORT_ID_DIY_LAST      0x50ff  //We have 256 possible ID's for custom values :)
#define SMARTPORT_ID_DIY_NAVSTATE  0x5000
#define SMARTPORT_ID_DIY_GPSFIX    0x5001

// Public functions
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
void sPort_send_GPS_LON(int uart);
void sPort_send_GPS_LAT(int uart);
void sPort_send_GPS_ALT(int uart);
void sPort_send_GPS_SPD(int uart);
void sPort_send_GPS_CRS(int uart);
void sPort_send_GPS_TIME(int uart);
void sPort_send_flight_mode(int uart);
void sPort_send_GPS_info(int uart);

void sPort_send_NAV_STATE(int uart);
void sPort_send_GPS_FIX(int uart);

#endif /* _SPORT_TELEMETRY_H */
