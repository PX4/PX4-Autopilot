/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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

/* @file mkt.cpp */

#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <drivers/drv_hrt.h>

#include "mtk.h"


MTK::MTK()
{
	decodeInit();
}

MTK::~MTK()
{
}

void
MTK::reset()
{

}

void
MTK::configure(const int &fd, bool &baudrate_changed, unsigned &baudrate)
{
	if (strlen(MTK_OUTPUT_5HZ) != write(fd, MTK_OUTPUT_5HZ, strlen(MTK_OUTPUT_5HZ)))
		warnx("mtk: config write failed");
	usleep(10000);

	if (strlen(MTK_SET_BINARY) != write(fd, MTK_SET_BINARY, strlen(MTK_SET_BINARY)))
		warnx("mtk: config write failed");
	usleep(10000);

	if (strlen(SBAS_ON) != write(fd, SBAS_ON, strlen(SBAS_ON)))
		warnx("mtk: config write failed");
	usleep(10000);

	if (strlen(WAAS_ON) != write(fd, WAAS_ON, strlen(WAAS_ON)))
		warnx("mtk: config write failed");
	usleep(10000);

	if (strlen(MTK_NAVTHRES_OFF) != write(fd, MTK_NAVTHRES_OFF, strlen(MTK_NAVTHRES_OFF)))
		warnx("mtk: config write failed");

	return;
}

void
MTK::decodeInit(void)
{
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_count = 0;
	_decode_state = MTK_DECODE_UNINIT;
}

void
MTK::parse(uint8_t b, struct vehicle_gps_position_s *gps_position, bool &config_needed, bool &pos_updated)
{
	if (_decode_state == MTK_DECODE_UNINIT) {

		if (b == 0xd0) {
			_decode_state = MTK_DECODE_GOT_CK_A;
			config_needed = false;
		}

	} else if (_decode_state == MTK_DECODE_GOT_CK_A) {
		if (b == 0xdd) {
			_decode_state = MTK_DECODE_GOT_CK_B;

		} else {
			// Second start symbol was wrong, reset state machine
			decodeInit();
		}

	} else if (_decode_state == MTK_DECODE_GOT_CK_B) {
		// Add to checksum
		if (_rx_count < 33)
			addByteToChecksum(b);

		// Fill packet buffer
		_rx_buffer[_rx_count] = b;
		_rx_count++;

		/* Packet size minus checksum */
		if (_rx_count >= 35) {
			type_gps_mtk_packet *packet = (type_gps_mtk_packet *) _rx_buffer;;

			/* Check if checksum is valid */
			if (_rx_ck_a == packet->ck_a && _rx_ck_b == packet->ck_b) {
				gps_position->lat = packet->latitude * 10; // from degrees*1e6 to degrees*1e7
				gps_position->lon = packet->longitude * 10; // from degrees*1e6 to degrees*1e7
				gps_position->alt = (int32_t)(packet->msl_altitude * 10); // from cm to mm
				gps_position->fix_type = packet->fix_type;
				gps_position->eph_m = packet->hdop;
				gps_position->epv_m = 0.0; //unknown in mtk custom mode
				gps_position->vel_m_s = ((float)packet->ground_speed)*1e-2f; // from cm/s to m/s
				gps_position->cog_rad = ((float)packet->heading) * M_DEG_TO_RAD_F * 1e-2f; //from deg *100 to rad
				gps_position->satellites_visible = packet->satellites;

				/* convert time and date information to unix timestamp */
				struct tm timeinfo; //TODO: test this conversion
				uint32_t timeinfo_conversion_temp;

				timeinfo.tm_mday = packet->date * 1e-4;
				timeinfo_conversion_temp = packet->date - timeinfo.tm_mday * 1e4;
				timeinfo.tm_mon = timeinfo_conversion_temp * 1e-2 - 1;
				timeinfo.tm_year = (timeinfo_conversion_temp - (timeinfo.tm_mon + 1) * 1e2) + 100;

				timeinfo.tm_hour = packet->utc_time * 1e-7;
				timeinfo_conversion_temp = packet->utc_time - timeinfo.tm_hour * 1e7;
				timeinfo.tm_min = timeinfo_conversion_temp * 1e-5;
				timeinfo_conversion_temp -= timeinfo.tm_min * 1e5;
				timeinfo.tm_sec = timeinfo_conversion_temp * 1e-3;
				timeinfo_conversion_temp -= timeinfo.tm_sec * 1e3;
				time_t epoch = mktime(&timeinfo);

				gps_position->time_gps_usec = epoch * 1e6; //TODO: test this
				gps_position->time_gps_usec += timeinfo_conversion_temp * 1e3;
				gps_position->timestamp_position = gps_position->timestamp_time = hrt_absolute_time();

				pos_updated = true;


			} else {
				warnx("mtk Checksum invalid, 0x%x, 0x%x instead of 0x%x, 0x%x", _rx_ck_a, packet->ck_a, _rx_ck_b, packet->ck_b);
			}

			// Reset state machine to decode next packet
			decodeInit();
		}
	}
	return;
}

void
MTK::addByteToChecksum(uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}
