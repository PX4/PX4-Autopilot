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
 * @file mtk.cpp
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 */

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "mtk.h"


MTK::MTK(const int &fd, struct vehicle_gps_position_s *gps_position) :
	_fd(fd),
	_gps_position(gps_position),
	_mtk_revision(0)
{
	decode_init();
}

MTK::~MTK()
{
}

int
MTK::configure(unsigned &baudrate)
{
	/* set baudrate first */
	if (GPS_Helper::set_baudrate(_fd, MTK_BAUDRATE) != 0)
		return -1;

	baudrate = MTK_BAUDRATE;

	/* Write config messages, don't wait for an answer */
	if (strlen(MTK_OUTPUT_5HZ) != write(_fd, MTK_OUTPUT_5HZ, strlen(MTK_OUTPUT_5HZ))) {
		warnx("mtk: config write failed");
		return -1;
	}

	usleep(10000);

	if (strlen(MTK_SET_BINARY) != write(_fd, MTK_SET_BINARY, strlen(MTK_SET_BINARY))) {
		warnx("mtk: config write failed");
		return -1;
	}

	usleep(10000);

	if (strlen(SBAS_ON) != write(_fd, SBAS_ON, strlen(SBAS_ON))) {
		warnx("mtk: config write failed");
		return -1;
	}

	usleep(10000);

	if (strlen(WAAS_ON) != write(_fd, WAAS_ON, strlen(WAAS_ON))) {
		warnx("mtk: config write failed");
		return -1;
	}

	usleep(10000);

	if (strlen(MTK_NAVTHRES_OFF) != write(_fd, MTK_NAVTHRES_OFF, strlen(MTK_NAVTHRES_OFF))) {
		warnx("mtk: config write failed");
		return -1;
	}

	return 0;
}

int
MTK::receive(unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	uint8_t buf[32];
	gps_mtk_packet_t packet;

	/* timeout additional to poll */
	uint64_t time_started = hrt_absolute_time();

	int j = 0;
	ssize_t count = 0;

	while (true) {

		/* first read whatever is left */
		if (j < count) {
			/* pass received bytes to the packet decoder */
			while (j < count) {
				if (parse_char(buf[j], packet) > 0) {
					handle_message(packet);
					return 1;
				}

				/* in case we keep trying but only get crap from GPS */
				if (time_started + timeout * 1000 < hrt_absolute_time()) {
					return -1;
				}

				j++;
			}

			/* everything is read */
			j = count = 0;
		}

		/* then poll for new data */
		int ret = ::poll(fds, sizeof(fds) / sizeof(fds[0]), timeout);

		if (ret < 0) {
			/* something went wrong when polling */
			return -1;

		} else if (ret == 0) {
			/* Timeout */
			return -1;

		} else if (ret > 0) {
			/* if we have new data from GPS, go handle it */
			if (fds[0].revents & POLLIN) {
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device.  If more bytes are
				 * available, we'll go back to poll() again...
				 */
				count = ::read(_fd, buf, sizeof(buf));
			}
		}
	}
}

void
MTK::decode_init(void)
{
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_count = 0;
	_decode_state = MTK_DECODE_UNINIT;
}
int
MTK::parse_char(uint8_t b, gps_mtk_packet_t &packet)
{
	int ret = 0;

	if (_decode_state == MTK_DECODE_UNINIT) {

		if (b == MTK_SYNC1_V16) {
			_decode_state = MTK_DECODE_GOT_CK_A;
			_mtk_revision = 16;

		} else if (b == MTK_SYNC1_V19) {
			_decode_state = MTK_DECODE_GOT_CK_A;
			_mtk_revision = 19;
		}

	} else if (_decode_state == MTK_DECODE_GOT_CK_A) {
		if (b == MTK_SYNC2) {
			_decode_state = MTK_DECODE_GOT_CK_B;

		} else {
			// Second start symbol was wrong, reset state machine
			decode_init();
		}

	} else if (_decode_state == MTK_DECODE_GOT_CK_B) {
		// Add to checksum
		if (_rx_count < 33)
			add_byte_to_checksum(b);

		// Fill packet buffer
		((uint8_t *)(&packet))[_rx_count] = b;
		_rx_count++;

		/* Packet size minus checksum, XXX ? */
		if (_rx_count >= sizeof(packet)) {
			/* Compare checksum */
			if (_rx_ck_a == packet.ck_a && _rx_ck_b == packet.ck_b) {
				ret = 1;

			} else {
				warnx("MTK Checksum invalid");
				ret = -1;
			}

			// Reset state machine to decode next packet
			decode_init();
		}
	}

	return ret;
}

void
MTK::handle_message(gps_mtk_packet_t &packet)
{
	if (_mtk_revision == 16) {
		_gps_position->lat = packet.latitude * 10; // from degrees*1e6 to degrees*1e7
		_gps_position->lon = packet.longitude * 10; // from degrees*1e6 to degrees*1e7

	} else if (_mtk_revision == 19) {
		_gps_position->lat = packet.latitude; // both degrees*1e7
		_gps_position->lon = packet.longitude; // both degrees*1e7

	} else {
		warnx("mtk: unknown revision");
		_gps_position->lat = 0;
		_gps_position->lon = 0;

		// Indicate this data is not usable and bail out
		_gps_position->eph_m = 1000.0f;
		_gps_position->epv_m = 1000.0f;
		_gps_position->fix_type = 0;
		return;
	}

	_gps_position->alt = (int32_t)(packet.msl_altitude * 10); // from cm to mm
	_gps_position->fix_type = packet.fix_type;
	_gps_position->eph_m = packet.hdop / 100.0f; // from cm to m
	_gps_position->epv_m = _gps_position->eph_m; // unknown in mtk custom mode, so we cheat with eph
	_gps_position->vel_m_s = ((float)packet.ground_speed) * 1e-2f; // from cm/s to m/s
	_gps_position->cog_rad = ((float)packet.heading) * M_DEG_TO_RAD_F * 1e-2f; //from deg *100 to rad
	_gps_position->satellites_visible = packet.satellites;

	/* convert time and date information to unix timestamp */
	struct tm timeinfo; //TODO: test this conversion
	uint32_t timeinfo_conversion_temp;

	timeinfo.tm_mday = packet.date * 1e-4;
	timeinfo_conversion_temp = packet.date - timeinfo.tm_mday * 1e4;
	timeinfo.tm_mon = timeinfo_conversion_temp * 1e-2 - 1;
	timeinfo.tm_year = (timeinfo_conversion_temp - (timeinfo.tm_mon + 1) * 1e2) + 100;

	timeinfo.tm_hour = packet.utc_time * 1e-7;
	timeinfo_conversion_temp = packet.utc_time - timeinfo.tm_hour * 1e7;
	timeinfo.tm_min = timeinfo_conversion_temp * 1e-5;
	timeinfo_conversion_temp -= timeinfo.tm_min * 1e5;
	timeinfo.tm_sec = timeinfo_conversion_temp * 1e-3;
	timeinfo_conversion_temp -= timeinfo.tm_sec * 1e3;
	time_t epoch = mktime(&timeinfo);

	_gps_position->time_gps_usec = epoch * 1e6; //TODO: test this
	_gps_position->time_gps_usec += timeinfo_conversion_temp * 1e3;
	_gps_position->timestamp_position = _gps_position->timestamp_time = hrt_absolute_time();

	// Position and velocity update always at the same time
	_rate_count_vel++;
	_rate_count_lat_lon++;

	return;
}

void
MTK::add_byte_to_checksum(uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}
