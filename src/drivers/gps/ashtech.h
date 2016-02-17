/****************************************************************************
 *
 *   Copyright (C) 2013. All rights reserved.
 *   Author: Boriskin Aleksey <a.d.boriskin@gmail.com>
 *           Kistanov Alexander <akistanov@gramant.ru>
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

/* @file ASHTECH protocol definitions */

#ifndef ASHTECH_H_
#define ASHTECH_H_

#include "gps_helper.h"

#ifndef RECV_BUFFER_SIZE
#define RECV_BUFFER_SIZE 512
#endif

#include <uORB/topics/satellite_info.h>


class ASHTECH : public GPS_Helper
{
	enum ashtech_decode_state_t {
		NME_DECODE_UNINIT,
		NME_DECODE_GOT_SYNC1,
		NME_DECODE_GOT_ASTERIKS,
		NME_DECODE_GOT_FIRST_CS_BYTE
	};

	int                    _fd;
	struct satellite_info_s *_satellite_info;
	struct vehicle_gps_position_s *_gps_position;
	int ashtechlog_fd;

	ashtech_decode_state_t   _decode_state;
	uint8_t               _rx_buffer[RECV_BUFFER_SIZE];
	uint16_t              _rx_buffer_bytes;
	bool                  _parse_error; 		/** parse error flag */
	char                 *_parse_pos; 		/** parse position */

	bool	_gsv_in_progress;			/**< Indicates that gsv data parsing is in progress */
	/* int     _satellites_count; 			**< Number of satellites info parsed. */
	uint8_t count;					/**< Number of satellites in satellite info */
	uint8_t svid[satellite_info_s::SAT_INFO_MAX_SATELLITES]; 		/**< Space vehicle ID [1..255], see scheme below  */
	uint8_t used[satellite_info_s::SAT_INFO_MAX_SATELLITES];		/**< 0: Satellite not used, 1: used for navigation */
	uint8_t elevation[satellite_info_s::SAT_INFO_MAX_SATELLITES];	/**< Elevation (0: right on top of receiver, 90: on the horizon) of satellite */
	uint8_t azimuth[satellite_info_s::SAT_INFO_MAX_SATELLITES];	/**< Direction of satellite, 0: 0 deg, 255: 360 deg. */
	uint8_t snr[satellite_info_s::SAT_INFO_MAX_SATELLITES];		/**< dBHz, Signal to noise ratio of satellite C/N0, range 0..99, zero when not tracking this satellite. */

public:
	ASHTECH(const int &fd, struct vehicle_gps_position_s *gps_position, struct satellite_info_s *satellite_info);
	~ASHTECH();
	int             receive(unsigned timeout);
	int             configure(unsigned &baudrate);
	void            decode_init(void);
	int             handle_message(int len);
	int             parse_char(uint8_t b);
	/** Read int ASHTECH parameter */
	int32_t         read_int();
	/** Read float ASHTECH parameter */
	double       read_float();
	/** Read char ASHTECH parameter */
	char            read_char();

};

#endif /* ASHTECH_H_ */
