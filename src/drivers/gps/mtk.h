/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @author Julian Oes <julian@oes.ch>
 */

#ifndef MTK_H_
#define MTK_H_

#include "gps_helper.h"

#define MTK_SYNC1_V16 0xd0
#define MTK_SYNC1_V19 0xd1
#define MTK_SYNC2 0xdd

#define MTK_OUTPUT_5HZ		"$PMTK220,200*2C\r\n"
#define MTK_SET_BINARY		"$PGCMD,16,0,0,0,0,0*6A\r\n"
#define SBAS_ON	        	"$PMTK313,1*2E\r\n"
#define WAAS_ON				"$PMTK301,2*2E\r\n"
#define MTK_NAVTHRES_OFF 	"$PMTK397,0*23\r\n"

#define MTK_TIMEOUT_5HZ 400
#define MTK_BAUDRATE 38400

typedef enum {
	MTK_DECODE_UNINIT = 0,
	MTK_DECODE_GOT_CK_A = 1,
	MTK_DECODE_GOT_CK_B = 2
} mtk_decode_state_t;

/** the structures of the binary packets */
#pragma pack(push, 1)

typedef struct {
	uint8_t payload; ///< Number of payload bytes
	int32_t latitude;  ///< Latitude in degrees * 10^7
	int32_t longitude; ///< Longitude in degrees * 10^7
	uint32_t msl_altitude;  ///< MSL altitude in meters * 10^2
	uint32_t ground_speed; ///< velocity in m/s
	int32_t heading; ///< heading in degrees * 10^2
	uint8_t satellites; ///< number of satellites used
	uint8_t fix_type;  ///< fix type: XXX correct for that
	uint32_t date;
	uint32_t utc_time;
	uint16_t hdop; ///< horizontal dilution of position (without unit)
	uint8_t ck_a;
	uint8_t ck_b;
} gps_mtk_packet_t;

#pragma pack(pop)

#define MTK_RECV_BUFFER_SIZE 40

class MTK : public GPS_Helper
{
public:
	MTK(const int &fd, struct vehicle_gps_position_s *gps_position);
	~MTK();
	int				receive(unsigned timeout);
	int				configure(unsigned &baudrate);

private:
	/**
	 * Parse the binary MTK packet
	 */
	int				parse_char(uint8_t b, gps_mtk_packet_t &packet);

	/**
	 * Handle the package once it has arrived
	 */
	void				handle_message(gps_mtk_packet_t &packet);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void				decode_init(void);

	/**
	 * While parsing add every byte (except the sync bytes) to the checksum
	 */
	void				add_byte_to_checksum(uint8_t);

	int					_fd;
	struct vehicle_gps_position_s *_gps_position;
	mtk_decode_state_t	_decode_state;
	uint8_t				_mtk_revision;
	unsigned			_rx_count;
	uint8_t 			_rx_ck_a;
	uint8_t				_rx_ck_b;
};

#endif /* MTK_H_ */
