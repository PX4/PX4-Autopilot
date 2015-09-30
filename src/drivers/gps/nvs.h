/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file nvs.h
 * 
 * NVS GNSS receiver BINR protocol implementation.
 * @(http://www.nvs-gnss.com/support/documentation/item/download/39.html)
 * 
 * LSA -Autonomous Systems Laboratory | ISEP
 *
 *
 * @author Pedro M. Sousa <1111519@isep.ipp.pt>
 * @author Tiago Miranda <tasantos@inesctec.ipp.pt>
 * @author Miguel Moreira <mmoreira@inesctec.pt>
 * @author Joel Oliveira <hjfo@inesctec.pt>
 *
 *
 */

#ifndef NVS_H_
#define NVS_H_

#include "gps_helper.h"

#define MAXRAWLEN 255

#define NVS_SYNC    0x10
#define NVS_DLE     0x10
#define NVS_ETX     0x03




#pragma pack(push, 1)


typedef struct {        /* receiver raw data control type */

	int nbyte;          /* number of bytes in message buffer */
	int len;            /* message length (bytes) */
	int flag;           /* general purpose flag */
	unsigned char buff[MAXRAWLEN]; /* message buffer */
} raw_t;

typedef struct {        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t */
	double sec;         /* fraction of second under 1 s */
} gtime_t;



//typedef type_gps_bin_ubx_state gps_bin_ubx_state_t;
#pragma pack(pop)

#define RECV_BUFFER_SIZE_NVS 500 //The NAV-SOL messages really need such a big buffer

class NVS : public GPS_Helper
{
public:
	NVS(const int &fd, struct vehicle_gps_position_s *gps_position);
	~NVS();
	int		receive(unsigned timeout);
	int		configure(unsigned &baudrate);

private:

	/**
	 * Parse the binary MTK packet
	 */
	int			parse_char(uint8_t data);

	/**
	 * Handle the package once it has arrived
	 */
	int		decode_nvs(void);

	int 	x61dop_decode(void);

	int 	x41cograd_decode(void);

	int 	x88pvt_decode(void);

	int 	x60nrs_decode(void);

	int 	xc2cfg_decode(void);

	int		_fd;

	struct vehicle_gps_position_s *_gps_position;

	bool			_configured;
	uint8_t			_rx_buffer[RECV_BUFFER_SIZE_NVS];
	raw_t           raw;
};

#endif /* NVS_H_ */
