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
 * @file frsky_data.h
 * @author Stefan Rado <px4@sradonia.net>
 *
 * FrSky telemetry implementation.
 *
 */
#ifndef _FRSKY_DATA_H
#define _FRSKY_DATA_H

#include <stdbool.h>

// Public functions
bool frsky_init(void);
void frsky_deinit(void);
void frsky_update_topics(void);
void frsky_send_frame1(int uart);
void frsky_send_frame2(int uart);
void frsky_send_frame3(int uart);

struct adc_linkquality {
	uint8_t ad1;
	uint8_t ad2;
	uint8_t linkq;
};
bool frsky_parse_host(uint8_t *sbuf, int nbytes, struct adc_linkquality *v);

#endif /* _FRSKY_TELEMETRY_H */
