/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <rk3dov@gmail.com>
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
 * @file sdlog2_messages.h
 *
 * Log messages and structures definition.
 *
 * @author Anton Babushkin <rk3dov@gmail.com>
 */

#ifndef SDLOG2_MESSAGES_H_
#define SDLOG2_MESSAGES_H_

#include "sdlog2_format.h"

/* define message formats */

/* === 1: TS - TIME STAMP === */
#define LOG_TS_MSG 1
struct log_TS_s {
	uint64_t t;
};

/* === 2: ATT - ATTITUDE === */
#define LOG_ATT_MSG 2
struct log_ATT_s {
	float roll;
	float pitch;
	float yaw;
};

/* === 3: RAW - SENSORS === */
#define LOG_RAW_MSG 3
struct log_RAW_s {
	float accX;
	float accY;
	float accZ;
};

/* construct list of all message formats */

static const struct log_format_s log_formats[] = {
	LOG_FORMAT(TS, "Q", "t"),
	LOG_FORMAT(ATT, "fff", "roll,pitch,yaw"),
	LOG_FORMAT(RAW, "fff", "accX,accY,accZ"),
};

static const int log_formats_num = sizeof(log_formats) / sizeof(struct log_format_s);

#endif /* SDLOG2_MESSAGES_H_ */
