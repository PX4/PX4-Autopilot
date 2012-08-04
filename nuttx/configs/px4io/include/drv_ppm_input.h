/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file PPM input decoder.
 *
 * Works in conjunction with the HRT driver, exports a device node
 * and a message queue (if message queues are enabled).
 *
 * Note that the device node supports both blocking and non-blocking
 * opens, but actually never blocks.  A nonblocking open will return
 * EWOULDBLOCK if there has not been an update since the last read,
 * while a blocking open will always return the most recent data.
 */

#include <sys/ioctl.h>

#define _PPM_INPUT_BASE		0x7600

/*
 * Fetch the state of the PPM input detector.
 */
#define PPM_INPUT_STATUS	_IOC(_PPM_INPUT_BASE, 0)

typedef enum {
	PPM_STATUS_NO_SIGNAL		= 0,
	PPM_STATUS_SIGNAL_CURRENT	= 1,
} ppm_input_status_t;

/*
 * Fetch the number of channels decoded (only valid when PPM_STATUS_SIGNAL_CURRENT).
 */
#define PPM_INPUT_CHANNELS	_IOC(_PPM_INPUT_BASE, 1)

typedef int ppm_input_channel_count_t;

/*
 * Device node
 */
#define PPM_DEVICE_NODE		"/dev/ppm_input"

/*
 * Message queue; if message queues are supported, PPM input data is
 * supplied to the queue when a frame is decoded.
 */
#ifndef CONFIG_DISABLE_MQUEUE
# define PPM_MESSAGE_QUEUE	"ppm_input"
#endif

/* 
 * Private hook from the HRT driver to the PPM decoder.
 *
 * This function is called for every edge of the incoming PPM
 * signal.
 *
 * @param reset		If true, the decoder should be reset (e.g.)
 *			capture failure was detected.
 * @param count		The counter value at which the edge
 *			was captured.
 */

void	ppm_input_decode(bool reset, uint16_t count);

/*
 * PPM input initialisation function.
 *
 * If message queues are enabled, and mq_name is not NULL, received input
 * is posted to the message queue as an array of 16-bit unsigned channel values.
 */
int	ppm_input_init(const char *mq_name);