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
 * @file Input capture interface.
 *
 * Input capture interface utilizes the FMU_AUX_PINS to time stamp
 * an edge.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "drv_hrt.h"

__BEGIN_DECLS

typedef void (*capture_callback_t)(void *context, uint32_t chan_index,
				   hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);

typedef uint16_t capture_filter_t;

typedef enum input_capture_edge {
	Disabled 	= 	0,
	Rising 	 	=   1,
	Falling		=   2,
	Both		=   3
} input_capture_edge;

typedef struct input_capture_stats_t {
	uint32_t 		   edges;
	uint32_t 		   overflows;
	uint32_t		   last_edge;
	hrt_abstime		   last_time;
	uint16_t		   latency;
} input_capture_stats_t;


__EXPORT int up_input_capture_set(unsigned channel, input_capture_edge edge, capture_filter_t filter,
				  capture_callback_t callback, void *context);

__EXPORT int up_input_capture_get_filter(unsigned channel, capture_filter_t *filter);
__EXPORT int up_input_capture_set_filter(unsigned channel,  capture_filter_t filter);

__EXPORT int up_input_capture_get_trigger(unsigned channel,  input_capture_edge *edge);
__EXPORT int up_input_capture_set_trigger(unsigned channel,  input_capture_edge edge);
__EXPORT int up_input_capture_get_callback(unsigned channel, capture_callback_t *callback, void **context);
__EXPORT int up_input_capture_set_callback(unsigned channel, capture_callback_t callback, void *context);
__EXPORT int up_input_capture_get_stats(unsigned channel, input_capture_stats_t *stats, bool clear);
__END_DECLS
