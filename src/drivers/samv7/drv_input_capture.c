/****************************************************************************
 *
 *   Copyright (C) 2012-2016 PX4 Development Team. All rights reserved.
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

/*
 * @file drv_input_capture.c
 *
 * Servo driver supporting input capture connected to STM32 timer blocks.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that
 * have input pins.
 *
 * Require an interrupt.
 *
 * The use of thie interface is mutually exclusive with the pwm
 * because the same timers are used and there is a resource contention
 * with the ARR as it sets the pwm rate and in this driver needs to match
 * that of the hrt to back calculate the actual point in time the edge
 * was detected.
 *
 * This  is accomplished by taking the difference between the current
 * count rCNT snapped at the time interrupt and the rCCRx captured on the
 * edge transition. This delta is applied to hrt time and the resulting
 * value is the absolute system time the edge occured.
 *
 *
 */

#include <px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_input_capture.h>
#include "drv_io_timer.h"

#include "drv_input_capture.h"

#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

static input_capture_stats_t channel_stats[MAX_TIMER_IO_CHANNELS];

static struct channel_handler_entry {
	capture_callback_t callback;
	void			  *context;
} channel_handlers[MAX_TIMER_IO_CHANNELS];

static void input_capture_chan_handler(void *context, const io_timers_t *timer, uint32_t chan_index,
				       const timer_io_channels_t *chan,
				       hrt_abstime isrs_time, uint16_t isrs_rcnt)
{
}

static void input_capture_bind(unsigned channel, capture_callback_t callback, void *context)
{
	irqstate_t flags = enter_critical_section();
	channel_handlers[channel].callback = callback;
	channel_handlers[channel].context = context;
	leave_critical_section(flags);
}

static void input_capture_unbind(unsigned channel)
{
	input_capture_bind(channel, NULL, NULL);
}

int up_input_capture_set(unsigned channel, input_capture_edge edge, capture_filter_t filter,
			 capture_callback_t callback, void *context)
{
	if (filter > 200) {//GTIM_CCMR1_IC1F_MASK) {
		return -EINVAL;
	}

	if (edge > Both) {
		return -EINVAL;
	}

	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {
		if (edge == Disabled) {

			io_timer_set_enable(false, IOTimerChanMode_Capture, 1 << channel);
			input_capture_unbind(channel);

		} else {

			if (-EBUSY == io_timer_is_channel_free(channel)) {
				io_timer_free_channel(channel);
			}

			input_capture_bind(channel, callback, context);

			rv = io_timer_channel_init(channel, IOTimerChanMode_Capture, input_capture_chan_handler, context);

			if (rv != 0) {
				return rv;
			}

			rv = up_input_capture_set_filter(channel, filter);

			if (rv == 0) {
				rv = up_input_capture_set_trigger(channel, edge);

				if (rv == 0) {
					rv = io_timer_set_enable(true, IOTimerChanMode_Capture, 1 << channel);
				}
			}
		}
	}

	return rv;
}



int up_input_capture_get_filter(unsigned channel, capture_filter_t *filter)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			uint32_t timer = timer_io_channels[channel].timer_index;
			rv = OK;

			switch (timer_io_channels[channel].timer_channel) {

			case 1:
			case 2:
			case 3:
			case 4:
			default:
				UNUSED(timer);
				rv = -EIO;
			}
		}
	}

	return rv;
}
int up_input_capture_set_filter(unsigned channel,  capture_filter_t filter)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			rv = OK;
			uint32_t timer = timer_io_channels[channel].timer_index;

			irqstate_t flags = enter_critical_section();

			switch (timer_io_channels[channel].timer_channel) {

			case 1:
			case 2:
			case 3:
			case 4:
			default:
				UNUSED(timer);
				rv = -EIO;
			}

			leave_critical_section(flags);
		}
	}

	return rv;
}

int up_input_capture_get_trigger(unsigned channel,  input_capture_edge *edge)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			rv = OK;

			uint32_t timer = timer_io_channels[channel].timer_index;
			uint32_t rvalue = 0;

			switch (timer_io_channels[channel].timer_channel) {

			case 1:
			case 2:
			case 3:
			case 4:
			default:
				UNUSED(timer);
				rv = -EIO;
			}

			if (rv == 0) {
				switch (rvalue) {

				case 0:
					*edge = Rising;
					break;

				case 1:// (GTIM_CCER_CC1P | GTIM_CCER_CC1NP):
					*edge = Both;
					break;

				case 2: //(GTIM_CCER_CC1P):
					*edge = Falling;
					break;

				default:
					rv = -EIO;
				}
			}
		}
	}

	return rv;
}
int up_input_capture_set_trigger(unsigned channel,  input_capture_edge edge)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			uint16_t edge_bits = 0xffff;

			switch (edge) {
			case Disabled:
				break;

			case Rising:
				edge_bits = 0;
				break;

			case Falling:
				edge_bits = 0;// GTIM_CCER_CC1P;
				break;

			case Both:
				edge_bits = 0;// GTIM_CCER_CC1P | GTIM_CCER_CC1NP;
				break;

			default:
				return -EINVAL;;
			}

			uint32_t timer = timer_io_channels[channel].timer_index;
			uint16_t rvalue;
			rv = OK;

			irqstate_t flags = enter_critical_section();

			switch (timer_io_channels[channel].timer_channel) {

			case 1:
			case 2:
			case 3:
			case 4:
			default:
				UNUSED(rvalue);
				UNUSED(timer);
				UNUSED(edge_bits);
				rv = -EIO;
			}

			leave_critical_section(flags);
		}
	}

	return rv;
}

int up_input_capture_get_callback(unsigned channel, capture_callback_t *callback, void **context)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {

			irqstate_t flags = enter_critical_section();
			*callback = channel_handlers[channel].callback;
			*context = channel_handlers[channel].context;
			leave_critical_section(flags);
			rv = OK;
		}
	}

	return rv;

}

int up_input_capture_set_callback(unsigned channel, capture_callback_t callback, void *context)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {

		rv = -ENXIO;

		/* Any pins in capture mode */

		if (io_timer_get_channel_mode(channel) == IOTimerChanMode_Capture) {
			input_capture_bind(channel, callback, context);
			rv = 0;
		}
	}

	return rv;
}

int up_input_capture_get_stats(unsigned channel, input_capture_stats_t *stats, bool clear)
{
	int rv = io_timer_validate_channel_index(channel);

	if (rv == 0) {
		irqstate_t flags = enter_critical_section();
		*stats =  channel_stats[channel];

		if (clear) {
			memset(&channel_stats[channel], 0, sizeof(*stats));
		}

		leave_critical_section(flags);
	}

	return rv;
}
