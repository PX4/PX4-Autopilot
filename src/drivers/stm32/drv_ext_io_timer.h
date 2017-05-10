/****************************************************************************
 *
 *   Copyright (C) 2012, 2017 PX4 Development Team. All rights reserved.
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
 * @file drv_io_timer.h
 *
 * stm32-specific PWM output data.
 */
#include <px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <drivers/drv_hrt.h>
#include "drv_io_timer.h"
#pragma once
__BEGIN_DECLS

/* supplied by board-specific code */
__EXPORT extern const io_timers_t ext_io_timers[MAX_IO_TIMERS];
__EXPORT extern const timer_io_channels_t ext_timer_io_channels[MAX_TIMER_IO_CHANNELS];

__EXPORT extern io_timer_channel_allocation_t allocations[IOTimerChanModeSize];
__EXPORT int ext_io_timer_handler0(int irq, void *context);
__EXPORT int ext_io_timer_handler1(int irq, void *context);
__EXPORT int ext_io_timer_handler2(int irq, void *context);
__EXPORT int ext_io_timer_handler3(int irq, void *context);

__EXPORT int ext_io_timer_channel_init(unsigned channel, io_timer_channel_mode_t mode,
				       channel_handler_t channel_handler, void *context);

__EXPORT int ext_io_timer_init_timer(unsigned timer);

__EXPORT int ext_io_timer_set_rate(unsigned timer, unsigned rate);
__EXPORT int ext_io_timer_set_enable(bool state, io_timer_channel_mode_t mode,
				     io_timer_channel_allocation_t masks);
__EXPORT uint16_t ext_io_channel_get_ccr(unsigned channel);
__EXPORT int ext_io_timer_set_ccr(unsigned channel, uint16_t value);
__EXPORT uint32_t ext_io_timer_get_group(unsigned timer);
__EXPORT int ext_io_timer_validate_channel_index(unsigned channel);
__EXPORT int ext_io_timer_is_channel_free(unsigned channel);
__EXPORT int ext_io_timer_free_channel(unsigned channel);
__EXPORT int ext_io_timer_get_channel_mode(unsigned channel);
__EXPORT int ext_io_timer_get_mode_channels(io_timer_channel_mode_t mode);
__EXPORT extern void ext_io_timer_trigger(void);

__END_DECLS
