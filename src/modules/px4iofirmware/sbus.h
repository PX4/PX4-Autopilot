/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file sbus.h
 *
 * S.BUS / S.BUS2 protocol decoder / encoder
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <drivers/drv_hrt.h>

__BEGIN_DECLS

#define SBUS_INTER_FRAME_TIMEOUT	3000	/**< 3000 us frame timeout */
#define SBUS_FRAME_SIZE			25
#define SBUS_BUFFER_SIZE		(SBUS_FRAME_SIZE + SBUS_FRAME_SIZE / 2)

int	sbus_init(const char *device);
bool	sbus_input(uint16_t *values, uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t *serial_frame_drops, uint16_t max_channels);
bool	sbus_parse(hrt_abstime now, uint8_t *frame, unsigned *partial_count, uint16_t *values,
	uint16_t *num_values, bool *sbus_failsafe, bool *sbus_frame_drop, uint16_t *serial_frame_drops, uint16_t max_channels);

void	sbus1_output(uint16_t *values, uint16_t num_values);
void	sbus2_output(uint16_t *values, uint16_t num_values);

__END_DECLS
