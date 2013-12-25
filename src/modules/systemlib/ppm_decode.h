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
 * @file ppm_decode.h
 *
 * PPM input decoder.
 */

#pragma once

#include <drivers/drv_hrt.h>

/**
 * Maximum number of channels that we will decode.
 */
#define PPM_MAX_CHANNELS	12

/* PPM input nominal min/max values */
#define PPM_MIN 1000
#define PPM_MAX 2000
#define PPM_MID ((PPM_MIN + PPM_MAX) / 2)

__BEGIN_DECLS

/*
 * PPM decoder state
 */
__EXPORT extern uint16_t	ppm_buffer[PPM_MAX_CHANNELS];	/**< decoded PPM channel values */
__EXPORT extern uint16_t	ppm_frame_length;				/**< length of the decoded PPM frame (includes gap) */
__EXPORT extern unsigned	ppm_decoded_channels;	/**< count of decoded channels */
__EXPORT extern hrt_abstime	ppm_last_valid_decode;	/**< timestamp of the last valid decode */

/**
 * Initialise the PPM input decoder.
 *
 * @param count_max		The maximum value of the counter passed to
 *				ppm_input_decode, used to determine how to
 *				handle counter wrap.
 */
__EXPORT void		ppm_input_init(unsigned count_max);

/**
 * Inform the decoder of an edge on the PPM input.
 *
 * This function can be registered with the HRT as the PPM edge handler.
 *
 * @param reset			If set, the edge detector has missed one or
 *				more edges and the decoder needs to be reset.
 * @param count			A microsecond timestamp corresponding to the
 *				edge, in the range 0-count_max.  This value
 *				is expected to wrap.
 */
__EXPORT void		ppm_input_decode(bool reset, unsigned count);

__END_DECLS