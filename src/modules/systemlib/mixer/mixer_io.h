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
 * @file mixer_parameters.h
 *
 * Descripition of mixer parameters
 */


#ifndef _SYSTEMLIB_MIXER_IO_H
#define _SYSTEMLIB_MIXER_IO_H value

//#include "mixer_types.h"


#define MIXER_IO_MIXER_TYPE_COUNT 7

#ifndef MIXER_NONE_INPUT_COUNT
#define MIXER_NONE_INPUT_COUNT 0
#define MIXER_NONE_OUTPUT_COUNT 0
#endif

#ifndef MIXER_NULL_INPUT_COUNT
#define MIXER_NULL_INPUT_COUNT 1
#define MIXER_NULL_OUTPUT_COUNT 0
#endif

#ifndef MIXER_SIMPLE_INPUT_COUNT
#define MIXER_SIMPLE_INPUT_COUNT 0
#define MIXER_SIMPLE_OUTPUT_COUNT 1
#endif

#ifndef MIXER_MULTI_INPUT_COUNT
#define MIXER_MULTI_INPUT_COUNT 4
#define MIXER_MULTI_OUTPUT_COUNT 0
#endif

#ifndef MIXER_HELI_INPUT_COUNT
#define MIXER_HELI_INPUT_COUNT 4
#define MIXER_HELI_OUTPUT_COUNT 6
#endif

#ifndef MIXER_SIMPLE_INPUT_INPUT_COUNT
#define MIXER_SIMPLE_INPUT_INPUT_COUNT 1
#define MIXER_SIMPLE_INPUT_OUTPUT_COUNT 0
#endif

#ifndef MIXER_ROTOR_INPUT_COUNT
#define MIXER_ROTOR_INPUT_COUNT 0
#define MIXER_ROTOR_OUTPUT_COUNT 1
#endif


#ifndef MIXER_INPUT_COUNTS
#define MIXER_INPUT_COUNTS {MIXER_NONE_INPUT_COUNT, MIXER_NULL_INPUT_COUNT, MIXER_SIMPLE_INPUT_COUNT, MIXER_MULTI_INPUT_COUNT, MIXER_HELI_INPUT_COUNT, MIXER_SIMPLE_INPUT_INPUT_COUNT, MIXER_ROTOR_INPUT_COUNT}
#endif

#ifndef MIXER_OUTPUT_COUNTS
#define MIXER_OUTPUT_COUNTS {MIXER_NONE_OUTPUT_COUNT, MIXER_NULL_OUTPUT_COUNT, MIXER_SIMPLE_OUTPUT_COUNT, MIXER_MULTI_OUTPUT_COUNT, MIXER_HELI_OUTPUT_COUNT, MIXER_SIMPLE_INPUT_OUTPUT_COUNT, MIXER_ROTOR_OUTPUT_COUNT}
#endif


#endif
