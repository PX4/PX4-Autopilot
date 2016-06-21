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


#ifndef _SYSTEMLIB_MIXER_PARAMETERS_H
#define _SYSTEMLIB_MIXER_PARAMETERS_H value

#include "mixer_types.h"

#if defined(CONFIG_ARCH_BOARD_PX4IO_V1) || defined(CONFIG_ARCH_BOARD_PX4IO_V2) || defined(CONFIG_ARCH_BOARD_PX4FMU_V1) || defined(CONFIG_ARCH_BOARD_PX4FMU_V2)

extern const char* const* MIXER_PARAMETER_TABLE[MIXER_TYPE_COUNT];
extern const unsigned MIXER_PARAMETER_COUNTS[MIXER_TYPE_COUNT];


const char* const* MIXER_PARAMETER_TABLE[MIXER_TYPE_COUNT] = {nullptr};

const unsigned MIXER_PARAMETER_COUNTS[MIXER_TYPE_COUNT] = {0};

#else

extern const char* const* MIXER_PARAMETER_TABLE[MIXER_TYPE_COUNT];
extern const unsigned MIXER_PARAMETER_COUNTS[MIXER_TYPE_COUNT];


#define MIXER_NONE_PARAMETER_COUNT 0
const char* const MIXER_NONE_PARAMETERS[MIXER_NONE_PARAMETER_COUNT] = {};



#define MIXER_NULL_PARAMETER_COUNT 0
const char* const MIXER_NULL_PARAMETERS[MIXER_NULL_PARAMETER_COUNT] = {};


#define MIXER_SIMPLE_PARAMETER_COUNT 5
const char* const MIXER_SIMPLE_PARAMETERS[MIXER_SIMPLE_PARAMETER_COUNT] = {
    "SCALE_NEG",
    "SCALE_POS",
    "OFFSET",
    "OUTPUT_MIN",
    "OUTPUT_MAX",
    };


#define MIXER_MULTI_PARAMETER_COUNT 4
const char* const MIXER_MULTI_PARAMETERS[MIXER_MULTI_PARAMETER_COUNT] = {
    "ROLL_SCALE",
    "PITCH_SCALE",
    "YAW_SCALE",
    "IDLE_SPEED",
    };


const char* const* MIXER_PARAMETER_TABLE[MIXER_TYPE_COUNT] = {
    MIXER_NONE_PARAMETERS,
    MIXER_NULL_PARAMETERS,
    MIXER_SIMPLE_PARAMETERS,
    MIXER_MULTI_PARAMETERS
};

const unsigned MIXER_PARAMETER_COUNTS[MIXER_TYPE_COUNT] = {
    MIXER_NONE_PARAMETER_COUNT,
    MIXER_NULL_PARAMETER_COUNT,
    MIXER_SIMPLE_PARAMETER_COUNT,
    MIXER_MULTI_PARAMETER_COUNT
};

#endif //defined(CONFIG_ARCH_BOARD_PX4IO_V1) || defined(CONFIG_ARCH_BOARD_PX4IO_V2) || defined(CONFIG_ARCH_BOARD_PX4FMU_V1) || defined(CONFIG_ARCH_BOARD_PX4FMU_V2)



#endif
