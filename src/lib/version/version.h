/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Anton Babushkin <anton.babushkin@me.com>
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
 * @file version.h
 *
 * Tools for system version detection.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#pragma once


#define FREEZE_STR(s) #s
#define STRINGIFY(s) FREEZE_STR(s)
#define FW_GIT STRINGIFY(GIT_VERSION)
#define FW_BUILD_URI STRINGIFY(BUILD_URI)

/* The preferred method for publishing a board name is to
 * define it in board_config.h as BOARD_NAME
 */
#if defined(CONFIG_ARCH_BOARD_SITL)
# define BOARD_NAME "SITL"
#elif defined(CONFIG_ARCH_BOARD_EAGLE)
# define BOARD_NAME "EAGLE"
#elif defined(CONFIG_ARCH_BOARD_EXCELSIOR)
# define BOARD_NAME "EXCELSIOR"
#elif defined(CONFIG_ARCH_BOARD_RPI)
# define BOARD_NAME "RPI"
#elif defined(CONFIG_ARCH_BOARD_BEBOP)
# define BOARD_NAME "BEBOP"
#else
# include "board_config.h"
# ifndef BOARD_NAME
#  error "board_config.h must define BOARD_NAME"
# endif
#endif


__BEGIN_DECLS

/**
 * get the board name as string (including the version if there are multiple)
 */
static inline const char *px4_board_name(void)
{
	return BOARD_NAME;
}



__END_DECLS

