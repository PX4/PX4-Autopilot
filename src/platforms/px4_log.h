/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
 * @file px4_log_os_impl.h
 * Platform dependant logging/debug implementation
 */

#pragma once

#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include <stdio.h>

__BEGIN_DECLS
__EXPORT extern uint64_t hrt_absolute_time(void);
//__EXPORT extern unsigned long pthread_self();

#define _PX4_LOG_LEVEL_ALWAYS		0
#define _PX4_LOG_LEVEL_PANIC		1
#define _PX4_LOG_LEVEL_ERROR		2
#define _PX4_LOG_LEVEL_WARN		3
#define _PX4_LOG_LEVEL_DEBUG		4

extern const char *__px4_log_level_str[5];
extern unsigned int __px4_log_level_current;

#define PX4_LOG_LEVEL_AT_RUN_TIME	_PX4_LOG_LEVEL_WARN

#define _PX4_LOG_LEVEL_STR(level)	__px4_log_level_str[level];

/****************************************************************************
 * Implementation of log section formatting based on printf
 ****************************************************************************/
#if defined(__PX4_ROS)
#define __px4__log_startline(level)	if (level <= __px4_log_level_current) ROS_WARN(
#else
#define __px4__log_startline(level)	if (level <= __px4_log_level_current) printf(
#endif
#define __px4__log_timestamp_fmt	"%-10" PRIu64 
#define __px4__log_timestamp_arg 	,hrt_absolute_time()
#define __px4__log_level_fmt		"%-5s "
#define __px4__log_level_arg(level)	,__px4_log_level_str[level]
#define __px4__log_thread_fmt		"%ld "
#define __px4__log_thread_arg		,pthread_self()

#define __px4__log_file_and_line_fmt 	" (file %s line %d)"
#define __px4__log_file_and_line_arg 	, __FILE__, __LINE__
#define __px4__log_end_fmt 		"\n"
#define __px4__log_endline 		)

/****************************************************************************
 * Output format macros
 * Use these to implement the code level macros below
 ****************************************************************************/
#define __px4_log_omit(level, FMT, ...)   { }

#define __px4_log(level, FMT, ...) \
	__px4__log_startline(level)\
	__px4__log_level_fmt \
	FMT\
	__px4__log_end_fmt \
	__px4__log_level_arg(level), ##__VA_ARGS__\
	__px4__log_endline

#define __px4_log_timestamp(level, FMT, ...) \
	__px4__log_startline(level)\
	__px4__log_timestamp_fmt\
	__px4__log_level_fmt\
	FMT\
	__px4__log_end_fmt\
	__px4__log_timestamp_arg\
	__px4__log_level_arg(level), ##__VA_ARGS__\
	__px4__log_endline

#define __px4_log_file_and_line(level, FMT, ...) \
	__px4__log_startline(level)\
	__px4__log_timestamp_fmt\
	__px4__log_level_fmt\
	FMT\
	__px4__log_file_and_line_fmt\
	__px4__log_end_fmt\
	__px4__log_timestamp_arg\
	__px4__log_level_arg(level), ##__VA_ARGS__\
	__px4__log_file_and_line_arg\
	__px4__log_endline

#define __px4_log_timestamp_file_and_line(level, FMT, ...) \
	__px4__log_startline(level)\
	__px4__log_timestamp_fmt\
	__px4__log_level_fmt\
	FMT\
	__px4__log_file_and_line_fmt\
	__px4__log_end_fmt\
	__px4__log_timestamp_arg\
	__px4__log_level_arg(level) , ##__VA_ARGS__\
	__px4__log_file_and_line_arg\
	__px4__log_endline

#define __px4_log_thread_file_and_line(level, FMT, ...) \
	__px4__log_startline(level)\
	__px4__log_thread_fmt\
	__px4__log_level_fmt\
	FMT\
	__px4__log_file_and_line_fmt\
	__px4__log_end_fmt\
	__px4__log_thread_arg\
	__px4__log_level_arg(level) , ##__VA_ARGS__\
	__px4__log_file_and_line_arg\
	__px4__log_endline


/****************************************************************************
 * Code level macros
 * These are the log APIs that should be used by the code
 ****************************************************************************/
#define PX4_LOG(FMT, ...) 	__px4_log(_PX4_LOG_LEVEL_ALWAYS, FMT, __VA_ARGS__)

#if defined(DEBUG_BUILD)

#define PX4_PANIC(FMT, ...)	__px4_log_timestamp_file_and_line(_PX4_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define PX4_ERR(FMT, ...)	__px4_log_timestamp_file_and_line(_PX4_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define PX4_WARN(FMT, ...) 	__px4_log_timestamp_file_and_line(_PX4_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define PX4_DEBUG(FMT, ...) 	__px4_log_timestamp(_PX4_LOG_LEVEL_DEBUG, FMT, __VA_ARGS__)

#else

#define PX4_PANIC(FMT, ...)	__px4_log_file_and_line(_PX4_LOG_LEVEL_PANIC, FMT, ##__VA_ARGS__)
#define PX4_ERR(FMT, ...)	__px4_log_file_and_line(_PX4_LOG_LEVEL_ERROR, FMT, ##__VA_ARGS__)
#define PX4_WARN(FMT, ...) 	__px4_log_file_and_line(_PX4_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define PX4_INFO(FMT, ...) 	__px4_log(_PX4_LOG_LEVEL_WARN,  FMT, ##__VA_ARGS__)
#define PX4_DEBUG(FMT, ...) 	__px4_log_omit(_PX4_LOG_LEVEL_DEBUG, FMT, ##__VA_ARGS__)

#endif
__END_DECLS
