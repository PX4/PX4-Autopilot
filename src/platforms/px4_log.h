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
 * @file px4_log.h
 * Platform dependant logging/debug
 */

#pragma once

#if defined(__PX4_QURT)

__BEGIN_DECLS
extern void qurt_log(const char *fmt, ...);
__END_DECLS

#include <stdio.h>

#define qurt_log(...)   {printf(__VA_ARGS__); printf("\n");}
#define PX4_DBG(...)	qurt_log(__VA_ARGS__)
#define PX4_DEBUG(...)	qurt_log(__VA_ARGS__)
#define PX4_INFO(...) 	qurt_log(__VA_ARGS__)
#define PX4_WARN(...) 	qurt_log(__VA_ARGS__)
#define PX4_ERR(...)	{ qurt_log("ERROR file %s line %d:", __FILE__, __LINE__); qurt_log(__VA_ARGS__); }

#elif defined(__PX4_LINUX)
#include <stdio.h>

#define linux_log(level, ...)   { \
	printf("%-5s ", level);\
	printf(__VA_ARGS__);\
	printf("\n");\
}
#define linux_log_verbose(level, ...)   { \
	printf("%-5s ", level);\
	printf(__VA_ARGS__);\
	printf(" (file %s line %d)\n", __FILE__, __LINE__);\
}
//#define PX4_DEBUG(...)	{ }
#define PX4_DEBUG(...) 	{ linux_log("DEBUG", __VA_ARGS__); }
#define PX4_INFO(...) 	{ linux_log("INFO",  __VA_ARGS__); }
#define PX4_WARN(...) 	{ linux_log_verbose("WARN",  __VA_ARGS__); }
#define PX4_ERR(...)	{ linux_log_verbose("ERROR", __VA_ARGS__); }

#elif defined(__PX4_ROS)

#define PX4_DBG(...) 
#define PX4_INFO(...)	ROS_WARN(__VA_ARGS__)
#define PX4_WARN(...) 	ROS_WARN(__VA_ARGS__)
#define PX4_ERR(...) 	ROS_WARN(__VA_ARGS__)

#elif defined(__PX4_NUTTX)

#include <systemlib/err.h>

#define PX4_DBG(...) 
#define PX4_INFO(...)	warnx(__VA_ARGS__)
#define PX4_WARN(...)	warnx(__VA_ARGS__)
#define PX4_ERR(...) 	warnx(__VA_ARGS__)

#else

#define PX4_DBG(...) 
#define PX4_WARN(...) 
#define PX4_INFO(...) 
#define PX4_ERR(...) 

#endif
