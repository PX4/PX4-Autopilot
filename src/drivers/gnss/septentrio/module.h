/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file module.h
 *
 * Module functionality for the Septentrio GNSS driver.
 *
 * @author Thomas Frans
*/

#pragma once

#include <px4_platform_common/log.h>

#ifdef DEBUG_BUILD
#ifndef SEP_LOG_ERROR
#define SEP_LOG_ERROR
#endif
#ifndef SEP_LOG_WARN
#define SEP_LOG_WARN
#endif
#ifndef SEP_LOG_INFO
#define SEP_LOG_INFO
#endif
#endif

#ifdef SEP_LOG_ERROR
#define SEP_ERR(...)            {PX4_WARN(__VA_ARGS__);}
#else
#define SEP_ERR(...)            {}
#endif

#ifdef SEP_LOG_WARN
#define SEP_WARN(...)           {PX4_WARN(__VA_ARGS__);}
#else
#define SEP_WARN(...)           {}
#endif

#ifdef SEP_LOG_INFO
#define SEP_INFO(...)           {PX4_INFO(__VA_ARGS__);}
#else
#define SEP_INFO(...)           {}
#endif

#ifdef SEP_LOG_TRACE_PARSING
#define SEP_TRACE_PARSING(...)  {PX4_DEBUG(__VA_ARGS__);}
#else
#define SEP_TRACE_PARSING(...)  {}
#endif
