/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: @author Lorenz Meier <lm@inf.ethz.ch>
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
 * @file mavlink_log.h
 * MAVLink text logging.
 */

#ifndef MAVLINK_LOG
#define MAVLINK_LOG

/*
 * IOCTL interface for sending log messages.
 */
#include <sys/ioctl.h>

/*
 * The mavlink log device node; must be opened before messages
 * can be logged.
 */
#define MAVLINK_LOG_DEVICE			"/dev/mavlink"

#define MAVLINK_IOC_SEND_TEXT_INFO		_IOC(0x1100, 1)
#define MAVLINK_IOC_SEND_TEXT_CRITICAL		_IOC(0x1100, 2)
#define MAVLINK_IOC_SEND_TEXT_EMERGENCY		_IOC(0x1100, 3)

/**
 * Send a mavlink emergency message.
 *
 * @param _fd		A file descriptor returned from open(MAVLINK_LOG_DEVICE, 0);
 * @param _text		The text to log;
 */
#define mavlink_log_emergency(_fd, _text)	ioctl(_fd, MAVLINK_IOC_SEND_TEXT_EMERGENCY, (unsigned long)_text);

/**
 * Send a mavlink critical message.
 *
 * @param _fd		A file descriptor returned from open(MAVLINK_LOG_DEVICE, 0);
 * @param _text		The text to log;
 */
#define mavlink_log_critical(_fd, _text)	ioctl(_fd, MAVLINK_IOC_SEND_TEXT_CRITICAL, (unsigned long)_text);

/**
 * Send a mavlink info message.
 *
 * @param _fd		A file descriptor returned from open(MAVLINK_LOG_DEVICE, 0);
 * @param _text		The text to log;
 */
#define mavlink_log_info(_fd, _text)		ioctl(_fd, MAVLINK_IOC_SEND_TEXT_INFO, (unsigned long)_text);

#endif

