/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: 	James Goppert
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
 * @file RoboClawDevice.h
 * RoboClaw Driver
 *
 * @author James Goppert
 */

#pragma once

// system includes
#include <inttypes.h> // uint16_t
#include <stdlib.h> // strtoul
#include <string.h> // strcmp
#include <unistd.h> // usleep
#include <sched.h> // task_create
#include <fcntl.h> // open
#include <termios.h> // tcgetattr etc.
#include <math.h> // fabs
#include <stdio.h> // snprintf

// px4 includes
#include <systemlib/err.h> // errx
#include <systemlib/systemlib.h> // SCHED_DEFAULT
#include <systemlib/scheduling_priorities.h> // SCHED_PRIORITY
#include <drivers/device/device.h> // device::CDev
#include <drivers/drv_sensor.h> // ioctl flags
#include <drivers/drv_hrt.h> // hrt_absolute_time
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <mavlink/mavlink_log.h>

// topics
#include <modules/uORB/topics/actuator_outputs.h>
#include <modules/uORB/topics/actuator_controls.h>
#include <modules/uORB/topics/encoders.h>

// roboclaw
#include "RoboClaw.h"

class RoboClawDevice : public device::CDev
{
public:
	// error flags
	enum e_error {
		ROBO_ERROR_NONE = 0,
		ROBO_ERROR_M1_OVERCURRENT = 1 << 0,
		ROBO_ERROR_M2_OVERCURRENT = 1 << 1,
		ROBO_ERROR_ESTOP = 1 << 2,
		ROBO_ERROR_TEMPERATURE = 1 << 3,
		ROBO_ERROR_MAIN_BATTERY_HIGH = 1 << 4,
		ROBO_ERROR_MAIN_BATTERY_LOW = 1 << 5,
		ROBO_ERROR_LOGIC_BATTERY_HIGH = 1 << 6,
		ROBO_ERROR_LOGIC_BATTERY_LOW = 1 << 7,
	};
	// quadrature status flags
	enum e_quadrature_status_flags {
		ROBO_ENCODER_UNDERFLOW = 1 << 0, /**< encoder went below 0 **/
		ROBO_ENCODER_REVERSE = 1 << 1, /**< motor doing in reverse dir **/
		ROBO_ENCODER_OVERFLOW = 1 << 2, /**< encoder went above 2^32 **/
	};
	RoboClawDevice(const char *port, uint8_t address,
		       uint32_t tout, bool doack);
	virtual ~RoboClawDevice();
	virtual int init();
	void update();
	static void errorToString(uint8_t error,
				  char *msg, size_t n);
	int64_t encoderToInt64(uint32_t count, uint8_t status,
			       int32_t *overflows);
private:
	RoboClaw m_roboclaw;
	uORB::Subscription<actuator_controls_s> m_controls;
	uORB::Publication<actuator_outputs_s> m_outputs;
	uORB::Publication<encoders_s> m_encoders;
	int m_mavlink_fd;
	uint8_t m_address;
	uint32_t m_period;
	uint8_t m_error;
	uint64_t m_timeErrorSent;
	uint64_t m_timeActuatorCommand;
	uint64_t m_timeUpdate;
	int32_t m_motor1Overflows;
	int32_t m_motor2Overflows;
	int16_t m_accel;
};

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
