/****************************************************************************
 *
 *   Copyright (c) 2014-2019 PX4 Development Team. All rights reserved.
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
 * @file SF0X.hpp
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Greg Hulands
 *
 * Driver for the Lightware SF0x laser rangefinder series
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termios.h>

#include <perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/device.h>
#include <drivers/device/ringbuffer.h>
#include <lib/parameters/param.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include "sf0x_parser.h"

/* Configuration Constants */

#define SF0X_TAKE_RANGE_REG		'd'

// designated SERIAL4/5 on Pixhawk
#define SF0X_DEFAULT_PORT		"/dev/ttyS6"

class SF0X : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	SF0X(const char *port = SF0X_DEFAULT_PORT, uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
	virtual ~SF0X();

	virtual int 			init() override;

	virtual ssize_t			read(device::file_t *filp, char *buffer, size_t buflen) override;
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg) override;

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

private:
	char 				_port[20];
	uint8_t _rotation;
	float				_min_distance;
	float				_max_distance;
	int         		        _conversion_interval;
	ringbuffer::RingBuffer		*_reports;
	int				_measure_interval;
	bool				_collect_phase;
	int				_fd;
	char				_linebuf[10];
	unsigned			_linebuf_index;
	enum SF0X_PARSE_STATE		_parse_state;
	hrt_abstime			_last_read;

	int				_class_instance;
	int				_orb_class_instance;

	orb_advert_t			_distance_sensor_topic;

	unsigned			_consecutive_fail_count;

	perf_counter_t			_sample_perf;
	perf_counter_t			_comms_errors;

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults SF0X_MIN_DISTANCE
	* and SF0X_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				Run() override;
	int				measure();
	int				collect();

};
