/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file LidarLite.h
 * @author Johan Jansen <jnsn.johan@gmail.com>
 *
 * Generic interface driver for the PulsedLight Lidar-Lite range finders.
 */
#pragma once

#include <drivers/device/device.h>
#include <drivers/drv_range_finder.h>

/* Device limits */
#define LL40LS_MIN_DISTANCE (0.00f)
#define LL40LS_MAX_DISTANCE (60.00f)

// normal conversion wait time
#define LL40LS_CONVERSION_INTERVAL 50*1000UL /* 50ms */

// maximum time to wait for a conversion to complete.
#define LL40LS_CONVERSION_TIMEOUT 100*1000UL /* 100ms */

class LidarLite
{
public:
	LidarLite();

	virtual ~LidarLite();

	virtual int init() = 0;

	virtual int ioctl(struct file *filp, int cmd, unsigned long arg);

	virtual void start() = 0;

	virtual void stop() = 0;

	/**
	* @brief
	*   Diagnostics - print some basic information about the driver.
	*/
	virtual void print_info() = 0;

	/**
	 * @brief
	 *   print registers to console
	 */
	virtual void print_registers() = 0;

protected:
	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults LL40LS_MIN_DISTANCE
	* and LL40LS_MAX_DISTANCE
	*/
	void                set_minimum_distance(const float min);
	void                set_maximum_distance(const float max);
	float               get_minimum_distance() const;
	float               get_maximum_distance() const;

	uint32_t            getMeasureTicks() const;

	virtual int         measure() = 0;
	virtual int         collect() = 0;

	virtual int         reset_sensor() = 0;

private:
	float               _min_distance;
	float               _max_distance;
	uint32_t            _measure_ticks;
};
