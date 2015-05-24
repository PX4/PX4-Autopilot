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
 * @file LidarLitePWM.h
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via PWM.
 *
 * This driver accesses the pwm_input published by the pwm_input driver.
 */
#pragma once

#include "LidarLite.h"

#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <drivers/device/ringbuffer.h>
#include <systemlib/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/distance_sensor.h>



class LidarLitePWM : public LidarLite, public device::CDev
{
public:
	LidarLitePWM(const char *path);
	virtual ~LidarLitePWM();

	int init() override;

	ssize_t read(struct file *filp, char *buffer, size_t buflen) override;
	int	ioctl(struct file *filp, int cmd, unsigned long arg);

	void start() override;

	void stop() override;

	void cycle();

	/**
	* @brief
	*   Diagnostics - print some basic information about the driver.
	*/
	void print_info() override;

	/**
	 * @brief
	 *   print registers to console
	 */
	void print_registers() override;

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg        Instance pointer for the driver that is polling.
	*/
	static void     cycle_trampoline(void *arg);

protected:

	int measure() override;

	int collect() override;

	int reset_sensor() override;

	void task_main_trampoline(int argc, char *argv[]);

private:
	work_s			_work;
	ringbuffer::RingBuffer	*_reports;
	int			_class_instance;
	int			_orb_class_instance;
	int			_pwmSub;
	struct pwm_input_s	_pwm;
	orb_advert_t	        _distance_sensor_topic;
	struct distance_sensor_s _range;

	perf_counter_t	        _sample_perf;
	perf_counter_t	        _read_errors;
	perf_counter_t	        _buffer_overflows;
	perf_counter_t	        _sensor_zero_resets;
};
