/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file camera_capture.hpp
 *
 */

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdbool.h>
#include <poll.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <parameters/param.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_workqueue.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_input_capture.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#define PX4FMU_DEVICE_PATH	"/dev/px4fmu"

// For AV-X board
#define GPIO_TRIG_AVX /* PD14 */  (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN14)


class CameraCapture
{
public:
	/**
	 * Constructor
	 */
	CameraCapture();

	/**
	 * Destructor, also kills task.
	 */
	~CameraCapture();

	/**
	 * Start the task.
	 */
	int			start();

	/**
	 * Stop the task.
	 */
	void			stop();

	void 			status();

	void			cycle();

	static void		capture_trampoline(void *context, uint32_t chan_index,
			hrt_abstime edge_time, uint32_t edge_state,
			uint32_t overflow);

	void 			set_capture_control(bool enabled);

	void			reset_statistics(bool reset_seq);

	void			publish_trigger();


	static struct work_s	_work;
	static struct work_s	_work_publisher;

private:

	bool			_capture_enabled;
	bool			_gpio_capture;

	// Publishers
	orb_advert_t	_trigger_pub;
	orb_advert_t	_command_ack_pub;

	// Subscribers
	int				_command_sub;

	// Trigger Buffer
	struct _trig_s {
		uint32_t chan_index;
		hrt_abstime edge_time;
		uint32_t edge_state;
		uint32_t overflow;
	};

	struct _trig_s _trigger;

	ringbuffer::RingBuffer *_trig_buffer;

	// Parameters
	param_t 		_p_strobe_delay;
	float			_strobe_delay;
	param_t			_p_camera_capture_mode;
	int32_t			_camera_capture_mode;
	param_t			_p_camera_capture_edge;
	int32_t			_camera_capture_edge;

	// Signal capture statistics
	uint32_t		_capture_seq;
	hrt_abstime		_last_trig_begin_time;
	hrt_abstime		_last_exposure_time;
	hrt_abstime		_last_trig_time;
	uint32_t 		_capture_overflows;

	// Signal capture callback
	void			capture_callback(uint32_t chan_index,
			hrt_abstime edge_time, uint32_t edge_state, uint32_t overflow);

	// GPIO interrupt routine (for AV_X board)
	static int		gpio_interrupt_routine(int irq, void *context, void *arg);

	// Signal capture publish
	static void		publish_trigger_trampoline(void *arg);

	// Low-rate command handling loop
	static void		cycle_trampoline(void *arg);

};
struct work_s CameraCapture::_work;
struct work_s CameraCapture::_work_publisher;
