/****************************************************************************
 *
 *   Copyright (C) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file ToneAlarm.h
 *
 * Low Level Driver for the PX4 audio alarm port. Subscribes to
 * tune_control and plays notes on this architecture specific timer HW.
 */

#pragma once

#include <circuit_breaker/circuit_breaker.h>
#include <drivers/device/device.h>
#include <drivers/drv_tone_alarm.h>
#include <lib/tunes/tunes.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/tune_control.h>

#include <string.h>

#if !defined(UNUSED)
#  define UNUSED(a) ((void)(a))
#endif

class ToneAlarm : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	ToneAlarm();
	~ToneAlarm();

	/**
	 * @brief Initializes the character device and hardware registers.
	 */
	int init() override;

	/**
	 * @brief Prints the driver status to the console.
	 */
	void status();

protected:

	/**
	 * @brief Parses the next note out of the string and plays it.
	 */
	void next_note();

	/**
	 * @brief Trampoline for the work queue.
	 */
	void Run() override;

	/**
	 * @brief Updates the uORB topics for local subscribers.
	 */
	void orb_update();

	/**
	 * @brief Starts playing the note.
	 */
	void start_note(unsigned frequency);

	/**
	 * @brief Stops playing the current note and makes the player 'safe'.
	 */
	void stop_note();

	volatile bool _running{false};		///< Flag to indicate the current driver status.

	int _cbrk{CBRK_UNINIT};     		///< If true, no audio output.

private:

	volatile bool _should_run{true};

	bool _play_tone{false};

	unsigned int _silence_length{0};	///< If nonzero, silence before next note.

	uORB::Subscription _tune_control_sub{ORB_ID(tune_control)};

	tune_control_s _tune{};

	Tunes _tunes = Tunes();
};
