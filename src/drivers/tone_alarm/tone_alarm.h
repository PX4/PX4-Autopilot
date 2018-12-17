/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file drv_tone_alarm.h
 *
 * Driver for the PX4 audio alarm port, /dev/tone_alarm.
 *
 * The tone_alarm driver supports a set of predefined "alarm"
 * patterns and one user-supplied pattern.  Patterns are ordered by
 * priority, with a higher-priority pattern interrupting any
 * lower-priority pattern that might be playing.
 *
 * The TONE_ALARM_SET_TUNE ioctl can be used to select a predefined
 * alarm pattern, from 1 - <TBD>.  Selecting pattern zero silences
 * the alarm.
 *
 * To supply a custom pattern, write an array of 1 - <TBD> tone_note
 * structures to /dev/tone_alarm.  The custom pattern has a priority
 * of zero.
 *
 * Patterns will normally play once and then silence (if a pattern
 * was overridden it will not resume).  A pattern may be made to
 * repeat by inserting a note with the duration set to
 * DURATION_REPEAT.  This pattern will loop until either a
 * higher-priority pattern is started or pattern zero is requested
 * via the ioctl.
 */

#pragma once

#include <circuit_breaker/circuit_breaker.h>
#include <lib/tunes/tunes.h>
#include <px4_config.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_workqueue.h>
#include <uORB/topics/parameter_update.h>


#define CBRK_BUZZER_KEY 782097
#define CBRK_OFF        0
#define CBRK_UNINIT     2

class ToneAlarm : public ModuleBase<ToneAlarm>, public ModuleParams
{
public:
	ToneAlarm();
	~ToneAlarm();

	/**
	 * @see ModuleBase::custom_command().
	 * @brief main Main entry point to the module that should be
	 *        called directly from the module's main method.
	 * @param argc The input argument count.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage().
	 * @brief Prints the module usage to the nuttshell console.
	 * @param reason The requested reason for printing to console.
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @see ModuleBase::task_spawn().
	 * @brief Initializes the class in the same context as the work queue
	 *        and starts the background listener.
	 * @param argv Pointer to the input argument array.
	 * @return Returns 0 iff successful, -1 otherwise.
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @brief Initiates the tone_alarm driver work queue. The task is started
	 *         as a new background task and continues if already running.
	 * @return Returns 1 iff start was successful.
	 */
	int start();

protected:

	/**
	 * @brief Called once to initialize uORB subscriptions.
	 */
	void initialize_subscriptions();

	/**
	 * @see ModuleBase::initialize_trampoline().
	 * @brief Trampoline initialization.
	 * @param argv Pointer to the task startup arguments.
	 */
	static void initialize_trampoline(void *argv);

private:

	/**
	 * @brief Configures hardware register values for activation.
	 */
	void activate_registers();

	/**
	 * @brief Calculates the heater element on/off time, carries out
	 *        closed loop feedback and feedforward temperature control,
	 *        and schedules the next cycle.
	 */
	void cycle();

	/**
	 * @brief Trampoline for the work queue.
	 * @param argv Pointer to the task startup arguments.
	 */
	static void cycle_trampoline(void *argv);

	/**
	 * @brief Configures hardware register values for deactivation.
	 */
	void deactivate_registers();

	/**
	 * @brief Converts a frequency value into a divisor for the configured timer's clock.
	 * @return Returns the divsor value.
	 */
	unsigned int frequency_to_divisor(unsigned frequency);

	/**
	 * @brief Obtains the hardware specific TONE_ALARM_CLOCK value.
	 * @return Returns the TONE_ALARM_CLOCK value.
	 */
	int get_tone_alarm_clock();

	/**
	 * @brief Initializes hardware registers.
	 */
	void initialize_registers();

	/**
	 * @brief Starts playing the note.
	 */
	void start_note(unsigned frequency);

	/**
	 * @brief Stops playing the current note and make the player 'safe'.
	 */
	void stop_note();

	/**
	 * @brief Updates and checks for updated uORB parameters.
	 * @param force Boolean to determine if an update check should be forced.
	 */
	void update_params(const bool force = false);

	volatile bool _running = false;

	volatile bool _should_run = true;

	int _cbrk = CBRK_UNINIT; ///< If true, no audio output.

	unsigned int _divisor = 0;

	int _params_sub = 0;

	unsigned int _period = 0;

	bool _play_tone = false;

	unsigned int _prescale = 0;

	unsigned int _silence_length = 0; ///< If nonzero, silence before next note.

	int _tune_control_sub = -1;

	tune_control_s _tune;

	Tunes _tunes = Tunes();

	static work_s _work;
};
