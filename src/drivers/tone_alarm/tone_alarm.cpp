/****************************************************************************
 *
 *   Copyright (C) 2013, 2016, 2018 PX4 Development Team. All rights reserved.
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
 * @file tone_alarm.cpp
 *
 * Base Class for the PX4 audio alarm port low level drivers.
 * Subscribes to tune_control and plays notes on an architecture
 * specific timer HW.
 */

#include "tone_alarm.h"

ToneAlarm::ToneAlarm() :
	ModuleParams(nullptr)
{
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
}

ToneAlarm::~ToneAlarm()
{
	_should_run = false;
	int counter = 0;

	while (_running && counter < 10) {
		// Allow sufficient time to stop the driver.
		counter++;
		px4_usleep(100000);
	}
}

int ToneAlarm::custom_command(int argc, char *argv[])
{
	// Check if the driver is running.
	if (!is_running() && !_object) {
		PX4_INFO("Driver not running");
		return PX4_ERROR;
	}

	const char *arg_v = argv[0];

	if (strcmp(arg_v, "cycle") == 0 ||
	    strcmp(arg_v, "next_note") == 0) {
		get_instance()->cycle();
		return PX4_OK;
	}

	get_instance()->print_usage("Unrecognized command");
	return PX4_OK;
}

void ToneAlarm::cycle()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	// Is there an inter-note gap to wait for?
	if (_silence_length > 0) {
		stop_note();
		work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::cycle_trampoline, this, USEC2TICK(_silence_length));
		_silence_length = 0;
		return;
	}

	// Check for updates.
	bool updated = false;
	orb_check(_tune_control_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(tune_control), _tune_control_sub, &_tune);
		_play_tone = _tunes.set_control(_tune) == 0;
	}

	unsigned int frequency = 0;
	unsigned int duration = 0;

	if (_play_tone) {
		_play_tone = false;
		int parse_ret_val = _tunes.get_next_tune(frequency, duration, _silence_length);

		if (parse_ret_val >= 0) {
			// A frequency of 0 correspond to stop_note.
			if (frequency > 0) {
				// Start playing the note.
				start_note(frequency);

			} else {
				stop_note();
			}

			if (parse_ret_val > 0) {
				// Continue playing.
				_play_tone = true;
			}
		}

	} else {
		// Schedule a call with the tunes max interval.
		duration = _tunes.get_maximum_update_interval();
		// Stop playing the last note after the duration elapsed.
		stop_note();
	}

	// Schedule a callback when the note should stop.
	work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::cycle_trampoline, this, USEC2TICK(duration));
}

void ToneAlarm::cycle_trampoline(void *argv)
{
	ToneAlarm *toneAlarm = reinterpret_cast<ToneAlarm *>(argv);
	toneAlarm->cycle();
}

unsigned ToneAlarm::frequency_to_divisor(unsigned frequency)
{
	float period = 0.5f / frequency;

	// and the divisor, rounded to the nearest integer.
	unsigned int divisor = (period * get_tone_alarm_clock()) + 0.5f;
	return divisor;
}

void ToneAlarm::initialize_subscriptions()
{
	// Subscribe to tune_control.
	if (_tune_control_sub < 0) {
		_tune_control_sub = orb_subscribe(ORB_ID(tune_control));
	}
}

void ToneAlarm::initialize_trampoline(void *argv)
{
	ToneAlarm *toneAlarm = new ToneAlarm();

	if (!toneAlarm) {
		PX4_ERR("Driver allocation failed");
		return;
	}

	_object = toneAlarm;
	toneAlarm->start();

	toneAlarm->initialize_registers();
}

int ToneAlarm::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Background process running periodically on the HP work queue to play audio tones and alarms.

This task can be started via boot script or CLI by invoking "tone_alarm".
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tone_alarm", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Starts the driver.");
	// PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Reports the driver on/off status.");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stops the driver.");
	return 0;
}

int ToneAlarm::start()
{
	if (is_running()) {
		PX4_INFO("Driver already running");
		return PX4_ERROR;
	}

	update_params(true);
	initialize_subscriptions();

	// Advancing the cycle can be called directly in the current context of the work queue.
	cycle();

	PX4_INFO("Driver started successfully");
	return PX4_OK;
}

void ToneAlarm::start_note(unsigned frequency)
{
	// Check if circuit breaker is enabled.
	if (_cbrk == CBRK_UNINIT) {
		_cbrk = circuit_breaker_enabled("CBRK_BUZZER", CBRK_BUZZER_KEY);
	}

	if (_cbrk != CBRK_OFF) {
		return;
	}

	// Compute the divisor.
	_divisor = frequency_to_divisor(frequency);

	// Pick the lowest useable prescaler value.
	// (Note that the effective prescale value is 1 greater.)
	_prescale = _divisor / 65536;

	// Calculate the timer period for the selected prescaler value.
	_period = (_divisor / (_prescale + 1)) - 1;

	activate_registers();
}

void ToneAlarm::stop_note()
{
	// Reset the registers to stop the current note.
	deactivate_registers();
}

int ToneAlarm::task_spawn(int argc, char *argv[])
{
	int result = work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::initialize_trampoline, nullptr, 0);

	if (result < 0) {
		return result;
	}

	result = wait_until_running();

	if (result < 0) {
		return result;
	}

	_task_id = task_id_is_work_queue;
	return 0;
}

void ToneAlarm::update_params(const bool force)
{
	bool updated = false;
	parameter_update_s param_update;

	orb_check(_params_sub, &updated);

	if (updated || force) {
		ModuleParams::updateParams();
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
	}
}

struct work_s ToneAlarm::_work = {};

/**
 * Tone alarm Driver 'main' command.
 * Entry point for the tone_alarm driver module.
 */
extern "C" __EXPORT int tone_alarm_main(int argc, char *argv[])
{
	return ToneAlarm::main(argc, argv);
};
