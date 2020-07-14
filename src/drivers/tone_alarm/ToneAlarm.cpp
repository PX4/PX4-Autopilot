/****************************************************************************
 *
 *   Copyright (C) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ToneAlarm.cpp
 */

#include "ToneAlarm.h"

#include <px4_platform_common/time.h>

ToneAlarm::ToneAlarm() :
	CDev(TONE_ALARM0_DEVICE_PATH),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

ToneAlarm::~ToneAlarm()
{
	_should_run = false;
	int counter = 0;

	while (_running && counter < 10) {
		px4_usleep(100000);
		counter++;
	}
}

int ToneAlarm::init()
{
	if (CDev::init() != OK) {
		return PX4_ERROR;
	}

	// NOTE: Implement hardware specific detail in the ToneAlarmInterface class implementation.
	ToneAlarmInterface::init();

	_running = true;

	ScheduleNow();

	return OK;
}

void ToneAlarm::next_note()
{
	if (!_should_run) {
		_running = false;
		return;
	}

	// Check for updates
	orb_update();

	unsigned int frequency = 0;
	unsigned int duration = 0;

	// Does an inter-note silence occur?
	if (_silence_length > 0) {
		stop_note();
		duration = _silence_length;
		_silence_length = 0;

	} else if (_play_tone) {
		int parse_ret_val = _tunes.get_next_note(frequency, duration, _silence_length);

		if (parse_ret_val > 0) {
			// Continue playing.
			_play_tone = true;

			// A frequency of 0 corresponds to stop_note();
			if (frequency > 0) {
				// Start playing the note.
				start_note(frequency);
			}

		} else {
			_play_tone = false;
			stop_note();
		}

	} else {
		// Schedule a callback with the tunes max interval.
		duration = _tunes.get_maximum_update_interval();
		stop_note();
	}

	// Schedule a callback when the note should stop.
	ScheduleDelayed(duration);
}

void ToneAlarm::Run()
{
	next_note();
}

void ToneAlarm::orb_update()
{
	// Check for updates
	if (_tune_control_sub.updated()) {
		_tune_control_sub.copy(&_tune);

		if (_tune.timestamp > 0) {
			_play_tone = _tunes.set_control(_tune) == 0;
		}
	}
}

void ToneAlarm::status()
{
	if (_running) {
		PX4_INFO("running");

	} else {
		PX4_INFO("stopped");
	}
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

	// NOTE: Implement hardware specific detail in the ToneAlarmInterface class implementation.
	ToneAlarmInterface::start_note(frequency);
}

void ToneAlarm::stop_note()
{
	// NOTE: Implement hardware specific detail in the ToneAlarmInterface class implementation.
	ToneAlarmInterface::stop_note();
}

/**
 * Local functions in support of the shell command.
 */
namespace
{

ToneAlarm *g_dev;

} // namespace

/**
 * Tone alarm Driver 'main' command.
 * Entry point for the tone_alarm driver module.
 */
extern "C" __EXPORT int tone_alarm_main(int argc, char *argv[])
{
	if (argc > 1) {
		const char *argv1 = argv[1];

		if (!strcmp(argv1, "start")) {
			if (g_dev == nullptr) {
				g_dev = new ToneAlarm();

				if (g_dev == nullptr) {
					PX4_ERR("could not allocate the driver.");
				}

				if (g_dev->init() != OK) {
					delete g_dev;
					g_dev = nullptr;
					PX4_ERR("driver init failed.");
				}

			} else {
				PX4_INFO("already started");
			}

			return 0;
		}

		if (!strcmp(argv1, "stop")) {
			delete g_dev;
			g_dev = nullptr;
			return 0;
		}

		if (!strcmp(argv1, "status")) {
			if (g_dev != nullptr) {
				g_dev->status();

			} else {
				PX4_INFO("driver stopped");
			}

			return 0;
		}

	} else {
		PX4_INFO("missing command, try 'start', status, 'stop'");
	}

	return 0;
};
