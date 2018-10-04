/****************************************************************************
 *
 *   Copyright (c) 2013, 2017-2018 PX4 Development Team. All rights reserved.
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

/*
 * Simulated Low Level Driver for the PX4 audio alarm port. Subscribes to
 * tune_control and plays notes on this architecture specific
 * timer HW
 */

#include <circuit_breaker/circuit_breaker.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_tone_alarm.h>
#include <lib/cdev/CDev.hpp>
#include <lib/tunes/tunes.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_workqueue.h>
#include <uORB/topics/tune_control.h>

#if !defined(UNUSED)
#  define UNUSED(a) ((void)(a))
#endif

#define CBRK_BUZZER_KEY 782097

class ToneAlarm : public cdev::CDev
{
public:
	ToneAlarm();
	~ToneAlarm();

	virtual int init();
	void status();

	enum {
		CBRK_OFF = 0,
		CBRK_ON,
		CBRK_UNINIT
	};

private:
	volatile bool _running;
	volatile bool _should_run;
	bool _play_tone;

	Tunes _tunes;

	unsigned _silence_length; // if nonzero, silence before next note

	int _cbrk; ///< if true, no audio output
	int _tune_control_sub;

	tune_control_s _tune;

	static work_s _work;

	// Convert a frequency value into a divisor for the configured timer's clock.
	//
	unsigned frequency_to_divisor(unsigned frequency);

	// Start playing the note
	//
	void start_note(unsigned frequency);

	// Stop playing the current note and make the player 'safe'
	//
	void stop_note();

	// Parse the next note out of the string and play it
	//
	void next_note();

	// work queue trampoline for next_note
	//
	static void next_trampoline(void *arg);

	// Unused
	virtual void _measure() {}
};

struct work_s ToneAlarm::_work = {};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int tone_alarm_main(int argc, char *argv[]);


ToneAlarm::ToneAlarm() :
	CDev(TONEALARM0_DEVICE_PATH),
	_running(false),
	_should_run(true),
	_play_tone(false),
	_tunes(),
	_silence_length(0),
	_cbrk(CBRK_UNINIT),
	_tune_control_sub(-1)
{
}

ToneAlarm::~ToneAlarm()
{
	_should_run = false;
	int counter = 0;

	while (_running && ++counter < 10) {
		px4_usleep(100000);
	}
}

int ToneAlarm::init()
{
	int ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	_running = true;
	work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::next_trampoline, this, 0);
	return OK;
}

void ToneAlarm::status()
{
	if (_running) {
		PX4_INFO("running");

	} else {
		PX4_INFO("stopped");
	}
}

unsigned ToneAlarm::frequency_to_divisor(unsigned frequency)
{
	const int TONE_ALARM_CLOCK = 120000000ul / 4;

	float period = 0.5f / frequency;

	// and the divisor, rounded to the nearest integer
	unsigned divisor = (period * TONE_ALARM_CLOCK) + 0.5f;

	return divisor;
}

void ToneAlarm::start_note(unsigned frequency)
{
	// check if circuit breaker is enabled
	if (_cbrk == CBRK_UNINIT) {
		_cbrk = circuit_breaker_enabled("CBRK_BUZZER", CBRK_BUZZER_KEY);
	}

	if (_cbrk != CBRK_OFF) { return; }

	// compute the divisor
	unsigned divisor = frequency_to_divisor(frequency);

	// pick the lowest prescaler value that we can use
	// (note that the effective prescale value is 1 greater)
	unsigned prescale = divisor / 65536;

	// calculate the timer period for the selected prescaler value
	unsigned period = (divisor / (prescale + 1)) - 1;

	// Silence warning of unused var
	UNUSED(period);
	PX4_DEBUG("ToneAlarm::start_note %u", period);
}

void ToneAlarm::stop_note()
{
}

void ToneAlarm::next_note()
{
	if (!_should_run) {
		if (_tune_control_sub >= 0) {
			orb_unsubscribe(_tune_control_sub);
		}

		_running = false;
		return;
	}

	// subscribe to tune_control
	if (_tune_control_sub < 0) {
		_tune_control_sub = orb_subscribe(ORB_ID(tune_control));
	}

	// do we have an inter-note gap to wait for?
	if (_silence_length > 0) {
		stop_note();
		work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::next_trampoline, this, USEC2TICK(_silence_length));
		_silence_length = 0;
		return;
	}

	// check for updates
	bool updated = false;
	orb_check(_tune_control_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(tune_control), _tune_control_sub, &_tune);
	}

	unsigned frequency = 0;
	unsigned duration = 0;

	if (_play_tone) {
		_play_tone = false;
		int parse_ret_val = _tunes.get_next_tune(frequency, duration, _silence_length);

		if (parse_ret_val >= 0) {
			// a frequency of 0 correspond to stop_note
			if (frequency > 0) {
				// start playing the note
				start_note(frequency);

			} else {
				stop_note();
			}


			if (parse_ret_val > 0) {
				// continue playing
				_play_tone = true;
			}
		}

	} else {
		// schedule a call with the tunes max interval
		duration = _tunes.get_maximum_update_interval();
		// stop playing the last note after the duration elapsed
		stop_note();
	}

	// and arrange a callback when the note should stop
	work_queue(HPWORK, &_work, (worker_t)&ToneAlarm::next_trampoline, this, USEC2TICK(duration));
}

void ToneAlarm::next_trampoline(void *arg)
{
	ToneAlarm *ta = (ToneAlarm *)arg;
	ta->next_note();
}

/**
 * Local functions in support of the shell command.
 */
namespace
{

ToneAlarm	*g_dev;

} // namespace

void tone_alarm_usage();

void tone_alarm_usage()
{
	PX4_INFO("missing command, try 'start', status, 'stop'");
}

int tone_alarm_main(int argc, char *argv[])
{

	if (argc > 1) {
		const char *argv1 = argv[1];

		if (!strcmp(argv1, "start")) {
			if (g_dev != nullptr) {
				PX4_ERR("already started");
				return 1;
			}

			if (g_dev == nullptr) {
				g_dev = new ToneAlarm();

				if (g_dev == nullptr) {
					PX4_ERR("couldn't allocate the ToneAlarm driver");
					return 1;
				}

				if (OK != g_dev->init()) {
					delete g_dev;
					g_dev = nullptr;
					PX4_ERR("ToneAlarm init failed");
					return 1;
				}
			}

			return 0;
		}

		if (!strcmp(argv1, "stop")) {
			delete g_dev;
			g_dev = nullptr;
			return 0;
		}

		if (!strcmp(argv1, "status")) {
			g_dev->status();
			return 0;
		}

	}

	tone_alarm_usage();
	return 0;
}
