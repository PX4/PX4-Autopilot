/****************************************************************************
 *
 *   Copyright (C) 2013, 2018 PX4 Development Team. All rights reserved.
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
 * Low Level Driver for the PX4 audio alarm port. Subscribes to
 * tune_control and plays notes on this architecture specific
 * timer HW
 */

#include <px4_config.h>
#include <px4_log.h>
#include <debug.h>

#include <drivers/device/device.h>
#include <drivers/drv_tone_alarm.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <ctype.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <chip/sam_pinmap.h>
#include <sam_gpio.h>
#include <sam_tc.h>

#include <systemlib/err.h>
#include <circuit_breaker/circuit_breaker.h>

#include <px4_workqueue.h>

#include <lib/tunes/tunes.h>
#include <uORB/uORB.h>
#include <uORB/topics/tune_control.h>


/* Check that tone alarm and HRT timers are different */
#if defined(TONE_ALARM_CHANNEL)  && defined(HRT_TIMER_CHANNEL)
# if TONE_ALARM_CHANNEL == HRT_TIMER_CHANNEL
#   error TONE_ALARM_CHANNEL and HRT_TIMER_CHANNEL must use different timers.
# endif
#endif

/* Tone alarm configuration */
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/********************* TONE_ALARM_NOT_DONE ****************/
/******           This code is not finished 	***********/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
/**********************************************************/
#ifdef TONE_ALARM_CHANNEL

#if defined(TONE_ALARM_TIMER)
# error "TONE_ALARM_TIMER should not be defined, instead define TONE_ALARM_CHANNEL from 0-11"
#endif

#if TONE_ALARM_CHANNEL == 0 || TONE_ALARM_CHANNEL == 1 || TONE_ALARM_CHANNEL == 2
# define TONE_ALARM_TIMER 	0
#endif
#if TONE_ALARM_CHANNEL == 3 || TONE_ALARM_CHANNEL == 4 || TONE_ALARM_CHANNEL == 5
# define TONE_ALARM_TIMER 	1
#endif
#if TONE_ALARM_CHANNEL == 6 || TONE_ALARM_CHANNEL == 7 || TONE_ALARM_CHANNEL == 8
# define TONE_ALARM_TIMER 	2
#endif
#if TONE_ALARM_CHANNEL == 9 || TONE_ALARM_CHANNEL == 10 || TONE_ALARM_CHANNEL == 11
# define TONE_ALARM_TIMER 	3
#endif

/* HRT configuration */
#if   TONE_ALARM_TIMER == 0
# define HRT_TIMER_BASE			SAM_TC012_BASE
# if !defined(CONFIG_SAMV7_TC0)
#  error "HRT_TIMER_CHANNEL 0-2 Require CONFIG_SAMV7_TC0=y"
# endif
#elif TONE_ALARM_TIMER == 1
# define HRT_TIMER_BASE			SAM_TC345_BASE
# if !defined(CONFIG_SAMV7_TC1)
#  error "HRT_TIMER_CHANNEL 3-5 Require CONFIG_SAMV7_TC1=y"
# endif
#elif TONE_ALARM_TIMER == 2
# define HRT_TIMER_BASE			SAM_TC678_BASE
# if !defined(CONFIG_SAMV7_TC2)
#  error "HRT_TIMER_CHANNEL 6-8 Require CONFIG_SAMV7_TC2=y"
# endif
#elif TONE_ALARM_TIMER == 3
# define HRT_TIMER_BASE			SAM_TC901_BASE
# if !defined(CONFIG_SAMV7_TC3)
#  error "HRT_TIMER_CHANNEL 9-11 Require CONFIG_SAMV7_TC3=y"
# endif
#else
# error "HRT_TIMER_CHANNEL should be defined valid value are from 0-11"
# endif

#define TONE_ALARM_CLOCK (BOARD_MCK_FREQUENCY/128)


/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(SAM_TC0_BASE + SAM_TC_CHAN_OFFSET(HRT_TIMER_CHANNEL) + _reg))

#define rCCR	REG(SAM_TC_CCR_OFFSET)
#define rCMR 	REG(SAM_TC_CMR_OFFSET)
#define rSMMR 	REG(SAM_TC_SMMR_OFFSET)
#define rRAB 	REG(SAM_TC_RAB_OFFSET)
#define rCV 	REG(SAM_TC_CV_OFFSET)
#define rRA 	REG(SAM_TC_RA_OFFSET)
#define rRB 	REG(SAM_TC_RB_OFFSET)
#define rRC 	REG(SAM_TC_RC_OFFSET)
#define rSR 	REG(SAM_TC_SR_OFFSET)
#define rIER 	REG(SAM_TC_IER_OFFSET)
#define rIDR 	REG(SAM_TC_IDR_OFFSET)
#define rIMR 	REG(SAM_TC_IMR_OFFSET)
#define rEMR 	REG(SAM_TC_EMR_OFFSET)

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
	// enable debug() calls
	//_debug_enabled = true;
}

ToneAlarm::~ToneAlarm()
{
	_should_run = false;
	int counter = 0;

	while (_running && ++counter < 10) {
		usleep(100000);
	}
}

int ToneAlarm::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* configure the GPIO to the idle state */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);

#if TONE_ALARM_NOT_DONE	/* initialise the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = 0;
	rCCER &= TONE_CCER;		/* unlock CCMR* registers */
	rCCMR1 = TONE_CCMR1;
	rCCMR2 = TONE_CCMR2;
	rCCER = TONE_CCER;
	rDCR = 0;

#ifdef rBDTR // If using an advanced timer, you need to activate the output
	rBDTR = ATIM_BDTR_MOE; // enable the main output of the advanced timer
#endif

	/* toggle the CC output each time the count passes 1 */
	TONE_rCCR = 1;

	/* default the timer to a prescale value of 1; playing notes will change this */
	rPSC = 0;

	/* make sure the timer is running */
	rCR1 = GTIM_CR1_CEN;
#endif
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
#if TONE_ALARM_NOT_DONE
	rPSC = prescale;	// load new prescaler
	rARR = period;		// load new toggle period
	rEGR = GTIM_EGR_UG;	// force a reload of the period
	rCCER |= TONE_CCER;	// enable the output
#else
	prescale++;
	period++;
#endif
	// configure the GPIO to enable timer output
	px4_arch_configgpio(GPIO_TONE_ALARM);
}

void ToneAlarm::stop_note()
{
	/* stop the current note */
#if TONE_ALARM_NOT_DONE
	rCCER &= ~TONE_CCER;
#endif
	/*
	 * Make sure the GPIO is not driving the speaker.
	 */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
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
		_play_tone = _tunes.set_control(_tune) == 0;
	}

	unsigned frequency = 0, duration = 0;

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
#endif /* TONE_ALARM_CHANNEL */
