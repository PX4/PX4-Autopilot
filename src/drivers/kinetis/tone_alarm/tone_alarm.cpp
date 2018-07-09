/****************************************************************************
 *
 *   Copyright (C) 2017-2018 PX4 Development Team. All rights reserved.
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
 * Driver for the PX4 audio alarm port, /dev/tone_alarm.
 *
 * The tone_alarm driver supports a set of predefined "alarm"
 * tunes and one user-supplied tune.
 *
 * The TONE_SET_ALARM ioctl can be used to select a predefined
 * alarm tune, from 1 - <TBD>.  Selecting tune zero silences
 * the alarm.
 *
 * Tunes follow the syntax of the Microsoft GWBasic/QBasic PLAY
 * statement, with some exceptions and extensions.
 *
 * From Wikibooks:
 *
 * PLAY "[string expression]"
 *
 * Used to play notes and a score ... The tones are indicated by letters A through G.
 * Accidentals are indicated with a "+" or "#" (for sharp) or "-" (for flat)
 * immediately after the note letter. See this example:
 *
 *   PLAY "C C# C C#"
 *
 * Whitespaces are ignored inside the string expression. There are also codes that
 * set the duration, octave and tempo. They are all case-insensitive. PLAY executes
 * the commands or notes the order in which they appear in the string. Any indicators
 * that change the properties are effective for the notes following that indicator.
 *
 * Ln     Sets the duration (length) of the notes. The variable n does not indicate an actual duration
 *        amount but rather a note type; L1 - whole note, L2 - half note, L4 - quarter note, etc.
 *        (L8, L16, L32, L64, ...). By default, n = 4.
 *        For triplets and quintets, use L3, L6, L12, ... and L5, L10, L20, ... series respectively.
 *        The shorthand notation of length is also provided for a note. For example, "L4 CDE L8 FG L4 AB"
 *        can be shortened to "L4 CDE F8G8 AB". F and G play as eighth notes while others play as quarter notes.
 * On     Sets the current octave. Valid values for n are 0 through 6. An octave begins with C and ends with B.
 *        Remember that C- is equivalent to B.
 * < >    Changes the current octave respectively down or up one level.
 * Nn     Plays a specified note in the seven-octave range. Valid values are from 0 to 84. (0 is a pause.)
 *        Cannot use with sharp and flat. Cannot use with the shorthand notation neither.
 * MN     Stand for Music Normal. Note duration is 7/8ths of the length indicated by Ln. It is the default mode.
 * ML     Stand for Music Legato. Note duration is full length of that indicated by Ln.
 * MS     Stand for Music Staccato. Note duration is 3/4ths of the length indicated by Ln.
 * Pn     Causes a silence (pause) for the length of note indicated (same as Ln).
 * Tn     Sets the number of "L4"s per minute (tempo). Valid values are from 32 to 255. The default value is T120.
 * .      When placed after a note, it causes the duration of the note to be 3/2 of the set duration.
 *        This is how to get "dotted" notes. "L4 C#." would play C sharp as a dotted quarter note.
 *        It can be used for a pause as well.
 *
 * Extensions/variations:
 *
 * MB MF  The MF command causes the tune to play once and then stop. The MB command causes the
 *        tune to repeat when it ends.
 *
 */

#include <px4_config.h>
#include <px4_log.h>
#include <systemlib/px4_macros.h>
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

#include "kinetis.h"
#include "chip/kinetis_sim.h"
#include "kinetis_tpm.h"

#include <systemlib/err.h>
#include <circuit_breaker/circuit_breaker.h>

#include <px4_workqueue.h>

#include <lib/tunes/tunes.h>
#include <uORB/uORB.h>
#include <uORB/topics/tune_control.h>

#define CAT3_(A, B, C)    A##B##C
#define CAT3(A, B, C)     CAT3_(A, B, C)

/* Check that tone alarm and HRT timers are different */
#if defined(TONE_ALARM_TIMER)  && defined(HRT_TIMER)
# if TONE_ALARM_TIMER == HRT_TIMER
#   error TONE_ALARM_TIMER and HRT_TIMER must use different timers.
# endif
#endif


/*
* Period of the free-running counter, in microseconds.
*/
#define TONE_ALARM_COUNTER_PERIOD	65536

/* Tone Alarm configuration */

#define TONE_ALARM_TIMER_CLOCK    BOARD_TPM_FREQ                                 /* The input clock frequency to the TPM block */
#define TONE_ALARM_TIMER_BASE     CAT(CAT(KINETIS_TPM, TONE_ALARM_TIMER),_BASE)  /* The Base address of the TPM */
#define TONE_ALARM_TIMER_VECTOR   CAT(KINETIS_IRQ_TPM, TONE_ALARM_TIMER)         /* The TPM Interrupt vector */
#define TONE_ALARM_SIM_SCGC2_TPM  CAT(SIM_SCGC2_TPM, TONE_ALARM_TIMER)           /* The Clock Gating enable bit for this TPM */
#define TONE_ALARM_TIMER_PRESCALE TPM_SC_PS_DIV16                                /* The constant Prescaler */

#if TONE_ALARM_TIMER == 1 && defined(CONFIG_KINETIS_TPM1)
#  error must not set CONFIG_KINETIS_TPM1=y and TONE_ALARM_TIMER=1
#elif   TONE_ALARM_TIMER == 2 && defined(CONFIG_KINETIS_TPM2)
#  error must not set CONFIG_STM32_TIM2=y and TONE_ALARM_TIMER=2
#endif


# define TONE_ALARM_TIMER_FREQ    (BOARD_TPM_FREQ/(1 << (TONE_ALARM_TIMER_PRESCALE >> TPM_SC_PS_SHIFT)))

/*
* Toan Alarm clock must be a multiple of 1MHz greater than 1MHz
*/
#if (TONE_ALARM_TIMER_CLOCK % TONE_ALARM_TIMER_FREQ) != 0
# error TONE_ALARM_TIMER_CLOCK must be a multiple of 1MHz
#endif
#if TONE_ALARM_TIMER_CLOCK <= TONE_ALARM_TIMER_FREQ
# error TONE_ALARM_TIMER_CLOCK must be greater than 1MHz
#endif

#if (TONE_ALARM_CHANNEL != 0) && (TONE_ALARM_CHANNEL != 1)
# error TONE_ALARM_CHANNEL must be a value between 0 and 1
#endif


/* Register accessors */

#define _REG(_addr)	(*(volatile uint32_t *)(_addr))

/* Timer register accessors */

#define REG(_reg)	_REG(TONE_ALARM_TIMER_BASE + (_reg))

#define rSC         REG(KINETIS_TPM_SC_OFFSET)
#define rCNT        REG(KINETIS_TPM_CNT_OFFSET)
#define rMOD        REG(KINETIS_TPM_MOD_OFFSET)
#define rC0SC       REG(KINETIS_TPM_C0SC_OFFSET)
#define rC0V        REG(KINETIS_TPM_C0V_OFFSET)
#define rC1SC       REG(KINETIS_TPM_C1SC_OFFSET)
#define rC1V        REG(KINETIS_TPM_C1V_OFFSET)
#define rSTATUS     REG(KINETIS_TPM_STATUS_OFFSET)
#define rCOMBINE    REG(KINETIS_TPM_COMBINE_OFFSET)
#define rPOL        REG(KINETIS_TPM_POL_OFFSET)
#define rFILTER     REG(KINETIS_TPM_FILTER_OFFSET)
#define rQDCTRL     REG(KINETIS_TPM_QDCTRL_OFFSET)
#define rCONF       REG(KINETIS_TPM_CONF_OFFSET)

/*
* Specific registers and bits used by Tone Alarm sub-functions
*/

# define rCNV       CAT3(rC, TONE_ALARM_CHANNEL, V)            /* Channel Value Register used by Tone alarm */
# define rCNSC      CAT3(rC, TONE_ALARM_CHANNEL, SC)           /* Channel Status and Control Register used by Tone alarm */
# define STATUS     CAT3(TPM_STATUS_CH, TONE_ALARM_CHANNEL, F) /* Capture and Compare Status Register used by Tone alarm */

#define CBRK_BUZZER_KEY 782097

class ToneAlarm : public device::CDev
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
	CDev("tone_alarm", TONEALARM0_DEVICE_PATH),
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

int
ToneAlarm::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK) {
		return ret;
	}

	/* configure the GPIO to the idle state */
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);

#ifdef GPIO_TONE_ALARM_NEG
	px4_arch_configgpio(GPIO_TONE_ALARM_NEG);
#endif


	/* Select a the clock source to the TPM */

	uint32_t regval = _REG(KINETIS_SIM_SOPT2);
	regval &= ~(SIM_SOPT2_TPMSRC_MASK);
	regval |= BOARD_TPM_CLKSRC;
	_REG(KINETIS_SIM_SOPT2) = regval;


	/* Enabled System Clock Gating Control for TPM */

	regval = _REG(KINETIS_SIM_SCGC2);
	regval |= TONE_ALARM_SIM_SCGC2_TPM;
	_REG(KINETIS_SIM_SCGC2) = regval;

	/* disable and configure the timer */

	rSC = TPM_SC_TOF;

	rCNT = 0;
	rMOD = TONE_ALARM_COUNTER_PERIOD - 1;

	rSTATUS   = (TPM_STATUS_TOF | STATUS);

	/* Configure for output compare to toggle on over flow */

	rCNSC     = (TPM_CnSC_CHF | TPM_CnSC_MSA | TPM_CnSC_ELSA);

	rCOMBINE  = 0;
	rPOL      = 0;
	rFILTER   = 0;
	rQDCTRL   = 0;
	rCONF     = TPM_CONF_DBGMODE_CONT;

	/* toggle the CC output each time the count passes 0 */

	rCNV     = 0;

	/* enable the timer */

	rSC |= (TPM_SC_CMOD_LPTPM_CLK | TONE_ALARM_TIMER_PRESCALE);


	/* default the timer to a modulo value of 1; playing notes will change this */
	rMOD = 0;

	DEVICE_DEBUG("ready");
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
	unsigned divisor = (period * TONE_ALARM_TIMER_FREQ) + 0.5f;

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

	rCNT = 0;
	rMOD = divisor;		// load new toggle period

	rSC |= (TPM_SC_CMOD_LPTPM_CLK);

	// configure the GPIO to enable timer output
	px4_arch_configgpio(GPIO_TONE_ALARM);
}

void ToneAlarm::stop_note()
{
	/* stop the current note */
	rSC &= ~TPM_SC_CMOD_MASK;

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

		if (_tunes.set_control(_tune) == 0) {
			_play_tone = true;

		} else {
			_play_tone = false;
		}
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
				exit(1);
			}

			if (g_dev == nullptr) {
				g_dev = new ToneAlarm();

				if (g_dev == nullptr) {
					PX4_ERR("couldn't allocate the ToneAlarm driver");
					exit(1);
				}

				if (OK != g_dev->init()) {
					delete g_dev;
					g_dev = nullptr;
					PX4_ERR("ToneAlarm init failed");
					exit(1);
				}
			}

			exit(0);
		}

		if (!strcmp(argv1, "stop")) {
			delete g_dev;
			g_dev = nullptr;
			exit(0);
		}

		if (!strcmp(argv1, "status")) {
			g_dev->status();
			exit(0);
		}

	}

	tone_alarm_usage();
	exit(0);
}
