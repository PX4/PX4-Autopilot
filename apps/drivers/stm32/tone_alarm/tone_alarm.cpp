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
 * Driver for the PX4 audio alarm port, /dev/tone_alarm.
 *
 * The tone_alarm driver supports a set of predefined "alarm"
 * patterns and one user-supplied pattern.  Patterns are ordered by
 * priority, with a higher-priority pattern interrupting any
 * lower-priority pattern that might be playing.
 *
 * The TONE_SET_ALARM ioctl can be used to select a predefined
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

#include <nuttx/config.h>

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

#include <arch/board/board.h>
#include <drivers/drv_hrt.h>

#include <arch/stm32/chip.h>
#include <up_internal.h>
#include <up_arch.h>

#include <stm32_internal.h>
#include <stm32_gpio.h>
#include <stm32_tim.h>

#include <systemlib/err.h>

#ifndef CONFIG_HRT_TIMER
# error This driver requires CONFIG_HRT_TIMER
#endif

/* Tone alarm configuration */
#if   TONE_ALARM_TIMER == 2
# define TONE_ALARM_BASE		STM32_TIM2_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM2_CLKIN
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM2EN
# ifdef CONFIG_STM32_TIM2
#  error Must not set CONFIG_STM32_TIM2 when TONE_ALARM_TIMER is 2
# endif
#elif TONE_ALARM_TIMER == 3
# define TONE_ALARM_BASE		STM32_TIM3_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM3_CLKIN
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM3EN
# ifdef CONFIG_STM32_TIM3
#  error Must not set CONFIG_STM32_TIM3 when TONE_ALARM_TIMER is 3
# endif
#elif TONE_ALARM_TIMER == 4
# define TONE_ALARM_BASE		STM32_TIM4_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM4_CLKIN
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM4EN
# ifdef CONFIG_STM32_TIM4
#  error Must not set CONFIG_STM32_TIM4 when TONE_ALARM_TIMER is 4
# endif
#elif TONE_ALARM_TIMER == 5
# define TONE_ALARM_BASE		STM32_TIM5_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM5_CLKIN
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM5EN
# ifdef CONFIG_STM32_TIM5
#  error Must not set CONFIG_STM32_TIM5 when TONE_ALARM_TIMER is 5
# endif
#elif TONE_ALARM_TIMER == 9
# define TONE_ALARM_BASE		STM32_TIM9_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM9_CLKIN
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM9EN
# ifdef CONFIG_STM32_TIM9
#  error Must not set CONFIG_STM32_TIM9 when TONE_ALARM_TIMER is 9
# endif
#elif TONE_ALARM_TIMER == 10
# define TONE_ALARM_BASE		STM32_TIM10_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM10_CLKIN
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM10EN
# ifdef CONFIG_STM32_TIM10
#  error Must not set CONFIG_STM32_TIM10 when TONE_ALARM_TIMER is 10
# endif
#elif TONE_ALARM_TIMER == 11
# define TONE_ALARM_BASE		STM32_TIM11_BASE
# define TONE_ALARM_CLOCK		STM32_APB1_TIM11_CLKIN
# define TONE_ALARM_CLOCK_ENABLE	RCC_APB1ENR_TIM11EN
# ifdef CONFIG_STM32_TIM11
#  error Must not set CONFIG_STM32_TIM11 when TONE_ALARM_TIMER is 11
# endif
#else
# error Must set TONE_ALARM_TIMER to a generic timer in order to use this driver.
#endif

#if TONE_ALARM_CHANNEL == 1
# define TONE_CCMR1	(3 << 4)
# define TONE_CCMR2	0
# define TONE_CCER	(1 << 0)
# define TONE_rCCR	rCCR1
#elif TONE_ALARM_CHANNEL == 2
# define TONE_CCMR1	(3 << 12)
# define TONE_CCMR2	0
# define TONE_CCER	(1 << 4)
# define TONE_rCCR	rCCR2
#elif TONE_ALARM_CHANNEL == 3
# define TONE_CCMR1	0
# define TONE_CCMR2	(3 << 4)
# define TONE_CCER	(1 << 8)
# define TONE_rCCR	rCCR3
#elif TONE_ALARM_CHANNEL == 4
# define TONE_CCMR1	0
# define TONE_CCMR2	(3 << 12)
# define TONE_CCER	(1 << 12)
# define TONE_rCCR	rCCR4
#else
# error Must set TONE_ALARM_CHANNEL to a value between 1 and 4 to use this driver.
#endif


/*
 * Timer register accessors
 */
#define REG(_reg)	(*(volatile uint32_t *)(TONE_ALARM_BASE + _reg))

#define rCR1     	REG(STM32_GTIM_CR1_OFFSET)
#define rCR2     	REG(STM32_GTIM_CR2_OFFSET)
#define rSMCR    	REG(STM32_GTIM_SMCR_OFFSET)
#define rDIER    	REG(STM32_GTIM_DIER_OFFSET)
#define rSR      	REG(STM32_GTIM_SR_OFFSET)
#define rEGR     	REG(STM32_GTIM_EGR_OFFSET)
#define rCCMR1   	REG(STM32_GTIM_CCMR1_OFFSET)
#define rCCMR2   	REG(STM32_GTIM_CCMR2_OFFSET)
#define rCCER    	REG(STM32_GTIM_CCER_OFFSET)
#define rCNT     	REG(STM32_GTIM_CNT_OFFSET)
#define rPSC     	REG(STM32_GTIM_PSC_OFFSET)
#define rARR     	REG(STM32_GTIM_ARR_OFFSET)
#define rCCR1    	REG(STM32_GTIM_CCR1_OFFSET)
#define rCCR2    	REG(STM32_GTIM_CCR2_OFFSET)
#define rCCR3    	REG(STM32_GTIM_CCR3_OFFSET)
#define rCCR4    	REG(STM32_GTIM_CCR4_OFFSET)
#define rDCR     	REG(STM32_GTIM_DCR_OFFSET)
#define rDMAR    	REG(STM32_GTIM_DMAR_OFFSET)

class ToneAlarm : public device::CDev
{
public:
	ToneAlarm();
	~ToneAlarm();

	virtual int		init();

	virtual int		ioctl(file *filp, int cmd, unsigned long arg);
	virtual ssize_t		write(file *filp, const char *buffer, size_t len);

private:
	static const unsigned	_max_pattern = 6;
	static const unsigned	_pattern_none = _max_pattern + 1;
	static const unsigned	_pattern_user = 0;
	static const unsigned	_max_pattern_len = 40;
	static const unsigned	_note_max = TONE_NOTE_MAX;

	static const tone_note	_patterns[_max_pattern][_max_pattern_len];
	static const uint16_t	_notes[_note_max];

	unsigned		_current_pattern;
	unsigned		_next_note;

	hrt_call		_note_end;
	tone_note		_user_pattern[_max_pattern_len];

	static void		next_trampoline(void *arg);
	void			next();
	bool			check(tone_note *pattern);
	void			stop();
};


/* predefined patterns for alarms 1-_max_pattern */
const tone_note ToneAlarm::_patterns[_max_pattern][_max_pattern_len] = {
	{
		{TONE_NOTE_A7, 12},
		{TONE_NOTE_D8, 12},
		{TONE_NOTE_C8, 12},
		{TONE_NOTE_A7, 12},
		{TONE_NOTE_D8, 12},
		{TONE_NOTE_C8, 12},
		{TONE_NOTE_D8, 4},
		{TONE_NOTE_C8, 4},
		{TONE_NOTE_D8, 4},
		{TONE_NOTE_C8, 4},
		{TONE_NOTE_D8, 4},
		{TONE_NOTE_C8, 4},
	},
	{{TONE_NOTE_B6, 100}, {TONE_NOTE_B6, DURATION_REPEAT}},
	{{TONE_NOTE_C7, 100}},
	{{TONE_NOTE_D7, 100}},
	{{TONE_NOTE_E7, 100}},
	{
		//This is tetris ;)
		{TONE_NOTE_C6, 40},
		{TONE_NOTE_G5, 20},
		{TONE_NOTE_G5S, 20},
		{TONE_NOTE_A5S, 40},
		{TONE_NOTE_G5S, 20},
		{TONE_NOTE_G5, 20},
		{TONE_NOTE_F5, 40},
		{TONE_NOTE_F5, 20},
		{TONE_NOTE_G5S, 20},
		{TONE_NOTE_C6, 40},
		{TONE_NOTE_A5S, 20},
		{TONE_NOTE_G5S, 20},
		{TONE_NOTE_G5, 60},
		{TONE_NOTE_G5S, 20},
		{TONE_NOTE_A5S, 40},
		{TONE_NOTE_C6, 40},
		{TONE_NOTE_G5S, 40},
		{TONE_NOTE_F5, 40},
		{TONE_NOTE_F5, 60},
		{TONE_NOTE_A5S, 40},
		{TONE_NOTE_C6S, 20},
		{TONE_NOTE_F6, 40},
		{TONE_NOTE_D6S, 20},
		{TONE_NOTE_C6S, 20},
		{TONE_NOTE_C6, 60},
		{TONE_NOTE_G5S, 20},
		{TONE_NOTE_C6, 40},
		{TONE_NOTE_A5S, 20},
		{TONE_NOTE_G5S, 20},
		{TONE_NOTE_G5, 40},
		{TONE_NOTE_G5, 20},
		{TONE_NOTE_G5S, 20},
		{TONE_NOTE_A5S, 40},
		{TONE_NOTE_C6, 40},
		{TONE_NOTE_G5S, 40},
		{TONE_NOTE_F5, 40},
		{TONE_NOTE_F5, 60},
	}
};

const uint16_t ToneAlarm::_notes[_note_max] = {
	63707, /* E4 */
	60132, /* F4 */
	56758, /* F#4/Gb4 */
	53571, /* G4 */
	50565, /* G#4/Ab4 */
	47727, /* A4 */
	45048, /* A#4/Bb4 */
	42520, /* B4 */
	40133, /* C5 */
	37880, /* C#5/Db5 */
	35755, /* D5 */
	33748, /* D#5/Eb5 */
	31853, /* E5 */
	30066, /* F5 */
	28378, /* F#5/Gb5 */
	26786, /* G5 */
	25282, /* G#5/Ab5 */
	23863, /* A5 */
	22524, /* A#5/Bb5 */
	21260, /* B5 */
	20066, /* C6 */
	18940, /* C#6/Db6 */
	17877, /* D6 */
	16874, /* D#6/Eb6 */
	15927, /* E6 */
	15033, /* F6 */
	14189, /* F#6/Gb6 */
	13393, /* G6 */
	12641, /* G#6/Ab6 */
	11931, /* A6 */
	11262, /* A#6/Bb6 */
	10630, /* B6 */
	10033, /* C7 */
	9470, /* C#7/Db7 */
	8938, /* D7 */
	8437, /* D#7/Eb7 */
	7963, /* E7 */
	7516, /* F7 */
	7094, /* F#7/Gb7 */
	6696, /* G7 */
	6320, /* G#7/Ab7 */
	5965, /* A7 */
	5631, /* A#7/Bb7 */
	5315, /* B7 */
	5016, /* C8 */
	4735, /* C#8/Db8 */
	4469, /* D8 */
	4218  /* D#8/Eb8 */
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int tone_alarm_main(int argc, char *argv[]);


ToneAlarm::ToneAlarm() :
	CDev("tone_alarm", "/dev/tone_alarm"),
	_current_pattern(_pattern_none),
	_next_note(0)
{
	// enable debug() calls
	//_debug_enabled = true;
}

ToneAlarm::~ToneAlarm()
{
}

int
ToneAlarm::init()
{
	int ret;

	ret = CDev::init();

	if (ret != OK)
		return ret;

	/* configure the GPIO to the idle state */
	stm32_configgpio(GPIO_TONE_ALARM);

	/* clock/power on our timer */
	modifyreg32(STM32_RCC_APB1ENR, 0, TONE_ALARM_CLOCK_ENABLE);

	/* initialise the timer */
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = 0;
	rCCER &= TONE_CCER;		/* unlock CCMR* registers */
	rCCMR1 = TONE_CCMR1;
	rCCMR2 = TONE_CCMR2;
	rCCER = TONE_CCER;
	rDCR = 0;

	/* toggle the CC output each time the count passes 1 */
	TONE_rCCR = 1;

	/*
	 * Configure the timebase to free-run at half max frequency.
	 * XXX this should be more flexible in order to get a better
	 * frequency range, but for the F4 with the APB1 timers based
	 * at 42MHz, this gets us down to ~320Hz or so.
	 */
	rPSC = 1;

	/* make sure the timer is running */
	rCR1 = GTIM_CR1_CEN;

	debug("ready");
	return OK;
}

int
ToneAlarm::ioctl(file *filp, int cmd, unsigned long arg)
{
	int result = 0;
	unsigned new_pattern = arg;

	debug("ioctl %i %u", cmd, new_pattern);

//	irqstate_t flags = irqsave();

	/* decide whether to increase the alarm level to cmd or leave it alone */
	switch (cmd) {
	case TONE_SET_ALARM:
		debug("TONE_SET_ALARM %u", arg);

		if (new_pattern == 0) {
			/* cancel any current alarm */
			_current_pattern = _pattern_none;
			next();

		} else if (new_pattern > _max_pattern) {

			/* not a legal alarm value */
			result = -ERANGE;

		} else if ((_current_pattern == _pattern_none) || (new_pattern > _current_pattern)) {

			/* higher priority than the current alarm */
			_current_pattern = new_pattern;
			_next_note = 0;

			/* and start playing it */
			next();

		} else {
			/* current pattern is higher priority than the new pattern, ignore */
		}

		break;

	default:
		result = -ENOTTY;
		break;
	}

//	irqrestore(flags);

	/* give it to the superclass if we didn't like it */
	if (result == -ENOTTY)
		result = CDev::ioctl(filp, cmd, arg);

	return result;
}

int
ToneAlarm::write(file *filp, const char *buffer, size_t len)
{
	irqstate_t flags;

	/* sanity-check the size of the write */
	if (len > (_max_pattern_len * sizeof(tone_note)))
		return -EFBIG;

	if ((len % sizeof(tone_note)) || (len == 0))
		return -EIO;

	if (!check((tone_note *)buffer))
		return -EIO;

	flags = irqsave();

	/* if we aren't playing an alarm tone */
	if (_current_pattern <= _pattern_user) {

		/* reset the tone state to play the new user pattern */
		_current_pattern = _pattern_user;
		_next_note = 0;

		/* copy in the new pattern */
		memset(_user_pattern, 0, sizeof(_user_pattern));
		memcpy(_user_pattern, buffer, len);

		/* and start it */
		debug("starting user pattern");
		next();
	}

	irqrestore(flags);

	return len;
}

void
ToneAlarm::next_trampoline(void *arg)
{
	ToneAlarm *ta = (ToneAlarm *)arg;

	ta->next();
}

void
ToneAlarm::next(void)
{
	const tone_note *np;

	/* stop the current note */
	rCCER &= ~TONE_CCER;

	/* if we are no longer playing a pattern, we have nothing else to do here */
	if (_current_pattern == _pattern_none) {
		stop();
		return;
	}

	ASSERT(_next_note < _note_max);

	/* find the note to play */
	if (_current_pattern == _pattern_user) {
		np = &_user_pattern[_next_note];

	} else {
		np = &_patterns[_current_pattern - 1][_next_note];
	}

	/* work out which note is next */
	_next_note++;

	if (_next_note >= _note_max) {
		/* hit the end of the pattern, stop */
		_current_pattern = _pattern_none;

	} else if (np[1].duration == DURATION_END) {
		/* hit the end of the pattern, stop */
		_current_pattern = _pattern_none;

	} else if (np[1].duration == DURATION_REPEAT) {
		/* next note is a repeat, rewind in preparation */
		_next_note = 0;
	}

	/* set the timer to play the note, if required */
	if (np->pitch <= TONE_NOTE_SILENCE) {

		/* set reload based on the pitch */
		rARR = _notes[np->pitch];

		/* force an update, reloads the counter and all registers */
		rEGR = GTIM_EGR_UG;

		/* enable the output */
		rCCER |= TONE_CCER;
	}

	/* arrange a callback when the note/rest is done */
	hrt_call_after(&_note_end, (hrt_abstime)10000 * np->duration, (hrt_callout)next_trampoline, this);
}

bool
ToneAlarm::check(tone_note *pattern)
{
	for (unsigned i = 0; i < _note_max; i++) {

		/* first note must not be repeat or end */
		if ((i == 0) &&
		    ((pattern[i].duration == DURATION_END) || (pattern[i].duration == DURATION_REPEAT)))
			return false;

		if (pattern[i].duration == DURATION_END)
			break;

		/* pitch must be legal */
		if (pattern[i].pitch >= _note_max)
			return false;
	}

	return true;
}

void
ToneAlarm::stop()
{
	/* stop the current note */
	rCCER &= ~TONE_CCER;

	/*
	 * Make sure the GPIO is not driving the speaker.
	 *
	 * XXX this presumes PX4FMU and the onboard speaker driver FET.
	 */
	stm32_gpiowrite(GPIO_TONE_ALARM, 0);
}

/**
 * Local functions in support of the shell command.
 */
namespace
{

ToneAlarm	*g_dev;

int
play_pattern(unsigned pattern)
{
	int	fd, ret;

	fd = open("/dev/tone_alarm", 0);

	if (fd < 0)
		err(1, "/dev/tone_alarm");

	ret = ioctl(fd, TONE_SET_ALARM, pattern);

	if (ret != 0)
		err(1, "TONE_SET_ALARM");

	exit(0);
}

} // namespace

int
tone_alarm_main(int argc, char *argv[])
{
	unsigned pattern;

	/* start the driver lazily */
	if (g_dev == nullptr) {
		g_dev = new ToneAlarm;

		if (g_dev == nullptr)
			errx(1, "couldn't allocate the ToneAlarm driver");

		if (g_dev->init() != OK) {
			delete g_dev;
			errx(1, "ToneAlarm init failed");
		}
	}

	if ((argc > 1) && !strcmp(argv[1], "start"))
		play_pattern(1);

	if ((argc > 1) && !strcmp(argv[1], "stop"))
		play_pattern(0);

	if ((pattern = strtol(argv[1], nullptr, 10)) != 0)
		play_pattern(pattern);

	errx(1, "unrecognised command, try 'start', 'stop' or an alarm number");
}
