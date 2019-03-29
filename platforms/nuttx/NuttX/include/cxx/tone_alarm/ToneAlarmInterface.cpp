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
 * @file ToneAlarmInterface.cpp
 */

#include <chip/sam_pinmap.h>

#include <drivers/device/device.h>
#include <lib/drivers/tone_alarm/ToneAlarmInterface.h>
#include <px4_defines.h>

/* Check that tone alarm and HRT timers are different */
#if defined(TONE_ALARM_CHANNEL) && defined(HRT_TIMER_CHANNEL)
# if TONE_ALARM_CHANNEL == HRT_TIMER_CHANNEL
#   error TONE_ALARM_CHANNEL and HRT_TIMER_CHANNEL must use different timers.
# endif
#endif

#if defined(TONE_ALARM_TIMER)
# error "TONE_ALARM_TIMER should not be defined, instead define TONE_ALARM_CHANNEL from 0-11"
#endif

#ifndef TONE_ALARM_CLOCK
#define TONE_ALARM_CLOCK (BOARD_MCK_FREQUENCY/128)
#endif

/* Period of the free-running counter, in microseconds. */
#ifndef TONE_ALARM_COUNTER_PERIOD
#define TONE_ALARM_COUNTER_PERIOD 65536
#endif

/* Tone alarm configuration */
#if TONE_ALARM_CHANNEL == 0 || TONE_ALARM_CHANNEL == 1 || TONE_ALARM_CHANNEL == 2
# define TONE_ALARM_TIMER       0
#endif
#if TONE_ALARM_CHANNEL == 3 || TONE_ALARM_CHANNEL == 4 || TONE_ALARM_CHANNEL == 5
# define TONE_ALARM_TIMER       1
#endif
#if TONE_ALARM_CHANNEL == 6 || TONE_ALARM_CHANNEL == 7 || TONE_ALARM_CHANNEL == 8
# define TONE_ALARM_TIMER       2
#endif
#if TONE_ALARM_CHANNEL == 9 || TONE_ALARM_CHANNEL == 10 || TONE_ALARM_CHANNEL == 11
# define TONE_ALARM_TIMER       3
#endif

/* HRT configuration */
#if   TONE_ALARM_TIMER == 0
# define HRT_TIMER_BASE                 SAM_TC012_BASE
# if !defined(CONFIG_SAMV7_TC0)
#  error "HRT_TIMER_CHANNEL 0-2 Require CONFIG_SAMV7_TC0=y"
# endif
#elif TONE_ALARM_TIMER == 1
# define HRT_TIMER_BASE                 SAM_TC345_BASE
# if !defined(CONFIG_SAMV7_TC1)
#  error "HRT_TIMER_CHANNEL 3-5 Require CONFIG_SAMV7_TC1=y"
# endif
#elif TONE_ALARM_TIMER == 2
# define HRT_TIMER_BASE                 SAM_TC678_BASE
# if !defined(CONFIG_SAMV7_TC2)
#  error "HRT_TIMER_CHANNEL 6-8 Require CONFIG_SAMV7_TC2=y"
# endif
#elif TONE_ALARM_TIMER == 3
# define HRT_TIMER_BASE                 SAM_TC901_BASE
# if !defined(CONFIG_SAMV7_TC3)
#  error "HRT_TIMER_CHANNEL 9-11 Require CONFIG_SAMV7_TC3=y"
# endif
#else
# error "HRT_TIMER_CHANNEL should be defined valid value are from 0-11"
# endif


/* Timer register accessors. */
#define REG(_reg) (*(volatile uint32_t *)(SAM_TC0_BASE + SAM_TC_CHAN_OFFSET(HRT_TIMER_CHANNEL) + _reg))

#define rCCR    REG(SAM_TC_CCR_OFFSET)
#define rCMR    REG(SAM_TC_CMR_OFFSET)
#define rCV     REG(SAM_TC_CV_OFFSET)
#define rEMR    REG(SAM_TC_EMR_OFFSET)
#define rIER    REG(SAM_TC_IER_OFFSET)
#define rIDR    REG(SAM_TC_IDR_OFFSET)
#define rIMR    REG(SAM_TC_IMR_OFFSET)
#define rRA     REG(SAM_TC_RA_OFFSET)
#define rRAB    REG(SAM_TC_RAB_OFFSET)
#define rRB     REG(SAM_TC_RB_OFFSET)
#define rRC     REG(SAM_TC_RC_OFFSET)
#define rSMMR   REG(SAM_TC_SMMR_OFFSET)
#define rSR     REG(SAM_TC_SR_OFFSET)

void ToneAlarmInterface::init()
{
#ifdef GPIO_TONE_ALARM_NEG
	px4_arch_configgpio(GPIO_TONE_ALARM_NEG);
#else
	// Configure the GPIO to the idle state.
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
#endif

// TODO - This is dead code.
#if TONE_ALARM_NOT_DONE
	// Initialize the timer.
	rCCER &= TONE_CCER;	// Unlock CCMR* registers.
	rCCER  = TONE_CCER;
	rCCMR1 = TONE_CCMR1;
	rCCMR2 = TONE_CCMR2;
	rDCR   = 0;
	rCR1   = 0;
	rCR2   = 0;
	rDIER  = 0;
	rSMCR  = 0;

#ifdef rBDTR // If using an advanced timer, you need to activate the output.
	rBDTR = ATIM_BDTR_MOE; // Enable the main output of the advanced timer.
#endif

	TONE_rCCR = 1;		// Toggle the CC output each time the count passes 1.
	rPSC = 0;		// Default the timer to a prescale value of 1; playing notes will change this.
	rCR1 = GTIM_CR1_CEN;	// Ensure the timer is running.
#endif
}

void ToneAlarmInterface::start_note(unsigned frequency)
{
	// Calculate the signal switching period.
	// (Signal switching period is one half of the frequency period).
	float signal_period = (1.0f / frequency) * 0.5f;

	// Calculate the hardware clock divisor rounded to the nearest integer.
	unsigned int divisor = std::round(signal_period * TONE_ALARM_CLOCK);

	// Pick the lowest prescaler value that we can use.
	// (Note that the effective prescale value is 1 greater.)
	unsigned int prescale = divisor / TONE_ALARM_COUNTER_PERIOD;

	// Calculate the period for the selected prescaler value.
	unsigned int period = (divisor / (prescale + 1)) - 1;

// TODO - This is dead code.
#if TONE_ALARM_NOT_DONE
	rPSC = prescale;	// Load new prescaler.
	rARR = period;		// Load new signal switching period.
	rEGR = GTIM_EGR_UG;	// Force a reload of the period.
	rCCER |= TONE_CCER;	// Enable the output.
#else
	prescale++;
	period++;
#endif

	// Configure the GPIO to enable timer output.
	px4_arch_configgpio(GPIO_TONE_ALARM);
}

void ToneAlarmInterface::stop_note()
{
	// Stop the current note.
// TODO - This is dead code.
#if TONE_ALARM_NOT_DONE
	rCCER &= ~TONE_CCER;
#endif
	// Ensure the GPIO is not driving the speaker.
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
}
