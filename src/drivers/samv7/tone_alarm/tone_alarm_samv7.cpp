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

/**
 * @file tone_alarm_samv7.cpp
 *
 * Low Level Driver for the PX4 audio alarm port. Subscribes to
 * tune_control and plays notes on this architecture specific
 * timer HW
 */

#include "tone_alarm_samv7.h"

void ToneAlarm::activate_registers()
{
	// TODO - This is dead code...
#if TONE_ALARM_NOT_DONE
	rARR = _period;         // Load new toggle period.
	rCCER |= TONE_CCER;     // Enable the output.
	rEGR = GTIM_EGR_UG;     // Force a reload of the period.
	rPSC = _prescale;       // Load new prescaler.
#else
	_prescale++;
	_period++;
#endif
	// Configure the GPIO to enable timer output.
	px4_arch_configgpio(GPIO_TONE_ALARM);
}

void ToneAlarm::deactivate_registers()
{
	// TODO - This is dead code...
#if TONE_ALARM_NOT_DONE
	rCCER &= ~TONE_CCER;
#endif
	// Make sure the GPIO is not driving the speaker.
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);
}

int ToneAlarm::get_tone_alarm_clock()
{
	return TONE_ALARM_CLOCK;
}

void ToneAlarm::initialize_registers()
{
	// Configure the GPIO to the idle state.
	px4_arch_configgpio(GPIO_TONE_ALARM_IDLE);

#if TONE_ALARM_NOT_DONE
	// Initialise the timer.
	rCR1 = 0;
	rCR2 = 0;
	rSMCR = 0;
	rDIER = 0;
	rCCER &= TONE_CCER;	// Unlock CCMR* registers.
	rCCMR1 = TONE_CCMR1;
	rCCMR2 = TONE_CCMR2;
	rCCER = TONE_CCER;
	rDCR = 0;

# ifdef rBDTR // If using an advanced timer, you need to activate the output
	rBDTR = ATIM_BDTR_MOE; // Enable the main output of the advanced timer
# endif

	// Toggle the CC output each time the count passes 1.
	TONE_rCCR = 1;

	// Default the timer to a prescale value of 1; playing notes will change this.
	rPSC = 0;

	// Make sure the timer is running.
	rCR1 = GTIM_CR1_CEN;
#endif
}
