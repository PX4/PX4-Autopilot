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
 * @file tone_alarm_kinetis.cpp
 *
 * Low Level Driver for the PX4 audio alarm port. Subscribes to
 * tune_control and plays notes on this architecture specific
 * timer HW
 */

#include "tone_alarm_kinetis.h"

void ToneAlarm::activate_registers()
{
	rCNT = 0;
	rMOD = _divisor;	// Load new toggle period.
	rSC |= (TPM_SC_CMOD_LPTPM_CLK);

	// Configure the GPIO to enable timer output.
	px4_arch_configgpio(GPIO_TONE_ALARM);
}

void ToneAlarm::deactivate_registers()
{
	// Stop the current note.
	rSC &= ~TPM_SC_CMOD_MASK;

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

#ifdef GPIO_TONE_ALARM_NEG

	px4_arch_configgpio(GPIO_TONE_ALARM_NEG);
#endif
	// Select a the clock source to the TPM.
	uint32_t regval = _REG(KINETIS_SIM_SOPT2);
	regval &= ~(SIM_SOPT2_TPMSRC_MASK);
	regval |= BOARD_TPM_CLKSRC;
	_REG(KINETIS_SIM_SOPT2) = regval;

	// Enabled System Clock Gating Control for TPM.
	regval = _REG(KINETIS_SIM_SCGC2);
	regval |= TONE_ALARM_SIM_SCGC2_TPM;
	_REG(KINETIS_SIM_SCGC2) = regval;

	// Disable and configure the timer.
	rSC = TPM_SC_TOF;

	rCNT      = 0;
	rCNV      = 0;	// Toggle the CC output each time the count passes 0.
	rCOMBINE  = 0;
	rFILTER   = 0;
	rPOL      = 0;
	rQDCTRL   = 0;

	rMOD = TONE_ALARM_COUNTER_PERIOD - 1;

	rSTATUS   = (TPM_STATUS_TOF | STATUS);

	// Configure for output compare to toggle on over flow.
	rCNSC     = (TPM_CnSC_CHF | TPM_CnSC_MSA | TPM_CnSC_ELSA);
	rCONF     = TPM_CONF_DBGMODE_CONT;

	// Enable the timer.
	rSC |= (TPM_SC_CMOD_LPTPM_CLK | TONE_ALARM_TIMER_PRESCALE);

	// Default the timer to a modulo value of 1; playing notes will change this.
	rMOD = 0;
}
