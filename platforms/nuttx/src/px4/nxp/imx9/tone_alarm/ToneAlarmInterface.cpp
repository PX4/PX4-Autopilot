/****************************************************************************
 *
 *   Copyright (C) 2017-2019 PX4 Development Team. All rights reserved.
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
 *
 * Tone alarm on an i.MX9 TPM channel: edge-aligned PWM at 50 % duty with the
 * TPM period set to the note frequency.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_arch/hw_description.h>
#include <hardware/imx9_memorymap.h>
#include <hardware/imx9_tpm.h>

#include <drivers/drv_tone_alarm.h>

#if !defined(TONE_ALARM_TIMER) || (TONE_ALARM_TIMER < 1) || (TONE_ALARM_TIMER > 6)
#  error TONE_ALARM_TIMER must be a TPM number between 1 and 6
#endif

#if !defined(TONE_ALARM_CHANNEL) || (TONE_ALARM_CHANNEL < 0) || (TONE_ALARM_CHANNEL > 3)
#  error TONE_ALARM_CHANNEL must be a value between 0 and 3
#endif

/* TPM counter clock, the TPM clock root the board configures (prescaler 1) */
#ifndef TONE_ALARM_CLOCK
#  define TONE_ALARM_CLOCK 1000000
#endif

static constexpr uint32_t TONE_ALARM_TIMER_BASE = timerBaseRegister(static_cast<Timer::Timer>(Timer::TPM1 + TONE_ALARM_TIMER - 1));

#define REG(_reg)       (*(volatile uint32_t *)(TONE_ALARM_TIMER_BASE + (_reg)))

#define rSC             REG(IMX9_TPM_SC_OFFSET)
#define rCNT            REG(IMX9_TPM_CNT_OFFSET)
#define rMOD            REG(IMX9_TPM_MOD_OFFSET)
#define rCONF           REG(IMX9_TPM_CONF_OFFSET)
#define rCNSC           REG(IMX9_TPM_CXSC_OFFSET(TONE_ALARM_CHANNEL))
#define rCNV            REG(IMX9_TPM_CXV_OFFSET(TONE_ALARM_CHANNEL))

namespace ToneAlarmInterface
{
void init()
{
	// Keep counting in debug mode, stop the counter and configure while it is stopped.
	rCONF |= TPM_CONF_DBGMODE_MASK;
	rSC    = TPM_SC_CMOD(TPM_SC_CMOD_VALUE_DISABLE);
	rCNT   = 0;
	rMOD   = TONE_ALARM_CLOCK / 1000 - 1; // short initial period so the first note loads promptly

	// Edge-aligned high-true PWM, output held low (CnV = 0) until a note starts.
	rCNSC  = TPM_CXSC_MSB_MASK | TPM_CXSC_ELSB_MASK;
	rCNV   = 0;

	// Up counting, prescaler 1. MOD and CnV writes take effect at the next counter overflow.
	rSC    = TPM_SC_CMOD(TPM_SC_CMOD_VALUE_COUNTER);
}

hrt_abstime start_note(unsigned frequency)
{
	const uint32_t mod = TONE_ALARM_CLOCK / frequency - 1;

	rMOD = mod;
	rCNV = (mod + 1) / 2;

	return hrt_absolute_time();
}

void stop_note()
{
	rCNV = 0;
}

} /* namespace ToneAlarmInterface */
