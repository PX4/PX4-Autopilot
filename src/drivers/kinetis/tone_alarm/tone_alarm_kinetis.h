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
 * @file tone_alarm_kinetis.h
 *
 * Low Level Driver for the PX4 audio alarm port. Subscribes to
 * tune_control and plays notes on this architecture specific
 * timer HW
 */

#pragma once

#include "chip/kinetis_sim.h"
#include "kinetis_tpm.h"

#include <drivers/tone_alarm/tone_alarm.h>

#define CAT3_(A, B, C)    A##B##C
#define CAT3(A, B, C)     CAT3_(A, B, C)

/* Check that tone alarm and HRT timers are different */
#if defined(TONE_ALARM_TIMER)  && defined(HRT_TIMER)
# if TONE_ALARM_TIMER == HRT_TIMER
#   error TONE_ALARM_TIMER and HRT_TIMER must use different timers.
# endif
#endif


/* Period of the free-running counter, in microseconds. */
#define TONE_ALARM_COUNTER_PERIOD       65536

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


# define TONE_ALARM_CLOCK    (BOARD_TPM_FREQ/(1 << (TONE_ALARM_TIMER_PRESCALE >> TPM_SC_PS_SHIFT)))

/* Tone Alarm clock must be a multiple of 1MHz greater than 1MHz. */
#if (TONE_ALARM_TIMER_CLOCK % TONE_ALARM_CLOCK) != 0
# error TONE_ALARM_TIMER_CLOCK must be a multiple of 1MHz
#endif
#if TONE_ALARM_TIMER_CLOCK <= TONE_ALARM_CLOCK
# error TONE_ALARM_TIMER_CLOCK must be greater than 1MHz
#endif

#if (TONE_ALARM_CHANNEL != 0) && (TONE_ALARM_CHANNEL != 1)
# error TONE_ALARM_CHANNEL must be a value between 0 and 1
#endif


/* Register accessors */
#define _REG(_addr)     (*(volatile uint32_t *)(_addr))

/* Timer register accessors */
#define REG(_reg)       _REG(TONE_ALARM_TIMER_BASE + (_reg))

#define rC0SC           REG(KINETIS_TPM_C0SC_OFFSET)
#define rC0V            REG(KINETIS_TPM_C0V_OFFSET)
#define rC1SC           REG(KINETIS_TPM_C1SC_OFFSET)
#define rC1V            REG(KINETIS_TPM_C1V_OFFSET)
#define rCNT            REG(KINETIS_TPM_CNT_OFFSET)
#define rCOMBINE        REG(KINETIS_TPM_COMBINE_OFFSET)
#define rCONF           REG(KINETIS_TPM_CONF_OFFSET)
#define rFILTER         REG(KINETIS_TPM_FILTER_OFFSET)
#define rMOD            REG(KINETIS_TPM_MOD_OFFSET)
#define rPOL            REG(KINETIS_TPM_POL_OFFSET)
#define rQDCTRL         REG(KINETIS_TPM_QDCTRL_OFFSET)
#define rSC             REG(KINETIS_TPM_SC_OFFSET)
#define rSTATUS         REG(KINETIS_TPM_STATUS_OFFSET)

/* Specific registers and bits used by Tone Alarm sub-functions. */
# define rCNV           CAT3(rC, TONE_ALARM_CHANNEL, V)            /* Channel Value Register used by Tone alarm */
# define rCNSC          CAT3(rC, TONE_ALARM_CHANNEL, SC)           /* Channel Status and Control Register used by Tone alarm */
# define STATUS         CAT3(TPM_STATUS_CH, TONE_ALARM_CHANNEL, F) /* Capture and Compare Status Register used by Tone alarm */
