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
  * @file Safety button logic.
  */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <debug.h>
#include <stdlib.h>
#include <errno.h>

#include <nuttx/clock.h>

#include <arch/board/up_boardinitialize.h>
#include <arch/board/drv_gpio.h>
#include <arch/board/drv_ppm_input.h>
#include <drivers/drv_hrt.h>

#include "px4io.h"

static struct hrt_call arming_call;

/*
 * Count the number of times in a row that we see the arming button
 * held down.
 */
static unsigned arm_counter;
#define ARM_COUNTER_THRESHOLD	10

static bool safety_led_state;

static void safety_check_button(void *arg);

void
safety_init(void)
{
	/* arrange for the button handler to be called at 10Hz */
	hrt_call_every(&arming_call, 1000, 100000, safety_check_button, NULL);
}

static void
safety_check_button(void *arg)
{
	/* 
	 * Debounce the safety button, change state if it has been held for long enough.
	 *
	 * Ignore the button if FMU has not said it's OK to arm yet.
	 */
	if (BUTTON_SAFETY && system_state.arm_ok) {
		if (arm_counter < ARM_COUNTER_THRESHOLD) {
			arm_counter++;
		} else if (arm_counter == ARM_COUNTER_THRESHOLD) {
			/* change our armed state and notify the FMU */
			system_state.armed = !system_state.armed;
			arm_counter++;
			system_state.fmu_report_due = true;
		}
	} else {
		arm_counter = 0;
	}

	/* when armed, toggle the LED; when safe, leave it on */
	if (system_state.armed) {
		safety_led_state = !safety_led_state;
	} else {
		safety_led_state = true;
	}
	LED_SAFETY(safety_led_state);
}
