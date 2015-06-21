/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file px4io.c
 * Top-level logic for the PX4IO module.
 */

#include <px4_config.h>
#include <nuttx/arch.h>

#include <stdio.h>	// required for task_create
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <signal.h>
#include <crc32.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/pwm_limit/pwm_limit.h>

#include <stm32_uart.h>

#define DEBUG
#include "px4io.h"

__EXPORT int user_start(int argc, char *argv[]);

extern void up_cxxinitialize(void);

struct sys_state_s 	system_state;

static struct hrt_call serial_dma_call;

pwm_limit_t pwm_limit;

/*
 * a set of debug buffers to allow us to send debug information from ISRs
 */

static volatile uint32_t msg_counter;
static volatile uint32_t last_msg_counter;
static volatile uint8_t msg_next_out, msg_next_in;

/*
 * WARNING: too large buffers here consume the memory required
 * for mixer handling. Do not allocate more than 80 bytes for
 * output.
 */
#define NUM_MSG 1
static char msg[NUM_MSG][40];

static void heartbeat_blink(void);
static void ring_blink(void);

/*
 * add a debug message to be printed on the console
 */
void
isr_debug(uint8_t level, const char *fmt, ...)
{
	if (level > r_page_setup[PX4IO_P_SETUP_SET_DEBUG]) {
		return;
	}

	va_list ap;
	va_start(ap, fmt);
	vsnprintf(msg[msg_next_in], sizeof(msg[0]), fmt, ap);
	va_end(ap);
	msg_next_in = (msg_next_in + 1) % NUM_MSG;
	msg_counter++;
}

/*
 * show all pending debug messages
 */
static void
show_debug_messages(void)
{
	if (msg_counter != last_msg_counter) {
		uint32_t n = msg_counter - last_msg_counter;

		if (n > NUM_MSG) { n = NUM_MSG; }

		last_msg_counter = msg_counter;

		while (n--) {
			debug("%s", msg[msg_next_out]);
			msg_next_out = (msg_next_out + 1) % NUM_MSG;
		}
	}
}

static void
heartbeat_blink(void)
{
	static bool heartbeat = false;
	LED_BLUE(heartbeat = !heartbeat);
}

static void
ring_blink(void)
{
#ifdef GPIO_LED4

	if (/* IO armed */ (r_status_flags & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)
			   /* and FMU is armed */ && (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {
		LED_RING(1);
		return;
	}

	// XXX this led code does have
	// intentionally a few magic numbers.
	const unsigned max_brightness = 118;

	static unsigned counter = 0;
	static unsigned brightness = max_brightness;
	static unsigned brightness_counter = 0;
	static unsigned on_counter = 0;

	if (brightness_counter < max_brightness) {

		bool on = ((on_counter * 100) / brightness_counter + 1) <= ((brightness * 100) / max_brightness + 1);

		// XXX once led is PWM driven,
		// remove the ! in the line below
		// to return to the proper breathe
		// animation / pattern (currently inverted)
		LED_RING(!on);
		brightness_counter++;

		if (on) {
			on_counter++;
		}

	} else {

		if (counter >= 62) {
			counter = 0;
		}

		int n;

		if (counter < 32) {
			n = counter;

		} else {
			n = 62 - counter;
		}

		brightness = (n * n) / 8;
		brightness_counter = 0;
		on_counter = 0;
		counter++;
	}

#endif
}

static uint64_t reboot_time;

/**
   schedule a reboot in time_delta_usec microseconds
 */
void schedule_reboot(uint32_t time_delta_usec)
{
	reboot_time = hrt_absolute_time() + time_delta_usec;
}

/**
   check for a scheduled reboot
 */
static void check_reboot(void)
{
	if (reboot_time != 0 && hrt_absolute_time() > reboot_time) {
		up_systemreset();
	}
}

static void
calculate_fw_crc(void)
{
#define APP_SIZE_MAX 0xf000
#define APP_LOAD_ADDRESS 0x08001000
	// compute CRC of the current firmware
	uint32_t sum = 0;

	for (unsigned p = 0; p < APP_SIZE_MAX; p += 4) {
		uint32_t bytes = *(uint32_t *)(p + APP_LOAD_ADDRESS);
		sum = crc32part((uint8_t *)&bytes, sizeof(bytes), sum);
	}

	r_page_setup[PX4IO_P_SETUP_CRC]   = sum & 0xFFFF;
	r_page_setup[PX4IO_P_SETUP_CRC + 1] = sum >> 16;
}

int
user_start(int argc, char *argv[])
{
	/* configure the first 8 PWM outputs (i.e. all of them) */
	up_pwm_servo_init(0xff);

	/* run C++ ctors before we go any further */
	up_cxxinitialize();

	/* reset all to zero */
	memset(&system_state, 0, sizeof(system_state));

	/* configure the high-resolution time/callout interface */
	hrt_init();

	/* calculate our fw CRC so FMU can decide if we need to update */
	calculate_fw_crc();

	/*
	 * Poll at 1ms intervals for received bytes that have not triggered
	 * a DMA event.
	 */
#ifdef CONFIG_ARCH_DMA
	hrt_call_every(&serial_dma_call, 1000, 1000, (hrt_callout)stm32_serial_dma_poll, NULL);
#endif

	/* print some startup info */
	lowsyslog("\nPX4IO: starting\n");

	/* default all the LEDs to off while we start */
	LED_AMBER(false);
	LED_BLUE(false);
	LED_SAFETY(false);
#ifdef GPIO_LED4
	LED_RING(false);
#endif

	/* turn on servo power (if supported) */
#ifdef POWER_SERVO
	POWER_SERVO(true);
#endif

	/* turn off S.Bus out (if supported) */
#ifdef ENABLE_SBUS_OUT
	ENABLE_SBUS_OUT(false);
#endif

	/* start the safety switch handler */
	safety_init();

	/* initialise the control inputs */
	controls_init();

	/* set up the ADC */
	adc_init();

	/* start the FMU interface */
	interface_init();

	/* add a performance counter for mixing */
	perf_counter_t mixer_perf = perf_alloc(PC_ELAPSED, "mix");

	/* add a performance counter for controls */
	perf_counter_t controls_perf = perf_alloc(PC_ELAPSED, "controls");

	/* and one for measuring the loop rate */
	perf_counter_t loop_perf = perf_alloc(PC_INTERVAL, "loop");

	struct mallinfo minfo = mallinfo();
	lowsyslog("MEM: free %u, largest %u\n", minfo.mxordblk, minfo.fordblks);

	/* initialize PWM limit lib */
	pwm_limit_init(&pwm_limit);

	/*
	 *    P O L I C E    L I G H T S
	 *
	 * Not enough memory, lock down.
	 *
	 * We might need to allocate mixers later, and this will
	 * ensure that a developer doing a change will notice
	 * that he just burned the remaining RAM with static
	 * allocations. We don't want him to be able to
	 * get past that point. This needs to be clearly
	 * documented in the dev guide.
	 *
	 */
	if (minfo.mxordblk < 600) {

		lowsyslog("ERR: not enough MEM");
		bool phase = false;

		while (true) {

			if (phase) {
				LED_AMBER(true);
				LED_BLUE(false);

			} else {
				LED_AMBER(false);
				LED_BLUE(true);
			}

			up_udelay(250000);

			phase = !phase;
		}
	}

	/* Start the failsafe led init */
	failsafe_led_init();

	/*
	 * Run everything in a tight loop.
	 */

	uint64_t last_debug_time = 0;
	uint64_t last_heartbeat_time = 0;

	for (;;) {

		/* track the rate at which the loop is running */
		perf_count(loop_perf);

		/* kick the mixer */
		perf_begin(mixer_perf);
		mixer_tick();
		perf_end(mixer_perf);

		/* kick the control inputs */
		perf_begin(controls_perf);
		controls_tick();
		perf_end(controls_perf);

		/*
		  blink blue LED at 4Hz in normal operation. When in
		  override blink 4x faster so the user can clearly see
		  that override is happening. This helps when
		  pre-flight testing the override system
		 */
                uint32_t heartbeat_period_us = 250*1000UL;
                if (r_status_flags & PX4IO_P_STATUS_FLAGS_OVERRIDE) {
			heartbeat_period_us /= 4;
                }

                if ((hrt_absolute_time() - last_heartbeat_time) > heartbeat_period_us) {
                    last_heartbeat_time = hrt_absolute_time();
                    heartbeat_blink();
                }

		ring_blink();

		check_reboot();

		/* check for debug activity (default: none) */
		show_debug_messages();

		/* post debug state at ~1Hz - this is via an auxiliary serial port
		 * DEFAULTS TO OFF!
		 */
		if (hrt_absolute_time() - last_debug_time > (1000 * 1000)) {

			isr_debug(1, "d:%u s=0x%x a=0x%x f=0x%x m=%u",
				  (unsigned)r_page_setup[PX4IO_P_SETUP_SET_DEBUG],
				  (unsigned)r_status_flags,
				  (unsigned)r_setup_arming,
				  (unsigned)r_setup_features,
				  (unsigned)mallinfo().mxordblk);
			last_debug_time = hrt_absolute_time();
		}
	}
}

