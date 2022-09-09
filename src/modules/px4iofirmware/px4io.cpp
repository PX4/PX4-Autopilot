/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file px4io.cpp
 * Top-level logic for the PX4IO module.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/px4_config.h>

#include <stdio.h>	// required for task_create
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <malloc.h>
#include <poll.h>
#include <signal.h>
#include <crc32.h>
#include <syslog.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_watchdog.h>

#include <stm32_uart.h>

#define DEBUG
#include "px4io.h"

struct sys_state_s system_state;

static struct hrt_call serial_dma_call;

/*
 * a set of debug buffers to allow us to send debug information from ISRs
 */

static volatile uint32_t msg_counter;
static volatile uint32_t last_msg_counter;
static volatile uint8_t msg_next_out;
static volatile uint8_t msg_next_in;

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

/*
 * Maximum interval in us before FMU signal is considered lost
 */
#define FMU_INPUT_DROP_LIMIT_US		500000

/* current servo arm/disarm state */
static volatile bool mixer_servos_armed = false;

static bool new_fmu_data = false;
static uint64_t last_fmu_update = 0;

/*
 * WARNING: too large buffers here consume the memory required
 * for mixer handling. Do not allocate more than 80 bytes for
 * output.
 */
#define NUM_MSG 1
static char msg[NUM_MSG][CONFIG_USART1_TXBUFSIZE];

static void heartbeat_blink(void);
static void ring_blink(void);
static void update_mem_usage(void);

void atomic_modify_or(volatile uint16_t *target, uint16_t modification)
{
	if ((*target | modification) != *target) {
		PX4_CRITICAL_SECTION(*target |= modification);
	}
}

void atomic_modify_clear(volatile uint16_t *target, uint16_t modification)
{
	if ((*target & ~modification) != *target) {
		PX4_CRITICAL_SECTION(*target &= ~modification);
	}
}

void atomic_modify_and(volatile uint16_t *target, uint16_t modification)
{
	if ((*target & modification) != *target) {
		PX4_CRITICAL_SECTION(*target &= modification);
	}
}

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

/*
 * Get the memory usage at 2 Hz while not armed
 */
static void
update_mem_usage(void)
{
	if (/* FMU is armed */ (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {
		return;
	}

	static uint64_t last_mem_time = 0;
	uint64_t now = hrt_absolute_time();

	if (now - last_mem_time > (500 * 1000)) {
		struct mallinfo minfo = mallinfo();
		r_page_status[PX4IO_P_STATUS_FREEMEM] = minfo.fordblks;
		last_mem_time = now;
	}
}

static void
heartbeat_blink(void)
{
#if defined(LED_BLUE)
	static bool heartbeat = false;
	LED_BLUE(heartbeat = !heartbeat);
#endif /* LED_BLUE */
}

static void
ring_blink(void)
{
#if defined(LED_GREEN)

	if (/* FMU is armed */ (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {
		LED_GREEN(true);
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
		LED_GREEN(!on);
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

static void mixer_tick()
{
	/* check that we are receiving fresh data from the FMU */
	irqstate_t irq_flags = enter_critical_section();
	const hrt_abstime fmu_data_received_time = system_state.fmu_data_received_time;
	leave_critical_section(irq_flags);

	if ((fmu_data_received_time == 0) ||
	    hrt_elapsed_time(&fmu_data_received_time) > FMU_INPUT_DROP_LIMIT_US) {

		/* too long without FMU input, time to go to failsafe */
		if (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK) {
			isr_debug(1, "AP RX timeout");
		}

		atomic_modify_clear(&r_status_flags, PX4IO_P_STATUS_FLAGS_FMU_OK);

	} else {
		atomic_modify_or(&r_status_flags, PX4IO_P_STATUS_FLAGS_FMU_OK);

		if (fmu_data_received_time > last_fmu_update) {
			new_fmu_data = true;
			last_fmu_update = fmu_data_received_time;
		}
	}

	/*
	 * Decide whether the servos should be armed right now.
	 *
	 * We must be armed, and we must have a PWM source; either raw from
	 * FMU or from the mixer.
	 *
	 */
	const bool should_arm = (r_status_flags & PX4IO_P_STATUS_FLAGS_FMU_OK)
				&& (r_setup_arming & PX4IO_P_SETUP_ARMING_FMU_ARMED)
				&& (r_status_flags & PX4IO_P_STATUS_FLAGS_RAW_PWM);

	if (should_arm && !mixer_servos_armed) {
		/* need to arm, but not armed */
		up_pwm_servo_arm(true, 0);
		mixer_servos_armed = true;
		isr_debug(5, "> PWM enabled");

	} else if (!should_arm && mixer_servos_armed) {
		/* armed but need to disarm */
		up_pwm_servo_arm(false, 0);
		mixer_servos_armed = false;
		isr_debug(5, "> PWM disabled");
	}

	if (mixer_servos_armed && should_arm) {
		/* update the servo outputs. */
		for (unsigned i = 0; i < PX4IO_SERVO_COUNT; i++) {
			up_pwm_servo_set(i, r_page_direct_pwm[i]);
		}
	}
}

extern "C" __EXPORT int user_start(int argc, char *argv[])
{
	/* configure the first 8 PWM outputs (i.e. all of them) */
	up_pwm_servo_init(0xff);

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
	syslog(LOG_INFO, "\nPX4IO: starting\n");

	/* default all the LEDs to off while we start */
	LED_AMBER(false);
#if defined(LED_BLUE)
	LED_BLUE(false);
#endif /* LED_BLUE */
	LED_SAFETY(false);
#if defined(LED_GREEN)
	LED_GREEN(false);
#endif /* LED_GREEN */

	/* start the safety button handler */
	safety_button_init();

	/* initialise the control inputs */
	controls_init();

	/* set up the ADC */
	adc_init();

	/* start the FMU interface */
	interface_init();

	struct mallinfo minfo = mallinfo();
	r_page_status[PX4IO_P_STATUS_FREEMEM] = minfo.mxordblk;
	syslog(LOG_INFO, "MEM: free %u, largest %u\n", minfo.mxordblk, minfo.fordblks);

	/* Start the failsafe led init */
	failsafe_led_init();

	/*
	 * Run everything in a tight loop.
	 */

	uint64_t last_debug_time = 0;
	uint64_t last_heartbeat_time = 0;

	watchdog_init();

	for (;;) {
		watchdog_pet();

		mixer_tick();
		controls_tick();

		/*
		blink blue LED at 4Hz in normal operation. When in
		override blink 4x faster so the user can clearly see
		that override is happening. This helps when
		pre-flight testing the override system
		*/
		uint32_t heartbeat_period_us = 250 * 1000UL;

		if ((hrt_absolute_time() - last_heartbeat_time) > heartbeat_period_us) {
			last_heartbeat_time = hrt_absolute_time();
			heartbeat_blink();
		}

#if defined(HEATER_OUTPUT_EN)

		if (r_page_setup[PX4IO_P_SETUP_THERMAL] != PX4IO_THERMAL_IGNORE) {
			if (r_page_setup[PX4IO_P_SETUP_THERMAL] < PX4IO_THERMAL_FULL) {
				/* switch resistive heater off */
				HEATER_OUTPUT_EN(false);

			} else {
				/* switch resistive heater hard on */
				HEATER_OUTPUT_EN(true);
			}
		}

#endif /* HEATER_OUTPUT_EN */

		update_mem_usage();

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
