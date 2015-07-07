/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file test_mixer.hpp
 *
 * Mixer load test
 */

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/pwm_limit/pwm_limit.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

#include <uORB/topics/actuator_controls.h>

#include "tests.h"

static int	mixer_callback(uintptr_t handle,
			       uint8_t control_group,
			       uint8_t control_index,
			       float &control);

const unsigned output_max = 8;
static float actuator_controls[output_max];
static bool should_prearm = false;

#define NAN_VALUE 0.0f/0.0f

int test_mixer(int argc, char *argv[])
{
	/*
	 * PWM limit structure
	 */
	pwm_limit_t pwm_limit;
	bool should_arm = false;
	uint16_t r_page_servo_disarmed[output_max];
	uint16_t r_page_servo_control_min[output_max];
	uint16_t r_page_servo_control_max[output_max];
	uint16_t r_page_servos[output_max];
	uint16_t servo_predicted[output_max];
	int16_t reverse_pwm_mask = 0;

	warnx("testing mixer");

	const char *filename = "/etc/mixers/IO_pass.mix";

	if (argc > 1) {
		filename = argv[1];
	}

	warnx("loading: %s", filename);

	char		buf[2048];

	load_mixer_file(filename, &buf[0], sizeof(buf));
	unsigned loaded = strlen(buf);

	warnx("loaded: \n\"%s\"\n (%d chars)", &buf[0], loaded);

	/* load the mixer in chunks, like
	 * in the case of a remote load,
	 * e.g. on PX4IO.
	 */

	const unsigned chunk_size = 64;

	MixerGroup mixer_group(mixer_callback, 0);

	/* load at once test */
	unsigned xx = loaded;
	mixer_group.load_from_buf(&buf[0], xx);
	warnx("complete buffer load: loaded %u mixers", mixer_group.count());

	if (mixer_group.count() != 8) {
		return 1;
	}

	unsigned empty_load = 2;
	char empty_buf[2];
	empty_buf[0] = ' ';
	empty_buf[1] = '\0';
	mixer_group.reset();
	mixer_group.load_from_buf(&empty_buf[0], empty_load);
	warnx("empty buffer load: loaded %u mixers, used: %u", mixer_group.count(), empty_load);

	if (empty_load != 0) {
		return 1;
	}

	/* FIRST mark the mixer as invalid */
	/* THEN actually delete it */
	mixer_group.reset();
	char mixer_text[256];		/* large enough for one mixer */
	unsigned mixer_text_length = 0;

	unsigned transmitted = 0;

	warnx("transmitted: %d, loaded: %d", transmitted, loaded);

	while (transmitted < loaded) {

		unsigned	text_length = (loaded - transmitted > chunk_size) ? chunk_size : loaded - transmitted;

		/* check for overflow - this would be really fatal */
		if ((mixer_text_length + text_length + 1) > sizeof(mixer_text)) {
			return 1;
		}

		/* append mixer text and nul-terminate */
		memcpy(&mixer_text[mixer_text_length], &buf[transmitted], text_length);
		mixer_text_length += text_length;
		mixer_text[mixer_text_length] = '\0';
		warnx("buflen %u, text:\n\"%s\"", mixer_text_length, &mixer_text[0]);

		/* process the text buffer, adding new mixers as their descriptions can be parsed */
		unsigned resid = mixer_text_length;
		mixer_group.load_from_buf(&mixer_text[0], resid);

		/* if anything was parsed */
		if (resid != mixer_text_length) {
			warnx("used %u", mixer_text_length - resid);

			/* copy any leftover text to the base of the buffer for re-use */
			if (resid > 0) {
				memcpy(&mixer_text[0], &mixer_text[mixer_text_length - resid], resid);
			}

			mixer_text_length = resid;
		}

		transmitted += text_length;
	}

	warnx("chunked load: loaded %u mixers", mixer_group.count());

	if (mixer_group.count() != 8) {
		return 1;
	}

	/* execute the mixer */

	float	outputs[output_max];
	unsigned mixed;
	const int jmax = 5;

	pwm_limit_init(&pwm_limit);

	/* run through arming phase */
	for (unsigned i = 0; i < output_max; i++) {
		actuator_controls[i] = 0.1f;
		r_page_servo_disarmed[i] = PWM_LOWEST_MIN;
		r_page_servo_control_min[i] = PWM_DEFAULT_MIN;
		r_page_servo_control_max[i] = PWM_DEFAULT_MAX;
	}

	warnx("PRE-ARM TEST: DISABLING SAFETY");
	/* mix */
	should_prearm = true;
	mixed = mixer_group.mix(&outputs[0], output_max, NULL);

	pwm_limit_calc(should_arm, should_prearm, mixed, reverse_pwm_mask, r_page_servo_disarmed, r_page_servo_control_min,
		r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

	//warnx("mixed %d outputs (max %d), values:", mixed, output_max);
	for (unsigned i = 0; i < mixed; i++) {

		warnx("pre-arm:\t %d: out: %8.4f, servo: %d", i, (double)outputs[i], (int)r_page_servos[i]);

		if (i != actuator_controls_s::INDEX_THROTTLE) {
			if (r_page_servos[i] < r_page_servo_control_min[i]) {
				warnx("active servo < min");
				return 1;
			}
		} else {
			if (r_page_servos[i] != r_page_servo_disarmed[i]) {
				warnx("throttle output != 0 (this check assumed the IO pass mixer!)");
				return 1;
			}
		}
	}

	should_arm = true;
	should_prearm = false;

	/* simulate another orb_copy() from actuator controls */
	for (unsigned i = 0; i < output_max; i++) {
		actuator_controls[i] = 0.1f;
	}

	warnx("ARMING TEST: STARTING RAMP");
	unsigned sleep_quantum_us = 10000;

	hrt_abstime starttime = hrt_absolute_time();
	unsigned sleepcount = 0;

	while (hrt_elapsed_time(&starttime) < INIT_TIME_US + RAMP_TIME_US + 2 * sleep_quantum_us) {

		/* mix */
		mixed = mixer_group.mix(&outputs[0], output_max, NULL);

		pwm_limit_calc(should_arm, should_prearm, mixed, reverse_pwm_mask, r_page_servo_disarmed, r_page_servo_control_min,
			r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

		//warnx("mixed %d outputs (max %d), values:", mixed, output_max);
		for (unsigned i = 0; i < mixed; i++) {

			warnx("ramp:\t %d: out: %8.4f, servo: %d", i, (double)outputs[i], (int)r_page_servos[i]);

			/* check mixed outputs to be zero during init phase */
			if (hrt_elapsed_time(&starttime) < INIT_TIME_US &&
			    r_page_servos[i] != r_page_servo_disarmed[i]) {
				warnx("disarmed servo value mismatch: %d vs %d", r_page_servos[i], r_page_servo_disarmed[i]);
				return 1;
			}

			if (hrt_elapsed_time(&starttime) >= INIT_TIME_US &&
			    r_page_servos[i] + 1 <= r_page_servo_disarmed[i]) {
				warnx("ramp servo value mismatch");
				return 1;
			}
		}

		usleep(sleep_quantum_us);
		sleepcount++;

		if (sleepcount % 10 == 0) {
			printf(".");
			fflush(stdout);
		}
	}

	printf("\n");

	warnx("ARMING TEST: NORMAL OPERATION");

	for (int j = -jmax; j <= jmax; j++) {

		for (unsigned i = 0; i < output_max; i++) {
			actuator_controls[i] = j / 10.0f + 0.1f * i;
			r_page_servo_disarmed[i] = PWM_LOWEST_MIN;
			r_page_servo_control_min[i] = PWM_DEFAULT_MIN;
			r_page_servo_control_max[i] = PWM_DEFAULT_MAX;
		}

		/* mix */
		mixed = mixer_group.mix(&outputs[0], output_max, NULL);

		pwm_limit_calc(should_arm, should_prearm, mixed, reverse_pwm_mask, r_page_servo_disarmed, r_page_servo_control_min, r_page_servo_control_max, outputs,
			       r_page_servos, &pwm_limit);

		warnx("mixed %d outputs (max %d)", mixed, output_max);

		for (unsigned i = 0; i < mixed; i++) {
			servo_predicted[i] = 1500 + outputs[i] * (r_page_servo_control_max[i] - r_page_servo_control_min[i]) / 2.0f;

			if (abs(servo_predicted[i] - r_page_servos[i]) > 2) {
				printf("\t %d: %8.4f predicted: %d, servo: %d\n", i, (double)outputs[i], servo_predicted[i], (int)r_page_servos[i]);
				warnx("mixer violated predicted value");
				return 1;
			}
		}
	}

	warnx("ARMING TEST: DISARMING");

	starttime = hrt_absolute_time();
	sleepcount = 0;
	should_arm = false;

	while (hrt_elapsed_time(&starttime) < 600000) {

		/* mix */
		mixed = mixer_group.mix(&outputs[0], output_max, NULL);

		pwm_limit_calc(should_arm, should_prearm, mixed, reverse_pwm_mask, r_page_servo_disarmed, r_page_servo_control_min, r_page_servo_control_max, outputs,
			       r_page_servos, &pwm_limit);

		//warnx("mixed %d outputs (max %d), values:", mixed, output_max);
		for (unsigned i = 0; i < mixed; i++) {

			warnx("disarmed:\t %d: out: %8.4f, servo: %d", i, (double)outputs[i], (int)r_page_servos[i]);

			/* check mixed outputs to be zero during init phase */
			if (r_page_servos[i] != r_page_servo_disarmed[i]) {
				warnx("disarmed servo value mismatch");
				return 1;
			}
		}

		usleep(sleep_quantum_us);
		sleepcount++;

		if (sleepcount % 10 == 0) {
			printf(".");
			fflush(stdout);
		}
	}

	printf("\n");

	warnx("ARMING TEST: REARMING: STARTING RAMP");

	starttime = hrt_absolute_time();
	sleepcount = 0;
	should_arm = true;

	while (hrt_elapsed_time(&starttime) < 600000 + RAMP_TIME_US) {

		/* mix */
		mixed = mixer_group.mix(&outputs[0], output_max, NULL);

		pwm_limit_calc(should_arm, should_prearm, mixed, reverse_pwm_mask, r_page_servo_disarmed, r_page_servo_control_min, r_page_servo_control_max, outputs,
			       r_page_servos, &pwm_limit);

		//warnx("mixed %d outputs (max %d), values:", mixed, output_max);
		for (unsigned i = 0; i < mixed; i++) {
			/* predict value */
			servo_predicted[i] = 1500 + outputs[i] * (r_page_servo_control_max[i] - r_page_servo_control_min[i]) / 2.0f;

			/* check ramp */

			warnx("ramp:\t %d: out: %8.4f, servo: %d", i, (double)outputs[i], (int)r_page_servos[i]);

			if (hrt_elapsed_time(&starttime) < RAMP_TIME_US &&
			    (r_page_servos[i] + 1 <= r_page_servo_disarmed[i] ||
			     r_page_servos[i] > servo_predicted[i])) {
				warnx("ramp servo value mismatch");
				return 1;
			}

			/* check post ramp phase */
			if (hrt_elapsed_time(&starttime) > RAMP_TIME_US &&
			    abs(servo_predicted[i] - r_page_servos[i]) > 2) {
				printf("\t %d: %8.4f predicted: %d, servo: %d\n", i, (double)outputs[i], servo_predicted[i], (int)r_page_servos[i]);
				warnx("mixer violated predicted value");
				return 1;
			}
		}

		usleep(sleep_quantum_us);
		sleepcount++;

		if (sleepcount % 10 == 0) {
			printf(".");
			fflush(stdout);
		}
	}

	printf("\n");

	/* load multirotor at once test */
	mixer_group.reset();

	if (argc > 2) {
		filename = argv[2];

	} else {
		filename = "/etc/mixers/quad_w.main.mix";
	}

	load_mixer_file(filename, &buf[0], sizeof(buf));
	loaded = strlen(buf);

	warnx("loaded: \n\"%s\"\n (%d chars)", &buf[0], loaded);

	unsigned mc_loaded = loaded;
	mixer_group.load_from_buf(&buf[0], mc_loaded);
	warnx("complete buffer load: loaded %u mixers", mixer_group.count());

	if (mixer_group.count() != 5) {
		warnx("FAIL: Quad W mixer load failed");
		return 1;
	}

	warnx("SUCCESS: No errors in mixer test");
	return 0;
}

static int
mixer_callback(uintptr_t handle,
	       uint8_t control_group,
	       uint8_t control_index,
	       float &control)
{
	if (control_group != 0) {
		return -1;
	}

	if (control_index > (sizeof(actuator_controls) / sizeof(actuator_controls[0]))) {
		return -1;
	}

	control = actuator_controls[control_index];

	if (should_prearm && control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
		control_index == actuator_controls_s::INDEX_THROTTLE) {
		control = NAN_VALUE;
	}

	return 0;
}
