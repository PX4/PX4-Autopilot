/****************************************************************************
 *
 *   Copyright (c) 2013, 2017 PX4 Development Team. All rights reserved.
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

#include <px4_config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <limits.h>
#include <math.h>

#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/pwm_limit/pwm_limit.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <px4iofirmware/mixer.h>
#include <px4iofirmware/protocol.h>

#include <uORB/topics/actuator_controls.h>

#include "tests_main.h"

#include <unit_test/unit_test.h>

static int	mixer_callback(uintptr_t handle,
			       uint8_t control_group,
			       uint8_t control_index,
			       float &control);

const unsigned output_max = 8;
static float actuator_controls[output_max];
static bool should_prearm = false;

#define NAN_VALUE 0.0f/0.0f

#ifdef __PX4_DARWIN
#define MIXER_DIFFERENCE_THRESHOLD 30
#else
#define MIXER_DIFFERENCE_THRESHOLD 2
#endif

#ifndef PATH_MAX
#ifdef __PX4_NUTTX
#define PATH_MAX 512
#else
#define PATH_MAX 4096
#endif
#endif

#if defined(CONFIG_ARCH_BOARD_SITL)
#define MIXER_PATH(_file)  "ROMFS/px4fmu_test/mixers/"#_file
#define MIXER_ONBOARD_PATH "ROMFS/px4fmu_common/mixers"
#else
#define MIXER_ONBOARD_PATH "/etc/mixers"
#define MIXER_PATH(_file) MIXER_ONBOARD_PATH"/"#_file
#endif

#define MIXER_VERBOSE

class MixerTest : public UnitTest
{
public:
	virtual bool run_tests();
	MixerTest();

private:
	bool mixerTest();
	bool loadIOPass();
	bool loadVTOL1Test();
	bool loadVTOL2Test();
	bool loadQuadTest();
	bool loadComplexTest();
	bool loadAllTest();
	bool load_mixer(const char *filename, unsigned expected_count, bool verbose = false);
	bool load_mixer(const char *filename, const char *buf, unsigned loaded, unsigned expected_count,
			const unsigned chunk_size, bool verbose);
#if defined(MIXER_TUNING)
	bool tuningTest();
	bool storeMixerTest();
	bool store_mixer_test(const char *filename);
#endif

	MixerGroup mixer_group;
};

MixerTest::MixerTest() : UnitTest(),
	mixer_group(mixer_callback, 0)
{
}

bool MixerTest::run_tests()
{
	ut_run_test(loadIOPass);
	ut_run_test(loadQuadTest);
	ut_run_test(loadVTOL1Test);
	ut_run_test(loadVTOL2Test);
	ut_run_test(loadComplexTest);
	ut_run_test(loadAllTest);
	ut_run_test(mixerTest);
#if defined(MIXER_TUNING)
	ut_run_test(tuningTest);
	ut_run_test(storeMixerTest);
#endif

	return (_tests_failed == 0);
}

ut_declare_test_c(test_mixer, MixerTest)

bool MixerTest::loadIOPass()
{
	return load_mixer(MIXER_PATH(IO_pass.mix), 8);
}

bool MixerTest::loadQuadTest()
{
	return load_mixer(MIXER_PATH(quad_test.mix), 5);
}

bool MixerTest::loadVTOL1Test()
{
	return load_mixer(MIXER_PATH(vtol1_test.mix), 4);
}

bool MixerTest::loadVTOL2Test()
{
	return load_mixer(MIXER_PATH(vtol2_test.mix), 6);
}

bool MixerTest::loadComplexTest()
{
	return load_mixer(MIXER_PATH(complex_test.mix), 8);
}

bool MixerTest::loadAllTest()
{
	PX4_INFO("Testing all mixers in %s", MIXER_ONBOARD_PATH);

	DIR *dp = opendir(MIXER_ONBOARD_PATH);

	if (dp == nullptr) {
		PX4_ERR("File open failed");
		// this is not an FTP error, abort directory by simulating eof
		return false;
	}

	struct dirent *result = nullptr;

	// move to the requested offset
	//seekdir(dp, payload->offset);

	for (;;) {
		errno = 0;
		result = readdir(dp);

		// read the directory entry
		if (result == nullptr) {
			if (errno) {
				PX4_ERR("readdir failed");
				closedir(dp);

				return false;
			}

			// We are just at the last directory entry
			break;
		}

		// Determine the directory entry type
		switch (result->d_type) {
#ifdef __PX4_NUTTX

		case DTYPE_FILE:
#else
		case DT_REG:
#endif
			if (strncmp(result->d_name, ".", 1) != 0) {

				char buf[PATH_MAX];
				(void)strncpy(&buf[0], MIXER_ONBOARD_PATH, sizeof(buf) - 1);
				/* enforce null termination */
				buf[sizeof(buf) - 1] = '\0';
				(void)strncpy(&buf[strlen(MIXER_ONBOARD_PATH)], "/", 1);
				(void)strncpy(&buf[strlen(MIXER_ONBOARD_PATH) + 1], result->d_name, sizeof(buf) - strlen(MIXER_ONBOARD_PATH) - 1);

				bool ret = load_mixer(buf, 0);

				if (!ret) {
					PX4_ERR("Error testing mixer %s", buf);
					return false;
				}
			}

			break;

		default:
			break;
		}
	}

	closedir(dp);

	return true;
}

bool MixerTest::load_mixer(const char *filename, unsigned expected_count, bool verbose)
{
	char buf[2048];

	load_mixer_file(filename, &buf[0], sizeof(buf));
	unsigned loaded = strlen(buf);

	if (verbose) {
		PX4_INFO("loaded: \n\"%s\"\n (file: %s, %d chars)", &buf[0], filename, loaded);
	}

	// Test a number of chunk sizes
	for (unsigned chunk_size = 6; chunk_size < PX4IO_MAX_TRANSFER_LEN + 1; chunk_size++) {
		bool ret = load_mixer(filename, buf, loaded, expected_count, chunk_size, verbose);

		if (!ret) {
			PX4_ERR("Mixer load failed with chunk size %u", chunk_size);
			return ret;
		}
	}

	return true;
}

bool MixerTest::load_mixer(const char *filename, const char *buf, unsigned loaded, unsigned expected_count,
			   const unsigned chunk_size,
			   bool verbose)
{


	/* load the mixer in chunks, like
	 * in the case of a remote load,
	 * e.g. on PX4IO.
	 */

	/* load at once test */
	unsigned xx = loaded;
	mixer_group.reset();
	mixer_group.load_from_buf(&buf[0], xx);

	if (expected_count > 0) {
		ut_compare("check number of mixers loaded", mixer_group.count(), expected_count);
	}

	unsigned empty_load = 2;
	char empty_buf[2];
	empty_buf[0] = ' ';
	empty_buf[1] = '\0';
	mixer_group.reset();
	mixer_group.load_from_buf(&empty_buf[0], empty_load);

	if (verbose) {
		PX4_INFO("empty buffer load: loaded %u mixers, used: %u", mixer_group.count(), empty_load);
	}

	ut_compare("empty buffer load", empty_load, 0);

	/* reset, load in chunks */
	mixer_group.reset();
	char mixer_text[PX4IO_MAX_MIXER_LENGHT];		/* large enough for one mixer */

	unsigned mixer_text_length = 0;
	unsigned transmitted = 0;
	unsigned resid = 0;

	while (transmitted < loaded) {

		unsigned text_length = (loaded - transmitted > chunk_size) ? chunk_size : loaded - transmitted;

		/* check for overflow - this would be really fatal */
		if ((mixer_text_length + text_length + 1) > sizeof(mixer_text)) {
			PX4_ERR("Mixer text length overflow for file: %s. Is PX4IO_MAX_MIXER_LENGHT too small? (curr len: %d)", filename,
				PX4IO_MAX_MIXER_LENGHT);
			return false;
		}

		/* append mixer text and nul-terminate */
		memcpy(&mixer_text[mixer_text_length], &buf[transmitted], text_length);
		mixer_text_length += text_length;
		mixer_text[mixer_text_length] = '\0';
		//fprintf(stderr, "buflen %u, text:\n\"%s\"\n", mixer_text_length, &mixer_text[0]);

		/* process the text buffer, adding new mixers as their descriptions can be parsed */
		resid = mixer_text_length;
		mixer_group.load_from_buf(&mixer_text[0], resid);

		/* if anything was parsed */
		if (resid != mixer_text_length) {
			//PX4_INFO("loaded %d mixers, used %u\n", mixer_group.count(), mixer_text_length - resid);

			/* copy any leftover text to the base of the buffer for re-use */
			if (resid > 0) {
				memmove(&mixer_text[0], &mixer_text[mixer_text_length - resid], resid);
				/* enforce null termination */
				mixer_text[resid] = '\0';
			}

			mixer_text_length = resid;
		}

		transmitted += text_length;

		if (verbose) {
			PX4_INFO("transmitted: %d, loaded: %d", transmitted, loaded);
		}
	}

	if (verbose) {
		PX4_INFO("chunked load: loaded %u mixers", mixer_group.count());
	}

	if (expected_count > 0 && mixer_group.count() != expected_count) {
		PX4_ERR("Load of mixer failed, last chunk: %s, transmitted: %u, text length: %u, resid: %u", mixer_text, transmitted,
			mixer_text_length, resid);
		ut_compare("check number of mixers loaded (chunk)", mixer_group.count(), expected_count);
	}

	return true;
}

bool MixerTest::mixerTest()
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

	bool load_ok = load_mixer(MIXER_PATH(IO_pass.mix), 8);

	if (!load_ok) {
		return load_ok;
	}

	/* execute the mixer */

	float	outputs[output_max];
	unsigned mixed;
	const int jmax = 5;

	pwm_limit_init(&pwm_limit);

	/* run through arming phase */
	for (unsigned i = 0; i < output_max; i++) {
		actuator_controls[i] = 0.1f;
		r_page_servo_disarmed[i] = PWM_MOTOR_OFF;
		r_page_servo_control_min[i] = PWM_DEFAULT_MIN;
		r_page_servo_control_max[i] = PWM_DEFAULT_MAX;
	}

	//PX4_INFO("PRE-ARM TEST: DISABLING SAFETY");

	/* mix */
	should_prearm = true;
	mixed = mixer_group.mix(&outputs[0], output_max, nullptr);

	pwm_limit_calc(should_arm, should_prearm, mixed, reverse_pwm_mask, r_page_servo_disarmed, r_page_servo_control_min,
		       r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

	//warnx("mixed %d outputs (max %d), values:", mixed, output_max);
	for (unsigned i = 0; i < mixed; i++) {

		//fprintf(stderr, "pre-arm:\t %d: out: %8.4f, servo: %d \n", i, (double)outputs[i], (int)r_page_servos[i]);

		if (i != actuator_controls_s::INDEX_THROTTLE) {
			if (r_page_servos[i] < r_page_servo_control_min[i]) {
				warnx("active servo < min");
				return false;
			}

		} else {
			if (r_page_servos[i] != r_page_servo_disarmed[i]) {
				warnx("throttle output != 0 (this check assumed the IO pass mixer!)");
				return false;
			}
		}
	}

	should_arm = true;
	should_prearm = false;

	/* simulate another orb_copy() from actuator controls */
	for (unsigned i = 0; i < output_max; i++) {
		actuator_controls[i] = 0.1f;
	}

	//PX4_INFO("ARMING TEST: STARTING RAMP");
	unsigned sleep_quantum_us = 10000;

	hrt_abstime starttime = hrt_absolute_time();
	unsigned sleepcount = 0;

	while (hrt_elapsed_time(&starttime) < INIT_TIME_US + RAMP_TIME_US + 2 * sleep_quantum_us) {

		/* mix */
		mixed = mixer_group.mix(&outputs[0], output_max, nullptr);

		pwm_limit_calc(should_arm, should_prearm, mixed, reverse_pwm_mask, r_page_servo_disarmed, r_page_servo_control_min,
			       r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

		//warnx("mixed %d outputs (max %d), values:", mixed, output_max);
		for (unsigned i = 0; i < mixed; i++) {

			//fprintf(stderr, "ramp:\t %d: out: %8.4f, servo: %d \n", i, (double)outputs[i], (int)r_page_servos[i]);

			/* check mixed outputs to be zero during init phase */
			if (hrt_elapsed_time(&starttime) < INIT_TIME_US &&
			    r_page_servos[i] != r_page_servo_disarmed[i]) {
				PX4_ERR("disarmed servo value mismatch: %d vs %d", r_page_servos[i], r_page_servo_disarmed[i]);
				return false;
			}

			if (hrt_elapsed_time(&starttime) >= INIT_TIME_US &&
			    r_page_servos[i] + 1 <= r_page_servo_disarmed[i]) {
				PX4_ERR("ramp servo value mismatch");
				return false;
			}
		}

		usleep(sleep_quantum_us);
		sleepcount++;

		if (sleepcount % 10 == 0) {
			fflush(stdout);
		}
	}

	//PX4_INFO("ARMING TEST: NORMAL OPERATION");

	for (int j = -jmax; j <= jmax; j++) {

		for (unsigned i = 0; i < output_max; i++) {
			actuator_controls[i] = j / 10.0f + 0.1f * i;
			r_page_servo_disarmed[i] = PWM_LOWEST_MIN;
			r_page_servo_control_min[i] = PWM_DEFAULT_MIN;
			r_page_servo_control_max[i] = PWM_DEFAULT_MAX;
		}

		/* mix */
		mixed = mixer_group.mix(&outputs[0], output_max, nullptr);

		pwm_limit_calc(should_arm, should_prearm, mixed, reverse_pwm_mask, r_page_servo_disarmed, r_page_servo_control_min,
			       r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

		//fprintf(stderr, "mixed %d outputs (max %d)", mixed, output_max);

		for (unsigned i = 0; i < mixed; i++) {
			servo_predicted[i] = 1500 + outputs[i] * (r_page_servo_control_max[i] - r_page_servo_control_min[i]) / 2.0f;

			if (abs(servo_predicted[i] - r_page_servos[i]) > MIXER_DIFFERENCE_THRESHOLD) {
				fprintf(stderr, "\t %d: %8.4f predicted: %d, servo: %d\n", i, (double)outputs[i], servo_predicted[i],
					(int)r_page_servos[i]);
				PX4_ERR("mixer violated predicted value");
				return false;
			}
		}
	}

	//PX4_INFO("ARMING TEST: DISARMING");

	starttime = hrt_absolute_time();
	sleepcount = 0;
	should_arm = false;

	while (hrt_elapsed_time(&starttime) < 600000) {

		/* mix */
		mixed = mixer_group.mix(&outputs[0], output_max, nullptr);

		pwm_limit_calc(should_arm, should_prearm, mixed, reverse_pwm_mask, r_page_servo_disarmed, r_page_servo_control_min,
			       r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

		//warnx("mixed %d outputs (max %d), values:", mixed, output_max);
		for (unsigned i = 0; i < mixed; i++) {

			//fprintf(stderr, "disarmed:\t %d: out: %8.4f, servo: %d \n", i, (double)outputs[i], (int)r_page_servos[i]);

			/* check mixed outputs to be zero during init phase */
			if (r_page_servos[i] != r_page_servo_disarmed[i]) {
				PX4_ERR("disarmed servo value mismatch");
				return false;
			}
		}

		usleep(sleep_quantum_us);
		sleepcount++;

		if (sleepcount % 10 == 0) {
			//printf(".");
			//fflush(stdout);
		}
	}

	//printf("\n");

	//PX4_INFO("ARMING TEST: REARMING: STARTING RAMP");

	starttime = hrt_absolute_time();
	sleepcount = 0;
	should_arm = true;

	while (hrt_elapsed_time(&starttime) < 600000 + RAMP_TIME_US) {

		/* mix */
		mixed = mixer_group.mix(&outputs[0], output_max, nullptr);

		pwm_limit_calc(should_arm, should_prearm, mixed, reverse_pwm_mask, r_page_servo_disarmed, r_page_servo_control_min,
			       r_page_servo_control_max, outputs, r_page_servos, &pwm_limit);

		//warnx("mixed %d outputs (max %d), values:", mixed, output_max);
		for (unsigned i = 0; i < mixed; i++) {
			/* predict value */
			servo_predicted[i] = 1500 + outputs[i] * (r_page_servo_control_max[i] - r_page_servo_control_min[i]) / 2.0f;

			/* check ramp */

			//fprintf(stderr, "ramp:\t %d: out: %8.4f, servo: %d \n", i, (double)outputs[i], (int)r_page_servos[i]);

			if (hrt_elapsed_time(&starttime) < RAMP_TIME_US &&
			    (r_page_servos[i] + 1 <= r_page_servo_disarmed[i] ||
			     r_page_servos[i] > servo_predicted[i])) {
				PX4_ERR("ramp servo value mismatch");
				return false;
			}

			/* check post ramp phase */
			if (hrt_elapsed_time(&starttime) > RAMP_TIME_US &&
			    abs(servo_predicted[i] - r_page_servos[i]) > 2) {
				printf("\t %d: %8.4f predicted: %d, servo: %d\n", i, (double)outputs[i], servo_predicted[i], (int)r_page_servos[i]);
				PX4_ERR("mixer violated predicted value");
				return false;
			}
		}

		usleep(sleep_quantum_us);
		sleepcount++;

		if (sleepcount % 10 == 0) {
			//	printf(".");
			//	fflush(stdout);
		}
	}

	return true;
}

static int
mixer_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &control)
{
	if (control_group != 0) {
		return -1;
	}

	if (control_index >= (sizeof(actuator_controls) / sizeof(actuator_controls[0]))) {
		return -1;
	}

	control = actuator_controls[control_index];

	if (should_prearm && control_group == actuator_controls_s::GROUP_INDEX_ATTITUDE &&
	    control_index == actuator_controls_s::INDEX_THROTTLE) {
		control = NAN_VALUE;
	}

	return 0;
}



const mixer_param_s param_test_values[] = {
	//index, type, mix_index, mix_sub_index, values[8], name[21], param_type, array_size, flags
	//Multicopter main mixer
	{0, MIXER_PARAM_MSG_TYPE_MIXTYPE,       0, 0,  {0.000000, 0.000000}, "MULTIROTOR",      0, 0, 1},
	{1, MIXER_PARAM_MSG_TYPE_PARAMETER,     0, 0,  {1.000000, 0.000000}, "IN_ROLL_SCALE",   0, 1, 0},
	{2, MIXER_PARAM_MSG_TYPE_PARAMETER,     0, 0,  {1.000000, 0.000000}, "IN_PITCH_SCALE",  0, 1, 0},
	{3, MIXER_PARAM_MSG_TYPE_PARAMETER,     0, 0,  {1.000000, 0.000000}, "IN_YAW_SCALE",    0, 1, 0},
	{4, MIXER_PARAM_MSG_TYPE_PARAMETER,     0, 0,  {-1.000000, 0.000000}, "IN_IDLE_SPEED",  0, 1, 0},
	//First multicopter submixer
	{5, MIXER_PARAM_MSG_TYPE_MIXTYPE,       0, 1,  {0.000000, 0.000000}, "MULTIROTOR_MOTOR", 0, 0, 1},
	{6, MIXER_PARAM_MSG_TYPE_PARAMETER,     0, 1,  {-0.927184, 0.000000}, "OUT_ROLL_SCALE", 0, 1, 1},
	{7, MIXER_PARAM_MSG_TYPE_PARAMETER,     0, 1,  {0.374607, 0.000000}, "OUT_PITCH_SCALE", 0, 1, 1},
	{8, MIXER_PARAM_MSG_TYPE_PARAMETER,     0, 1,  {1.000000, 0.000000}, "OUT_YAW_SCALE",   0, 1, 1},
	{9, MIXER_PARAM_MSG_TYPE_PARAMETER,     0, 1,  {1.000000, 0.000000}, "OUT_SCALE",       0, 1, 1},
	//Last multicopter submixer
	{20, MIXER_PARAM_MSG_TYPE_MIXTYPE,      0, 4,  {0.000000, 0.000000}, "MULTIROTOR_MOTOR",   0, 0, 1},
	{21, MIXER_PARAM_MSG_TYPE_PARAMETER,    0, 4,  {-0.777146, 0.000000}, "OUT_ROLL_SCALE",    0, 1, 1},
	{22, MIXER_PARAM_MSG_TYPE_PARAMETER,    0, 4,  {-0.629320, 0.000000}, "OUT_PITCH_SCALE",   0, 1, 1},
	{23, MIXER_PARAM_MSG_TYPE_PARAMETER,    0, 4,  {-1.000000, 0.000000}, "OUT_YAW_SCALE",     0, 1, 1},
	{24, MIXER_PARAM_MSG_TYPE_PARAMETER,    0, 4,  {1.000000, 0.000000},  "OUT_SCALE",         0, 1, 1},
	//First SimpleMixer main output mixer
	{25, MIXER_PARAM_MSG_TYPE_MIXTYPE,      1, 0,  {1.000000, 0.000000}, "SIMPLE",          0, 0, 1},
	{26, MIXER_PARAM_MSG_TYPE_PARAMETER,    1, 0,  {1.000000, 0.000000}, "OUT_NEG_SCALE",   0, 1, 0},
	{27, MIXER_PARAM_MSG_TYPE_PARAMETER,    1, 0,  {1.000000, 0.000000}, "OUT_POS_SCALE",   0, 1, 0},
	{28, MIXER_PARAM_MSG_TYPE_PARAMETER,    1, 0,  {0.000000, 0.000000}, "OUT_OFFSET",      0, 1, 0},
	{29, MIXER_PARAM_MSG_TYPE_PARAMETER,    1, 0,  {-1.000000, 0.000000}, "MIN_OUTPUT",      0, 1, 0},
	{30, MIXER_PARAM_MSG_TYPE_PARAMETER,    1, 0,  {1.000000, 0.000000}, "MAX_OUTPUT",      0, 1, 0},
	// First SimpleMixer input submixer
	{31, MIXER_PARAM_MSG_TYPE_MIXTYPE,      1, 1,  {0.000000, 0.000000}, "SIMPLE_INPUT",    0, 0, 1},
	{32, MIXER_PARAM_MSG_TYPE_MIX_CONN,     1, 1,  {0.000000, 4.000000}, "INPUT",           0, 2, 1},
	{33, MIXER_PARAM_MSG_TYPE_PARAMETER,    1, 1,  {1.000000, 0.000000}, "IN_NEG_SCALE",    0, 1, 0},
	{34, MIXER_PARAM_MSG_TYPE_PARAMETER,    1, 1,  {1.000000, 0.000000}, "IN_POS_SCALE",    0, 1, 0},
	{35, MIXER_PARAM_MSG_TYPE_PARAMETER,    1, 1,  {0.000000, 0.000000}, "IN_OFFSET",       0, 1, 0},
	{36, MIXER_PARAM_MSG_TYPE_PARAMETER,    1, 1,  {-1.000000, 0.000000}, "MIN_INPUT",       0, 1, 0},
	{37, MIXER_PARAM_MSG_TYPE_PARAMETER,    1, 1,  {1.000000, 0.000000}, "MAX_INPUT",       0, 1, 0},
	//First param of remaining mixers
	{38, MIXER_PARAM_MSG_TYPE_MIXTYPE,      2, 0,  {1.000000, 0.000000}, "SIMPLE",          0, 0, 1},
	{51, MIXER_PARAM_MSG_TYPE_MIXTYPE,      3, 0,  {1.000000, 0.000000}, "SIMPLE",          0, 0, 1},
	{64, MIXER_PARAM_MSG_TYPE_MIXTYPE,      4, 0,  {1.000000, 0.000000}, "SIMPLE",          0, 0, 1},
	//Last param of last mixer
	{76, MIXER_PARAM_MSG_TYPE_PARAMETER,    4, 1,  {1.000000, 0.000000}, "MAX_INPUT",       0, 1, 0}
};


#if defined(MIXER_TUNING)
bool MixerTest::tuningTest()
{
#if defined(MIXER_VERBOSE)
	const bool verbose = true;
#else
	const bool verbose = false;
#endif
	char buf[1024];
	const char *mixpath = MIXER_PATH(quad_test.mix);
	const int expected_mixer_count = 5;
	const int expected_param_count = 77;

	mixer_param_s param;

	load_mixer_file(mixpath, &buf[0], sizeof(buf));
	unsigned loaded = strlen(buf);

	if (verbose) {
		PX4_INFO("Mixer tuning test resetting and loading mixers from %s", mixpath);
	}

	mixer_group.reset();
	mixer_group.load_from_buf(&buf[0], loaded);

	int mixer_count = mixer_group.count();

	if (verbose) {
		PX4_INFO("Mixer tuning test - mixer count expected: %d - found %d mixers", expected_mixer_count, mixer_count);
	}

	if (mixer_count != expected_mixer_count) {
		return false;
	}

	int param_count = mixer_group.group_param_count();

	if (verbose) {
		PX4_INFO("Mixer tuning test - param count expected: %d - found %d parameters", expected_param_count, param_count);
	}

	if (param_count != expected_param_count) {
		return false;
	}

	//Check all parameters can be read
	for (int i = 0; i < param_count; i++) {

		memset(&param, 0, sizeof(param));
		param.index = i;

		if (mixer_group.group_get_param(&param) < 0) {
			if (verbose) {
				PX4_INFO("Mixer tuning test - could not get parameter at index:%d", i);
			}

			return false;
		}

		if (verbose) {
			PX4_INFO(" - Index:%d MixIndex:%d SubIndex:%d Type:%d ParamType:%d ArraySize:%d Flags:%u name:%s values[0]:%f values[1]:%f",
				 param.index, param.mix_index, param.mix_sub_index, param.type, param.param_type,
				 param.array_size, param.flags, param.name, (double) param.values[0], (double) param.values[1]);
		}
	}

	const int param_test_val_count = sizeof(param_test_values) / sizeof(mixer_param_s);
	PX4_INFO("Mixer tuning test - Testing values in mixer against %d test values", param_test_val_count);

	for (int i = 0; i < param_test_val_count; i++) {
		memset(&param, 0, sizeof(param));
		param.index = param_test_values[i].index;
		mixer_group.group_get_param(&param);

		if (param.index != param_test_values[i].index) {
			if (verbose) {
				PX4_INFO("Mixer tuning test - parameter index was not as expected at index:%d", param_test_values[i].index);
			}

			return false;
		}

		if (param.mix_index != param_test_values[i].mix_index) {
			if (verbose) {
				PX4_INFO("Mixer tuning test - parameter mix_index was not as expected at index:%d", param_test_values[i].index);
			}

			return false;
		}

		if (param.mix_sub_index != param_test_values[i].mix_sub_index) {
			if (verbose) {
				PX4_INFO("Mixer tuning test - parameter mix_sub_index was not as expected at index:%d", param_test_values[i].index);
			}

			return false;
		}

		if (param.type != param_test_values[i].type) {
			if (verbose) {
				PX4_INFO("Mixer tuning test - parameter type was not as expected at index:%d", param_test_values[i].index);
			}

			return false;
		}

		if (param.param_type != param_test_values[i].param_type) {
			if (verbose) {
				PX4_INFO("Mixer tuning test - parameter param_type was not as expected at index:%d", param_test_values[i].index);
			}

			return false;
		}

		if (param.array_size != param_test_values[i].array_size) {
			if (verbose) {
				PX4_INFO("Mixer tuning test - parameter array_size was not as expected at index:%d", param_test_values[i].index);
			}

			return false;
		}

		if (strncmp(param.name, param_test_values[i].name, 16) != 0) {
			if (verbose) {
				PX4_INFO("Mixer tuning test - parameter name was not as expected");
			}

			return false;
		}

		if (param.flags != param_test_values[i].flags) {
			if (verbose) {
				PX4_INFO("Mixer tuning test - parameter array_size was not as expected");
			}

			return false;
		}

		//Value comparison to go here
	}

	//Check for out of bounds paramter access
	memset(&param, 0, sizeof(param));
	param.index = expected_param_count;
	mixer_group.group_get_param(&param);

	if (mixer_group.group_get_param(&param) >= 0) {
		if (verbose) {
			PX4_INFO("Mixer tuning test - Should have failure code returned at mixer index :%d", expected_param_count);
		}

		return false;
	}

	if ((param.flags & 0x80) == 0) {
		if (verbose) {
			PX4_INFO("Mixer tuning test - Should have failure code set in flags at mixer index :%d", expected_param_count);
		}

		return false;
	}


	// Check parameters can be set or not set according to read only status.
	for (int i = 0; i < param_test_val_count; i++) {
		memset(&param, 0, sizeof(param));
		param.index = param_test_values[i].index;
		mixer_group.group_get_param(&param);
		param.values[0] = (float) i + 2;
		int result = mixer_group.group_set_param(&param);
		bool read_only = ((param_test_values[i].flags & 0x01) == 1);

		//Test if read only parameters are reporting a write failure
		if ((read_only && (result == 0))) {
			PX4_INFO("Mixer tuning test - Returned a valid write result on a read only register index:%d", param.index);
		}

		// Read back to verify the set value
		param.values[0] = -1.0;
		mixer_group.group_get_param(&param);

		// Test if read/write params are written and read only are not written;
		if (!read_only) {
			if (param.values[0] != (float) i + 2) {
				if (verbose) {
					PX4_INFO("Mixer tuning test - Read/Write parameter did not read back correct at index:%d", param.index);
				}

				return -1;
			}

		} else {
			if (param.values[0] == (float) i + 2) {
				if (verbose) {
					PX4_INFO("Mixer tuning test - Read only parameter read back written (incorrect) value at index:%d", param.index);
				}

				return -1;
			}
		}
	}

	return true;
}

bool MixerTest::storeMixerTest()
{
	if (!store_mixer_test(MIXER_PATH(quad_test.mix))) {
		return false;
	}

	if (!store_mixer_test(MIXER_PATH(complex_test.mix))) {
		return false;
	}

	return true;
}

bool
MixerTest::store_mixer_test(const char *filename)
{
#if defined(MIXER_VERBOSE)
	const bool verbose = true;
#else
	const bool verbose = false;
#endif
	char loadbuf[1024];
	char savebuf[1024];

	if (load_mixer_file(filename, &loadbuf[0], sizeof(loadbuf)) < 0) {
		if (verbose) {
			PX4_INFO("Mixer tuning test - Failed to load mixer file : %s", filename);
		}

		return false;
	}

	unsigned loaded = strlen(loadbuf);
	unsigned original = loaded;

	if (verbose) {
		PX4_INFO("Mixer tuning test - Resetting and loading mixers from %s", filename);
	}

	mixer_group.reset();
	mixer_group.load_from_buf(&loadbuf[0], loaded);

	memset(&    savebuf, 0, sizeof(savebuf));
	unsigned saved = sizeof(savebuf);

	if (mixer_group.save_to_buf(&savebuf[0], saved) < 0) {
		PX4_INFO("Mixer tuning test - Save to buffer returned error");
		return false;
	}

	if (saved != strlen(savebuf)) {
		if (verbose) {
			PX4_INFO("Mixer tuning test - save to buf returned buffer size not in agreement with strlen");
		}

		return false;
	}

	if (verbose) {
		PX4_INFO("Mixer tuning test - Load buffer size:%d - Save buffer size:%d", original, saved);
	}

	if (saved != original) {
		return false;
	}

	if (strncmp(loadbuf, savebuf, original) != 0) {
		if (verbose) {
			PX4_INFO("Mixer tuning test - Saved buffer did not match loaded buffer");
		}

		return false;
	}

	return true;
}

#endif  //(MIXER_TUNING)
