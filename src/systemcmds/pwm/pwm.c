/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file pwm.c
 *
 * PWM servo output configuration and monitoring tool.
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <nuttx/i2c.h>
#include <nuttx/mtd.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>

#include <arch/board/board.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "systemlib/param/param.h"
#include "drivers/drv_pwm_output.h"

static void	usage(const char *reason);
__EXPORT int	pwm_main(int argc, char *argv[]);


static void
usage(const char *reason)
{
	if (reason != NULL) {
		warnx("%s", reason);
	}

	errx(1,
	     "usage:\n"
	     "pwm arm|disarm|rate|failsafe|disarmed|min|max|test|info  ...\n"
	     "\n"
	     "arm\t\t\t\tArm output\n"
	     "disarm\t\t\t\tDisarm output\n"
	     "\n"
	     "rate ...\t\t\tConfigure PWM rates\n"
	     "\t[-g <channel group>]\t(e.g. 0,1,2)\n"
	     "\t[-m <channel mask> ]\t(e.g. 0xF)\n"
	     "\t[-a]\t\t\tConfigure all outputs\n"
	     "\t-r <alt_rate>\t\tPWM rate (50 to 400 Hz)\n"
	     "\n"
	     "failsafe ...\t\t\tFailsafe PWM\n"
	     "disarmed ...\t\t\tDisarmed PWM\n"
	     "min ...\t\t\t\tMinimum PWM\n"
	     "max ...\t\t\t\tMaximum PWM\n"
	     "\t[-c <channels>]\t\t(e.g. 1234)\n"
	     "\t[-m <channel mask> ]\t(e.g. 0xF)\n"
	     "\t[-a]\t\t\tConfigure all outputs\n"
	     "\t-p <pwm value>\t\tPWM value\n"
	     "\n"
	     "test ...\t\t\tDirectly set PWM\n"
	     "\t[-c <channels>]\t\t(e.g. 1234)\n"
	     "\t[-m <channel mask> ]\t(e.g. 0xF)\n"
	     "\t[-a]\t\t\tConfigure all outputs\n"
	     "\t-p <pwm value>\t\tPWM value\n"
	     "\n"
	     "info\t\t\t\tPrint information\n"
	     "\n"
	     "\t-v\t\t\tVerbose\n"
	     "\t-d <dev>\t\t(default " PWM_OUTPUT0_DEVICE_PATH ")\n"
	    );

}

int
pwm_main(int argc, char *argv[])
{
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;
	unsigned alt_rate = 0;
	uint32_t alt_channel_groups = 0;
	bool alt_channels_set = false;
	bool print_verbose = false;
	int ch;
	int ret;
	char *ep;
	uint32_t set_mask = 0;
	unsigned group;
	unsigned long channels;
	unsigned single_ch = 0;
	unsigned pwm_value = 0;

	if (argc < 2) {
		usage(NULL);
	}

	while ((ch = getopt(argc - 1, &argv[1], "d:vc:g:m:ap:r:")) != EOF) {
		switch (ch) {

		case 'd':
			if (NULL == strstr(optarg, "/dev/")) {
				warnx("device %s not valid", optarg);
				usage(NULL);
			}

			dev = optarg;
			break;

		case 'v':
			print_verbose = true;
			break;

		case 'c':
			/* Read in channels supplied as one int and convert to mask: 1234 -> 0xF */
			channels = strtoul(optarg, &ep, 0);

			while ((single_ch = channels % 10)) {

				set_mask |= 1 << (single_ch - 1);
				channels /= 10;
			}

			break;

		case 'g':
			group = strtoul(optarg, &ep, 0);

			if ((*ep != '\0') || (group >= 32)) {
				usage("bad channel_group value");
			}

			alt_channel_groups |= (1 << group);
			alt_channels_set = true;
			warnx("alt channels set, group: %d", group);
			break;

		case 'm':
			/* Read in mask directly */
			set_mask = strtoul(optarg, &ep, 0);

			if (*ep != '\0') {
				usage("BAD set_mask VAL");
			}

			break;

		case 'a':
			for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
				set_mask |= 1 << i;
			}

			break;

		case 'p':
			{
				/* check if this is a param name */
				if (strncmp("p:", optarg, 2) == 0) {

					char buf[32];
					strncpy(buf, optarg + 2, 16);
					/* user wants to use a param name */
					param_t parm = param_find(buf);

					if (parm != PARAM_INVALID) {
						int32_t pwm_parm;
						int gret = param_get(parm, &pwm_parm);

						if (gret == 0) {
							pwm_value = pwm_parm;
						} else {
							usage("PARAM LOAD FAIL");
						}
					} else {
						usage("PARAM NAME NOT FOUND");
					}
				} else {

					pwm_value = strtoul(optarg, &ep, 0);
				}

				if (*ep != '\0') {
					usage("BAD PWM VAL");
				}
			}

			break;

		case 'r':
			alt_rate = strtoul(optarg, &ep, 0);

			if (*ep != '\0') {
				usage("BAD rate VAL");
			}

			break;

		default:
			break;
		}
	}

	if (print_verbose && set_mask > 0) {
		warnx("Channels: ");
		printf("    ");

		for (unsigned i = 0; i < PWM_OUTPUT_MAX_CHANNELS; i++) {
			if (set_mask & 1 << i) {
				printf("%d ", i + 1);
			}
		}

		printf("\n");
	}

	/* open for ioctl only */
	int fd = open(dev, 0);

	if (fd < 0) {
		err(1, "can't open %s", dev);
	}

	/* get the number of servo channels */
	unsigned servo_count;
	ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);

	if (ret != OK) {
		err(1, "PWM_SERVO_GET_COUNT");
	}

	if (!strcmp(argv[1], "arm")) {
		/* tell safety that its ok to disable it with the switch */
		ret = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);

		if (ret != OK) {
			err(1, "PWM_SERVO_SET_ARM_OK");
		}

		/* tell IO that the system is armed (it will output values if safety is off) */
		ret = ioctl(fd, PWM_SERVO_ARM, 0);

		if (ret != OK) {
			err(1, "PWM_SERVO_ARM");
		}

		if (print_verbose) {
			warnx("Outputs armed");
		}

		exit(0);

	} else if (!strcmp(argv[1], "disarm")) {
		/* disarm, but do not revoke the SET_ARM_OK flag */
		ret = ioctl(fd, PWM_SERVO_DISARM, 0);

		if (ret != OK) {
			err(1, "PWM_SERVO_DISARM");
		}

		if (print_verbose) {
			warnx("Outputs disarmed");
		}

		exit(0);

	} else if (!strcmp(argv[1], "rate")) {

		/* change alternate PWM rate */
		if (alt_rate > 0) {
			ret = ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, alt_rate);

			if (ret != OK) {
				err(1, "PWM_SERVO_SET_UPDATE_RATE (check rate for sanity)");
			}
		}

		/* directly supplied channel mask */
		if (set_mask > 0) {
			ret = ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, set_mask);

			if (ret != OK) {
				err(1, "PWM_SERVO_SET_SELECT_UPDATE_RATE");
			}
		}

		/* assign alternate rate to channel groups */
		if (alt_channels_set) {
			uint32_t mask = 0;

			for (group = 0; group < 32; group++) {
				if ((1 << group) & alt_channel_groups) {
					uint32_t group_mask;

					ret = ioctl(fd, PWM_SERVO_GET_RATEGROUP(group), (unsigned long)&group_mask);

					if (ret != OK) {
						err(1, "PWM_SERVO_GET_RATEGROUP(%u)", group);
					}

					mask |= group_mask;
				}
			}

			ret = ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, mask);

			if (ret != OK) {
				err(1, "PWM_SERVO_SET_SELECT_UPDATE_RATE");
			}
		}

		exit(0);

	} else if (!strcmp(argv[1], "min")) {

		if (set_mask == 0) {
			usage("no channels set");
		}

		if (pwm_value == 0) {
			usage("no PWM value provided");
		}

		struct pwm_output_values pwm_values;

		memset(&pwm_values, 0, sizeof(pwm_values));

		pwm_values.channel_count = servo_count;

		/* first get current state before modifying it */
		ret = ioctl(fd, PWM_SERVO_GET_MIN_PWM, (long unsigned int)&pwm_values);

		if (ret != OK) {
			errx(ret, "failed get min values");
		}

		for (unsigned i = 0; i < servo_count; i++) {
			if (set_mask & 1 << i) {
				pwm_values.values[i] = pwm_value;

				if (print_verbose) {
					warnx("Channel %d: min PWM: %d", i + 1, pwm_value);
				}
			}
		}

		if (pwm_values.channel_count == 0) {
			usage("no PWM values added");

		} else {

			ret = ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);

			if (ret != OK) {
				errx(ret, "failed setting min values");
			}
		}

		exit(0);

	} else if (!strcmp(argv[1], "max")) {

		if (set_mask == 0) {
			usage("no channels set");
		}

		if (pwm_value == 0) {
			usage("no PWM value provided");
		}

		struct pwm_output_values pwm_values;

		memset(&pwm_values, 0, sizeof(pwm_values));

		pwm_values.channel_count = servo_count;

		/* first get current state before modifying it */
		ret = ioctl(fd, PWM_SERVO_GET_MAX_PWM, (long unsigned int)&pwm_values);

		if (ret != OK) {
			errx(ret, "failed get max values");
		}

		for (unsigned i = 0; i < servo_count; i++) {
			if (set_mask & 1 << i) {
				pwm_values.values[i] = pwm_value;

				if (print_verbose) {
					warnx("Channel %d: max PWM: %d", i + 1, pwm_value);
				}
			}
		}

		if (pwm_values.channel_count == 0) {
			usage("no PWM values added");

		} else {

			ret = ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);

			if (ret != OK) {
				errx(ret, "failed setting max values");
			}
		}

		exit(0);

	} else if (!strcmp(argv[1], "disarmed")) {

		if (set_mask == 0) {
			usage("no channels set");
		}

		if (pwm_value == 0) {
			warnx("reading disarmed value of zero, disabling disarmed PWM");
		}

		struct pwm_output_values pwm_values;

		memset(&pwm_values, 0, sizeof(pwm_values));

		pwm_values.channel_count = servo_count;

		/* first get current state before modifying it */
		ret = ioctl(fd, PWM_SERVO_GET_DISARMED_PWM, (long unsigned int)&pwm_values);

		if (ret != OK) {
			errx(ret, "failed get disarmed values");
		}

		for (unsigned i = 0; i < servo_count; i++) {
			if (set_mask & 1 << i) {
				pwm_values.values[i] = pwm_value;

				if (print_verbose) {
					warnx("chan %d: disarmed PWM: %d", i + 1, pwm_value);
				}
			}
		}

		if (pwm_values.channel_count == 0) {
			usage("no PWM values added");

		} else {

			ret = ioctl(fd, PWM_SERVO_SET_DISARMED_PWM, (long unsigned int)&pwm_values);

			if (ret != OK) {
				errx(ret, "failed setting disarmed values");
			}
		}

		exit(0);

	} else if (!strcmp(argv[1], "failsafe")) {

		if (set_mask == 0) {
			usage("no channels set");
		}

		if (pwm_value == 0) {
			usage("no PWM provided");
		}

		struct pwm_output_values pwm_values;

		memset(&pwm_values, 0, sizeof(pwm_values));

		pwm_values.channel_count = servo_count;

		/* first get current state before modifying it */
		ret = ioctl(fd, PWM_SERVO_GET_FAILSAFE_PWM, (long unsigned int)&pwm_values);

		if (ret != OK) {
			errx(ret, "failed get failsafe values");
		}

		for (unsigned i = 0; i < servo_count; i++) {
			if (set_mask & 1 << i) {
				pwm_values.values[i] = pwm_value;

				if (print_verbose) {
					warnx("Channel %d: failsafe PWM: %d", i + 1, pwm_value);
				}
			}
		}

		if (pwm_values.channel_count == 0) {
			usage("no PWM values added");

		} else {

			ret = ioctl(fd, PWM_SERVO_SET_FAILSAFE_PWM, (long unsigned int)&pwm_values);

			if (ret != OK) {
				errx(ret, "BAD input VAL");
			}
		}

		exit(0);

	} else if (!strcmp(argv[1], "test")) {

		if (set_mask == 0) {
			usage("no channels set");
		}

		if (pwm_value == 0) {
			usage("no PWM provided");
		}

		/* get current servo values */
		struct pwm_output_values last_spos;

		for (unsigned i = 0; i < servo_count; i++) {


			ret = ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]);

			if (ret != OK) {
				err(1, "PWM_SERVO_GET(%d)", i);
			}
		}

		/* perform PWM output */

		/* Open console directly to grab CTRL-C signal */
		struct pollfd fds;
		fds.fd = 0; /* stdin */
		fds.events = POLLIN;

		warnx("Press CTRL-C or 'c' to abort.");

		while (1) {
			for (unsigned i = 0; i < servo_count; i++) {
				if (set_mask & 1 << i) {
					ret = ioctl(fd, PWM_SERVO_SET(i), pwm_value);

					if (ret != OK) {
						err(1, "PWM_SERVO_SET(%d)", i);
					}
				}
			}

			/* abort on user request */
			char c;
			ret = poll(&fds, 1, 0);

			if (ret > 0) {

				read(0, &c, 1);

				if (c == 0x03 || c == 0x63 || c == 'q') {
					/* reset output to the last value */
					for (unsigned i = 0; i < servo_count; i++) {
						if (set_mask & 1 << i) {
							ret = ioctl(fd, PWM_SERVO_SET(i), last_spos.values[i]);

							if (ret != OK) {
								err(1, "PWM_SERVO_SET(%d)", i);
							}
						}
					}

					warnx("User abort\n");
					exit(0);
				}
			}

			usleep(2000);
		}

		exit(0);


	} else if (!strcmp(argv[1], "steps")) {

		if (set_mask == 0) {
			usage("no channels set");
		}

		/* get current servo values */
		struct pwm_output_values last_spos;

		for (unsigned i = 0; i < servo_count; i++) {

			ret = ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]);

			if (ret != OK) {
				err(1, "PWM_SERVO_GET(%d)", i);
			}
		}

		/* perform PWM output */

		/* Open console directly to grab CTRL-C signal */
		struct pollfd fds;
		fds.fd = 0; /* stdin */
		fds.events = POLLIN;

		warnx("Running 5 steps. WARNING! Motors will be live in 5 seconds\nPress any key to abort now.");
		sleep(5);

		unsigned off = 900;
		unsigned idle = 1300;
		unsigned full = 2000;
		unsigned steps_timings_us[] = {2000, 5000, 20000, 50000};

		unsigned phase = 0;
		unsigned phase_counter = 0;
		unsigned const phase_maxcount = 20;

		for (unsigned steps_timing_index = 0;
		     steps_timing_index < sizeof(steps_timings_us) / sizeof(steps_timings_us[0]);
		     steps_timing_index++) {

			warnx("Step input (0 to 100%%) over %u us ramp", steps_timings_us[steps_timing_index]);

			while (1) {
				for (unsigned i = 0; i < servo_count; i++) {
					if (set_mask & 1 << i) {

						unsigned val;

						if (phase == 0) {
							val = idle;

						} else if (phase == 1) {
							/* ramp - depending how steep it is this ramp will look instantaneous on the output */
							val = idle + (full - idle) * (phase_maxcount / (float)phase_counter);

						} else {
							val = off;
						}

						ret = ioctl(fd, PWM_SERVO_SET(i), val);

						if (ret != OK) {
							err(1, "PWM_SERVO_SET(%d)", i);
						}
					}
				}

				/* abort on user request */
				char c;
				ret = poll(&fds, 1, 0);

				if (ret > 0) {

					ret = read(0, &c, 1);

					if (ret > 0) {
						/* reset output to the last value */
						for (unsigned i = 0; i < servo_count; i++) {
							if (set_mask & 1 << i) {
								ret = ioctl(fd, PWM_SERVO_SET(i), last_spos.values[i]);

								if (ret != OK) {
									err(1, "PWM_SERVO_SET(%d)", i);
								}
							}
						}

						warnx("User abort\n");
						exit(0);
					}
				}

				if (phase == 1) {
					usleep(steps_timings_us[steps_timing_index] / phase_maxcount);

				} else if (phase == 0) {
					usleep(50000);

				} else if (phase == 2) {
					usleep(50000);

				} else {
					break;
				}

				phase_counter++;

				if (phase_counter > phase_maxcount) {
					phase++;
					phase_counter = 0;
				}
			}
		}

		exit(0);


	} else if (!strcmp(argv[1], "info")) {

		printf("device: %s\n", dev);

		uint32_t info_default_rate;
		uint32_t info_alt_rate;
		uint32_t info_alt_rate_mask;

		ret = ioctl(fd, PWM_SERVO_GET_DEFAULT_UPDATE_RATE, (unsigned long)&info_default_rate);

		if (ret != OK) {
			err(1, "PWM_SERVO_GET_DEFAULT_UPDATE_RATE");
		}

		ret = ioctl(fd, PWM_SERVO_GET_UPDATE_RATE, (unsigned long)&info_alt_rate);

		if (ret != OK) {
			err(1, "PWM_SERVO_GET_UPDATE_RATE");
		}

		ret = ioctl(fd, PWM_SERVO_GET_SELECT_UPDATE_RATE, (unsigned long)&info_alt_rate_mask);

		if (ret != OK) {
			err(1, "PWM_SERVO_GET_SELECT_UPDATE_RATE");
		}

		struct pwm_output_values failsafe_pwm;

		struct pwm_output_values disarmed_pwm;

		struct pwm_output_values min_pwm;

		struct pwm_output_values max_pwm;

		ret = ioctl(fd, PWM_SERVO_GET_FAILSAFE_PWM, (unsigned long)&failsafe_pwm);

		if (ret != OK) {
			err(1, "PWM_SERVO_GET_FAILSAFE_PWM");
		}

		ret = ioctl(fd, PWM_SERVO_GET_DISARMED_PWM, (unsigned long)&disarmed_pwm);

		if (ret != OK) {
			err(1, "PWM_SERVO_GET_DISARMED_PWM");
		}

		ret = ioctl(fd, PWM_SERVO_GET_MIN_PWM, (unsigned long)&min_pwm);

		if (ret != OK) {
			err(1, "PWM_SERVO_GET_MIN_PWM");
		}

		ret = ioctl(fd, PWM_SERVO_GET_MAX_PWM, (unsigned long)&max_pwm);

		if (ret != OK) {
			err(1, "PWM_SERVO_GET_MAX_PWM");
		}

		/* print current servo values */
		for (unsigned i = 0; i < servo_count; i++) {
			servo_position_t spos;

			ret = ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&spos);

			if (ret == OK) {
				printf("channel %u: %u us", i + 1, spos);

				if (info_alt_rate_mask & (1 << i)) {
					printf(" (alternative rate: %d Hz", info_alt_rate);

				} else {
					printf(" (default rate: %d Hz", info_default_rate);
				}


				printf(" failsafe: %d, disarmed: %d us, min: %d us, max: %d us)",
				       failsafe_pwm.values[i], disarmed_pwm.values[i], min_pwm.values[i], max_pwm.values[i]);
				printf("\n");

			} else {
				printf("%u: ERROR\n", i);
			}
		}

		/* print rate groups */
		for (unsigned i = 0; i < servo_count; i++) {
			uint32_t group_mask;

			ret = ioctl(fd, PWM_SERVO_GET_RATEGROUP(i), (unsigned long)&group_mask);

			if (ret != OK) {
				break;
			}

			if (group_mask != 0) {
				printf("channel group %u: channels", i);

				for (unsigned j = 0; j < 32; j++)
					if (group_mask & (1 << j)) {
						printf(" %u", j + 1);
					}

				printf("\n");
			}
		}

		exit(0);

	} else if (!strcmp(argv[1], "forcefail")) {

		if (argc < 3) {
			errx(1, "arg missing [on|off]");

		} else {

			if (!strcmp(argv[2], "on")) {
				/* force failsafe */
				ret = ioctl(fd, PWM_SERVO_SET_FORCE_FAILSAFE, 1);

			} else {
				/* force failsafe */
				ret = ioctl(fd, PWM_SERVO_SET_FORCE_FAILSAFE, 0);
			}

			if (ret != OK) {
				warnx("FAILED setting forcefail %s", argv[2]);
			}
		}

		exit(0);

	} else if (!strcmp(argv[1], "terminatefail")) {

		if (argc < 3) {
			errx(1, "arg missing [on|off]");

		} else {

			if (!strcmp(argv[2], "on")) {
				/* force failsafe */
				ret = ioctl(fd, PWM_SERVO_SET_TERMINATION_FAILSAFE, 1);

			} else {
				/* force failsafe */
				ret = ioctl(fd, PWM_SERVO_SET_TERMINATION_FAILSAFE, 0);
			}

			if (ret != OK) {
				warnx("FAILED setting termination failsafe %s", argv[2]);
			}
		}

		exit(0);
	}

	usage(NULL);
	return 0;
}

