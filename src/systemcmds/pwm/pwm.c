/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
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
#include "drivers/drv_pwm_output.h"

static void	usage(const char *reason);
__EXPORT int	pwm_main(int argc, char *argv[]);


static void
usage(const char *reason)
{
	if (reason != NULL)
		warnx("%s", reason);
	errx(1, 
		"usage:\n"
		"pwm [-v] [-d <device>] set|config|arm|disarm|info ...\n"
		""
		"  -v                     Print verbose information\n"
		"  -d <device>            PWM output device (defaults to " PWM_OUTPUT_DEVICE_PATH ")\n"
		"\n"
		"arm                      Arm output\n"
		"disarm                   Disarm output\n"
		"\n"
		"set <channel_value> ...  Directly set PWM values\n"
		"\n"
		"config rate <alt_rate>   Configure PWM rates\n"
		"  [-c <channel group>]   Channel group that should update at the alternate rate\n"
		"  [-m <chanmask> ]       Directly supply alt rate channel mask\n"
		"\n"
		"config min <channel value]> ...     Configure minimum PWM values\n"
		"config max <channel value]> ...     Configure maximum PWM values\n"
		"config disarmed <channel_value> ... Configure disarmed PWM values\n"
		"\n"
		"info                     Print information about the PWM device\n"
		);

}

int
pwm_main(int argc, char *argv[])
{
	const char *dev = PWM_OUTPUT_DEVICE_PATH;
	unsigned alt_rate = 0;
	uint32_t alt_channel_groups = 0;
	bool alt_channels_set = false;
	bool print_verbose = false;
	int ch;
	int ret;
	char *ep;
	int32_t set_mask = -1;
	unsigned group;

	if (argc < 1)
		usage(NULL);

	while ((ch = getopt(argc, argv, "v:d:")) != EOF) {
		switch (ch) {

		case 'd':
			if (NULL == strstr(optarg, "/dev/")) {
				warnx("device %s not valid", optarg);
				usage(NULL);
			}
			dev = optarg;
			argv+=2;
			argc-=2;
			break;

		case 'v':
			print_verbose = true;
			argv+=1;
			argc-=1;
			break;

		default:
			break;
		}
	}

	/* get rid of cmd name */
	argv+=1;
	argc-=1;

	/* open for ioctl only */
	int fd = open(dev, 0);
	if (fd < 0)
		err(1, "can't open %s", dev);


	for (int argi=0; argi<argc; argi++) {

		if (!strcmp(argv[argi], "arm")) {
			/* tell IO that its ok to disable its safety with the switch */
			ret = ioctl(fd, PWM_SERVO_SET_ARM_OK, 0);
			if (ret != OK)
				err(1, "PWM_SERVO_SET_ARM_OK");
			/* tell IO that the system is armed (it will output values if safety is off) */
			ret = ioctl(fd, PWM_SERVO_ARM, 0);
			if (ret != OK)
				err(1, "PWM_SERVO_ARM");

			warnx("Outputs armed");
			exit(0);

		} else if (!strcmp(argv[argi], "disarm")) {
			/* disarm, but do not revoke the SET_ARM_OK flag */
			ret = ioctl(fd, PWM_SERVO_DISARM, 0);
			if (ret != OK)
				err(1, "PWM_SERVO_DISARM");

			warnx("Outputs disarmed");
			exit(0);

		} else if (!strcmp(argv[argi], "set")) {

			/* iterate remaining arguments */
			unsigned nchannels = 0;
			unsigned channel[PWM_OUTPUT_MAX_CHANNELS] = {0};

			while (argc - argi > 1) {
				argi++;
				unsigned pwm_value = strtol(argv[argi], &ep, 0);
				if (*ep == '\0') {
					if (nchannels > sizeof(channel) / sizeof(channel[0]))
						err(1, "too many pwm values (max %d)", sizeof(channel) / sizeof(channel[0]));
					//XXX check for sane values ?
					channel[nchannels] = pwm_value;
					if (print_verbose)
						warnx("Set channel %d: %d us", nchannels+1, channel[nchannels]);

					nchannels++;

					continue;
				}
				usage("unrecognized option");
			}

			/* perform PWM output */
			if (nchannels) {

				/* Open console directly to grab CTRL-C signal */
				struct pollfd fds;
				fds.fd = 0; /* stdin */
				fds.events = POLLIN;

				warnx("Press CTRL-C or 'c' to abort.");

				while (1) {
					for (unsigned i = 0; i < nchannels; i++) {
						ret = ioctl(fd, PWM_SERVO_SET(i), channel[i]);
						if (ret != OK)
							err(1, "PWM_SERVO_SET(%d)", i);
					}

					/* abort on user request */
					char c;
					ret = poll(&fds, 1, 0);
					if (ret > 0) {

					read(0, &c, 1);
						if (c == 0x03 || c == 0x63 || c == 'q') {
							warnx("User abort\n");
							exit(0);
						}
					}
				}
			} else {
				usage("no PWM values supplied");
			}

		} else if (!strcmp(argv[argi], "config")) {

			struct pwm_output_values pwm_values = {.values = {0}, .channel_count = 0};

			argi++;

			if (!strcmp(argv[argi], "rate")) {

				while ((ch = getopt(argc, argv, "m:c:")) != EOF) {
					switch (ch) {

					case 'm':
						set_mask = strtol(optarg, &ep, 0);
						if (*ep != '\0')
							usage("bad set_mask value");
						break;
						argi+=2;

					case 'c':
						group = strtoul(optarg, &ep, 0);
						if ((*ep != '\0') || (group >= 32))
							usage("bad channel_group value");
						alt_channel_groups |= (1 << group);
						alt_channels_set = true;
						argi+=2;
					break;
					}
				}
				argi++;
				if (argi >= argc)
					usage("no alt_rate value supplied");

				alt_rate = strtol(argv[argi], &ep, 0);
				if (*ep != '\0')
					usage("bad alt_rate value");
				break;

				/* change alternate PWM rate */
				if (alt_rate > 0) {
					ret = ioctl(fd, PWM_SERVO_SET_UPDATE_RATE, alt_rate);
					if (ret != OK)
						err(1, "PWM_SERVO_SET_UPDATE_RATE (check rate for sanity)");
				}

				/* directly supplied channel mask */
				if (set_mask != -1) {
					ret = ioctl(fd, PWM_SERVO_SELECT_UPDATE_RATE, set_mask);
					if (ret != OK)
						err(1, "PWM_SERVO_SELECT_UPDATE_RATE");
				}

				/* assign alternate rate to channel groups */
				if (alt_channels_set) {
					uint32_t mask = 0;

					for (group = 0; group < 32; group++) {
						if ((1 << group) & alt_channel_groups) {
							uint32_t group_mask;

							ret = ioctl(fd, PWM_SERVO_GET_RATEGROUP(group), (unsigned long)&group_mask);
							if (ret != OK)
								err(1, "PWM_SERVO_GET_RATEGROUP(%u)", group);

							mask |= group_mask;
						}
					}

					ret = ioctl(fd, PWM_SERVO_SELECT_UPDATE_RATE, mask);
					if (ret != OK)
						err(1, "PWM_SERVO_SELECT_UPDATE_RATE");
				}


			} else if (!strcmp(argv[argi], "min")) {

				/* iterate remaining arguments */
				while (argc - argi > 1) {
					argi++;
					unsigned pwm_value = strtol(argv[argi], &ep, 0);
					if (*ep == '\0') {
						if (pwm_values.channel_count > PWM_OUTPUT_MAX_CHANNELS)
							err(1, "too many pwm values (max %d)", PWM_OUTPUT_MAX_CHANNELS);
						//XXX check for sane values ?
						pwm_values.values[pwm_values.channel_count] = pwm_value;
						pwm_values.channel_count++;

						continue;
					}
					usage("unrecognized option");
				}
				if (pwm_values.channel_count == 0) {
					usage("no PWM values added");
				} else {

					ret = ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);
					if (ret != OK)
						errx(ret, "failed setting idle values");
				}


			} else if (!strcmp(argv[argi], "max")) {

				/* iterate remaining arguments */
				while (argc - argi > 1) {
					argi++;
					unsigned pwm_value = strtol(argv[argi], &ep, 0);
					if (*ep == '\0') {
						if (pwm_values.channel_count > PWM_OUTPUT_MAX_CHANNELS)
							err(1, "too many pwm values (max %d)", PWM_OUTPUT_MAX_CHANNELS);
						//XXX check for sane values ?
						pwm_values.values[pwm_values.channel_count] = pwm_value;
						pwm_values.channel_count++;

						continue;
					}
					usage("unrecognized option");
				}
				if (pwm_values.channel_count == 0) {
					usage("no PWM values added");
				} else {

					ret = ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);
					if (ret != OK)
						errx(ret, "failed setting idle values");
				}


			} else if (!strcmp(argv[argi], "disarmed")) {

				/* iterate remaining arguments */
				while (argc - argi > 1) {
					argi++;
					unsigned pwm_value = strtol(argv[argi], &ep, 0);
					if (*ep == '\0') {
						if (pwm_values.channel_count > PWM_OUTPUT_MAX_CHANNELS)
							err(1, "too many pwm values (max %d)", PWM_OUTPUT_MAX_CHANNELS);
						//XXX check for sane values ?
						pwm_values.values[pwm_values.channel_count] = pwm_value;
						pwm_values.channel_count++;

						continue;
					}
					usage("unrecognized option");
				}
				if (pwm_values.channel_count == 0) {
					usage("no PWM values added");
				} else {

					ret = ioctl(fd, PWM_SERVO_SET_DISARMED_PWM, (long unsigned int)&pwm_values);
					if (ret != OK)
						errx(ret, "failed setting idle values");
				}

			} else {
				usage("specify rate, min, max or disarmed");
			}

		} else if (!strcmp(argv[argi], "info")) {

			/* get the number of servo channels */
			unsigned count;
			ret = ioctl(fd, PWM_SERVO_GET_COUNT, (unsigned long)&count);
			if (ret != OK)
				err(1, "PWM_SERVO_GET_COUNT");

			/* print current servo values */
			for (unsigned i = 0; i < count; i++) {
				servo_position_t spos;

				ret = ioctl(fd, PWM_SERVO_GET(i), (unsigned long)&spos);
				if (ret == OK) {
					printf("channel %u: %uus\n", i, spos);
				} else {
					printf("%u: ERROR\n", i);
				}
			}

			/* print rate groups */
			for (unsigned i = 0; i < count; i++) {
				uint32_t group_mask;

				ret = ioctl(fd, PWM_SERVO_GET_RATEGROUP(i), (unsigned long)&group_mask);
				if (ret != OK)
					break;
				if (group_mask != 0) {
					printf("channel group %u: channels", i);
					for (unsigned j = 0; j < 32; j++)
						if (group_mask & (1 << j))
							printf(" %u", j);
					printf("\n");
				}
			}

		} else {
			usage("specify arm|disarm|set|config");
		}
	}
	exit(0);
}



