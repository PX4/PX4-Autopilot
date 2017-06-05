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
 * @file mixer.c
 *
 * Mixer utility.
 */

#include <px4_config.h>
#include <px4_posix.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>

#include <systemlib/err.h>
#include <systemlib/mixer/mixer.h>
#include <uORB/topics/actuator_controls.h>

/**
 * Mixer utility for loading mixer files to devices
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mixer_main(int argc, char *argv[]);

static void	usage(const char *reason);
static int	load(const char *devname, const char *fname, bool append);

#if (defined(MIXER_TUNING) && !defined(MIXER_CONFIG_NO_NSH))
static int  save(const char *devname, const char *fname);
static int  mixer_param_list(const char *devname);
static int  mixer_param_set(const char *devname, int index, float *values);
static int  mixer_show_config(const char *devname);
#endif //defined(MIXER_TUNING)

int
mixer_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "load")) {
		if (argc < 4) {
			usage("missing device or filename");
			return 1;
		}

		int ret = load(argv[2], argv[3], false);

		if (ret != 0) {
			PX4_ERR("failed to load mixer");
			return 1;
		}

	} else if (!strcmp(argv[1], "append")) {
		if (argc < 4) {
			usage("missing device or filename");
			return 1;
		}

		int ret = load(argv[2], argv[3], true);

		if (ret != 0) {
			PX4_ERR("failed to append mixer");
			return 1;
		}

#if (defined(MIXER_TUNING) && !defined(MIXER_CONFIG_NO_NSH))

	}  else if (!strcmp(argv[1], "save")) {
		if (argc < 4) {
			usage("missing device or filename");
			return 1;
		}

		int ret = save(argv[2], argv[3]);

		if (ret != 0) {
			warnx("failed to save mixer");
			return 1;
		}

	} else if (!strcmp(argv[1], "params")) {
		if (argc < 3) {
			usage("missing device");
			return 1;
		}

		int ret;

		ret = mixer_param_list(argv[2]);

		if (ret != 0) {
			warnx("failed to list parameters");
			return 1;
		}

	} else if (!strcmp(argv[1], "set")) {
		if (argc < 5) {
			usage("missing device, index or value");
			return 1;
		}

		int ret;
		float params[8];
		params[0] = (float) strtod(argv[4], nullptr);

		if (argc == 5) {
			ret = mixer_param_set(argv[2],
					      strtoul(argv[3], nullptr, 10),
					      params);

		} else {
			PX4_INFO("No support for more than one array value", argc);
			return 1;
		}

		if (ret != 0) {
			warnx("failed to list parameters");
			return 1;
		}

	} else if (!strcmp(argv[1], "config")) {
		if (argc < 3) {
			usage("missing device: usage 'mixer config <device>'");
			return 1;
		}

		int ret = mixer_show_config(argv[2]);

		if (ret != 0) {
			warnx("failed to show config");
			return 1;
		}


#endif //defined(MIXER_TUNING)

	} else {
		usage("Unknown command");
		return 1;
	}

	return 0;
}

static void
usage(const char *reason)
{
	if (reason && *reason) {
		PX4_INFO("%s", reason);
	}

	PX4_INFO("usage:");
	PX4_INFO("  mixer load <device> <filename>");
#if (defined(MIXER_TUNING) && !defined(MIXER_CONFIG_NO_NSH))
	PX4_INFO("  mixer save <device> <filename>");
	PX4_INFO("  mixer params <device>");
	PX4_INFO("  mixer set <device> <index> <value>");
	PX4_INFO("  mixer config <device>");
#endif //defined(MIXER_TUNING)
}

static int
load(const char *devname, const char *fname, bool append)
{
	// sleep a while to ensure device has been set up
	usleep(20000);

	int dev;

	/* open the device */
	if ((dev = px4_open(devname, 0)) < 0) {
		PX4_ERR("can't open %s\n", devname);
		return 1;
	}

	/* reset mixers on the device, but not if appending */
	if (!append) {
		if (px4_ioctl(dev, MIXERIOCRESET, 0)) {
			PX4_ERR("can't reset mixers on %s", devname);
			return 1;
		}
	}

	char buf[2048];

	if (load_mixer_file(fname, &buf[0], sizeof(buf)) < 0) {
		PX4_ERR("can't load mixer file: %s", fname);
		return 1;
	}

	/* Pass the buffer to the device */
	int ret = px4_ioctl(dev, MIXERIOCLOADBUF, (unsigned long)buf);

	if (ret < 0) {
		PX4_ERR("failed to load mixers from %s", fname);
		return 1;
	}

	return 0;
}

#if (defined(MIXER_TUNING) && !defined(MIXER_CONFIG_NO_NSH))
static int
save(const char *devname, const char *fname)
{
	// sleep a while to ensure device has been set up
	usleep(20000);

	int dev;

	/* open the device */
	if ((dev = px4_open(devname, 0)) < 0) {
		warnx("can't open %s\n", devname);
		return -1;
	}

	char buf[1024];
	mixer_config_s config = {buf, 1022};
	int ret = px4_ioctl(dev, MIXERIOCGETCONFIG, (unsigned long)&config);
	px4_close(dev);


	if (ret != 0) {
		warnx("Could not get mixer config for %s\n", devname);
		return -1;
	}

	/* Create the mixer definition file */
#ifdef __PX4_NUTTX
	int fd = open(fname, O_CREAT | O_WRONLY | O_DSYNC);
#else
	int fd = open(fname, O_CREAT | O_WRONLY | O_DSYNC, PX4_O_MODE_666);
#endif

	if (fd < 0) {
		warnx("not able to create file %s", fname);
		return -1;
	}

	unsigned buflen = strlen(buf);

	/* Write the buffer to the file*/
	ssize_t wr_len = write(fd, buf, strlen(buf));

	if (wr_len != buflen) {
		warnx("not able to fully write to file %s", fname);

	} else {
		PX4_INFO("Wrote mixer %s to file %s\n", devname, fname);
	}

	fsync(fd);
	close(fd);

	return 0;
}

static int  mixer_show_config(const char *devname)
{
	int dev;

	/* open the device */
	if ((dev = px4_open(devname, 0)) < 0) {
		warnx("can't open %s\n", devname);
		return 1;
	}

	char buf[1024];
	mixer_config_s config = {buf, 1022};
	int ret = px4_ioctl(dev, MIXERIOCGETCONFIG, (unsigned long)&config);
	px4_close(dev);

	if (ret == 0) {
		printf("%s", buf);

	} else {
		warnx("Could not generate mixer config for %s\n", devname);
		return 1;
	}

	return 0;
}


static int
mixer_param_list(const char *devname)
{
	mixer_param_s param;
	int dev;

	/* open the device */
	if ((dev = px4_open(devname, 0)) < 0) {
		warnx("can't open %s\n", devname);
		return 1;
	}

	int param_count;

	/* Get the mixer count */
	int ret = px4_ioctl(dev, MIXERIOCGETPARAMCOUNT, (unsigned long)&param_count);

	if (ret < 0) {
		warnx("can't get parameter count for :%s\n", devname);
		px4_close(dev);
		return 1;
	}

	if (param_count == 0) {
		printf("Device:%s  parameter list empty\n", devname);
		px4_close(dev);
		return 1;
	}

	printf("Device:%s parameter count:%u\n", devname, param_count);

	for (int index = 0; index < param_count; index++) {
		param.index = index;
		ret = px4_ioctl(dev, MIXERIOCGETPARAM, (unsigned long)&param);

		if (ret < 0) {
			warnx("can't get parameter\n");
			px4_close(dev);
			return 1;
		}

		printf("index:%-3u mixer:%u submixer:%u type:%u size:%u id:%-16s values:[", index, param.mix_index,
		       param.mix_sub_index,
		       param.type, param.array_size, param.name);

		switch (param.param_type) {
		case 5: { //MAV_PARAM_TYPE_UINT32
				for (int val = 0; val < param.array_size; val++) {
					printf("0x%X, ", param.values[val].uintval);
				}

				break;
			}

		case 9: { //MAV_PARAM_TYPE_REAL32
				for (int val = 0; val < param.array_size; val++) {
					printf("%.3f, ", (double) param.values[val].realval);
				}

				break;
			}
		}


		printf("]\n");
	}


	px4_close(dev);
	return 0;
}


static int
mixer_param_set(const char *devname, int index, float *values)
{
	mixer_param_s param;
	int dev;

	/* open the device */
	if ((dev = px4_open(devname, 0)) < 0) {
		warnx("can't open %s\n", devname);
		return 1;
	}

	// Set main index to value but set all other refs to -1
	// This indicates that those refs should not be used.
	param.index = index;
	param.mix_index = -1;
	param.mix_sub_index = -1;
	param.type = MIXER_PARAM_MSG_TYPE_PARAMETER;
	strcpy(param.name, "");

	//Clear value to zero and only use first value.  px4 mixers specifc
	memset(param.values, 0, sizeof(param.values));
	param.values[0].realval = values[0];

	int ret = px4_ioctl(dev, MIXERIOCSETPARAM, (unsigned long)&param);
	px4_close(dev);

	if (ret < 0) {
		PX4_WARN("fail to set mixer parameter");
		return -1;
	}

	PX4_INFO("mixer param index:%u value:%.4f set success\n", param.index, (double) param.values[0].realval);
	return 0;
}


#endif //defined(MIXER_TUNING)
