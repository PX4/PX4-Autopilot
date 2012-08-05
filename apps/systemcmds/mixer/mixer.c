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
 * @file	Mixer utility.
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>
#include <unistd.h>
#include <fcntl.h>

#include <systemlib/mixer.h>
#include <drivers/drv_mixer.h>
#include <uORB/topics/actuator_controls.h>

__EXPORT int mixer_main(int argc, char *argv[]);

static void	usage(void);
static void	load(const char *devname, const char *fname);
static void	save(const char *devname, const char *fname);
static void	show(const char *devname);

enum Operation {
	LOAD,
	SAVE,
	SHOW,
	NONE
};

int
mixer_main(int argc, char *argv[])
{
	const char *devname = NULL;
	const char *fname = NULL;
	int ch;
	enum Operation todo = NONE;

	if (argc < 2)
		usage();
	if (!strcmp(argv[1], "load")) {
		todo = LOAD;
	} else if (!strcmp(argv[1], "save")) {
		todo = SAVE;
	} else if (!strcmp(argv[1], "show")) {
		todo = SHOW;
	} else {
		usage();
	}

	while ((ch = getopt(argc, argv, "d:f:")) != EOF) {
		switch (ch) {
		case 'd':
			devname = optarg;
			break;
		case 'f':
			fname = optarg;
			break;
		default:
			usage();
		}
	}

	switch (todo) {
	case LOAD:
		if ((devname == NULL) || (fname == NULL))
			usage();
		load(devname, fname);
		break;

	case SAVE:
		if ((devname == NULL) || (fname == NULL))
			usage();
		save(devname, fname);
		break;

	case SHOW:
		if (devname == NULL)
			usage();
		show(devname);
		break;
	default:
		exit(1);
	}
	return 0;
}

static void
usage(void)
{
	fprintf(stderr, "usage:\n");
	fprintf(stderr, "  mixer {load|save|show} -d <device> [-f <filename>]\n");
	exit(1);
}

static void
load(const char *devname, const char *fname)
{
	int		defs = -1;
	int		dev = -1;
	unsigned	num_mixers = 0;
	int		ret, result = 1;
	struct MixMixer	*mixer;

	/* open the device */
	if ((dev = open(devname, 0)) < 0) {
		fprintf(stderr, "can't open %s\n", devname);
		goto out;
	}

	/* open the definition file */
	if ((defs = open(fname, O_RDONLY)) < 0) {
		fprintf(stderr, "can't open %s\n", fname);
		goto out;
	}

	/* find out how many mixers the device supports */
	ioctl(dev, MIXERIOCGETMIXERCOUNT, (unsigned long)&num_mixers);
	if (num_mixers < 1) {
		fprintf(stderr, "can't get mixer count from %s\n", devname);
		goto out;
	}

	/* send mixers to the device */
	for (unsigned i = 0; i < num_mixers; i++) {
		ret = mixer_load(defs, &mixer);
		if (ret < 0)
			goto out;

		/* end of file? */
		if (ret == 0)
			break;

		/* sanity check the mixer */
		if (mixer_check(mixer, NUM_ACTUATOR_CONTROLS)) {
			fprintf(stderr, "mixer %u fails sanity check\n", i);
			goto out;
		}

		/* send the mixer to the device */
		ret = ioctl(dev, MIXERIOCSETMIXER(i), (unsigned long)mixer);
		if (ret < 0)
			goto out;

		free(mixer);
		mixer = NULL;
	}

	printf("mixer: loaded %d mixers to %s\n", num_mixers, devname);
	result = 0;

out:
	/* free the mixers array */
	if (mixer != NULL)
		free(mixer);
	if (defs != -1)
		close(defs);
	if (dev != -1)
		close(dev);

	exit(result);
}

static int
getmixer(int dev, unsigned mixer_number, struct MixInfo **mip)
{
	struct MixInfo	*mi = *mip;
	int		ret;

	/* first-round initialisation */
	if (mi == NULL) {
		mi->num_controls = 0;
		mi = (struct MixInfo *)malloc(MIXINFO_SIZE(mi->num_controls));
	}

	/* loop trying to get the next mixer until the buffer is big enough */
	do {
		/* try to get the mixer into the buffer as it stands */
		ret = ioctl(dev, MIXERIOCGETMIXER(mixer_number), (unsigned long)mi);
		if (ret < 0) {
			fprintf(stderr, "MIXERIOCGETMIXER error\n");
			return -1;
		}

		/* did the mixer fit? */
		if (mi->mixer.control_count <= mi->num_controls)
			break;

		/* re-allocate to suit */
		mi->num_controls = mi->mixer.control_count;
		mi = (struct MixInfo *)realloc(mi, MIXINFO_SIZE(mi->num_controls));

		/* oops, blew up the heap */
		if (mi == NULL)
			return -1;

	} while(true);

	*mip = mi;
	return 0;
}

static void
save(const char *devname, const char *fname)
{
	struct MixInfo	*mi = NULL;
	int		defs = -1;
	int		dev = -1;
	unsigned	num_mixers = 0;
	int		ret, result = 1;

	/* open the device */
	if ((dev = open(devname, 0)) < 0) {
		fprintf(stderr, "can't open %s\n", devname);
		goto out;
	}

	/* find out how many mixers the device supports */
	ioctl(dev, MIXERIOCGETMIXERCOUNT, (unsigned long)&num_mixers);
	if (num_mixers < 1) {
		fprintf(stderr, "can't get mixer count from %s\n", devname);
		goto out;
	}

	/* open the definition file */
	if ((defs = open(fname, O_WRONLY | O_CREAT)) < 0) {
		fprintf(stderr, "can't open %s\n", fname);
		goto out;
	}

	/* get mixers from the device and save them */
	for (unsigned i = 0; i < num_mixers; i++) {

		ret = getmixer(dev, i, &mi);
		if (ret < 0)
			goto out;

		ret = mixer_save(defs, &mi->mixer);
		if (ret < 0)
			goto out;
	}

	result = 0;

out:
	/* free the mixinfo */
	if (mi != NULL)
		free(mi);
	if (defs != -1)
		close(defs);
	if (dev != -1)
		close(dev);

	exit(result);
}

static void
show(const char *devname)
{
	struct MixInfo	*mi;
	int		dev = -1;
	unsigned	num_mixers = 0;
	int		ret;

	/* open the device */
	if ((dev = open(devname, 0)) < 0) {
		fprintf(stderr, "can't open %s\n", devname);
		goto out;
	}

	/* find out how many mixers the device supports */
	ioctl(dev, MIXERIOCGETMIXERCOUNT, (unsigned long)&num_mixers);
	if (num_mixers < 1) {
		fprintf(stderr, "can't get mixer count from %s\n", devname);
		goto out;
	}

	/* get mixers from the device and print them */
	for (unsigned i = 0; i < num_mixers; i++) {

		ret = getmixer(dev, i, &mi);
		if (ret < 0)
			goto out;

		printf("mixer %d : %d controls\n", i, mi->mixer.control_count);
		printf("        -ve scale  +ve scale  offset  low limit  high limit");
		printf("output  %f   %f   %f   %f   %f\n", 
			mi->mixer.output_scaler.negative_scale,
			mi->mixer.output_scaler.positive_scale,
			mi->mixer.output_scaler.offset,
			mi->mixer.output_scaler.lower_limit,
			mi->mixer.output_scaler.upper_limit);
		for (unsigned j = 0; j < mi->mixer.control_count; j++) {
			printf("%d:     %f   %f   %f   %f   %f\n", 
				mi->mixer.control_scaler[j].negative_scale,
				mi->mixer.control_scaler[j].positive_scale,
				mi->mixer.control_scaler[j].offset,
				mi->mixer.control_scaler[j].lower_limit,
				mi->mixer.control_scaler[j].upper_limit);
		}
	}

out:
	/* free the mixinfo */
	if (mi != NULL)
		free(mi);
	if (dev != -1)
		close(dev);
	exit(0);
}
