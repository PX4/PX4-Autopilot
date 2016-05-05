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
 * @file mixer_load.c
 *
 * Programmable multi-channel mixer library.
 */

#include <px4_config.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <systemlib/err.h>

#include "mixer_load.h"

int load_mixer_file(const char *fname, char *buf, unsigned maxlen)
{
	FILE		*fp;
	char		line[120];

	/* open the mixer definition file */
	fp = fopen(fname, "r");

	if (fp == NULL) {
		warnx("file not found");
		return -1;
	}

	/* read valid lines from the file into a buffer */
	buf[0] = '\0';

	for (;;) {

		/* get a line, bail on error/EOF */
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == NULL) {
			break;
		}

		/* if the line doesn't look like a mixer definition line, skip it */
		if ((strlen(line) < 2) || !isupper(line[0]) || (line[1] != ':')) {
			continue;
		}

		/* compact whitespace in the buffer */
		char *t, *f;

		for (f = line; *f != '\0'; f++) {
			/* scan for space characters */
			if (*f == ' ') {
				/* look for additional spaces */
				t = f + 1;

				while (*t == ' ') {
					t++;
				}

				if (*t == '\0') {
					/* strip trailing whitespace */
					*f = '\0';

				} else if (t > (f + 1)) {
					memmove(f + 1, t, strlen(t) + 1);
				}
			}
		}

		/* if the line is too long to fit in the buffer, bail */
		if ((strlen(line) + strlen(buf) + 1) >= maxlen) {
			warnx("line too long");
			fclose(fp);
			return -1;
		}

		/* add the line to the buffer */
		strcat(buf, line);
	}

	fclose(fp);
	return 0;
}

