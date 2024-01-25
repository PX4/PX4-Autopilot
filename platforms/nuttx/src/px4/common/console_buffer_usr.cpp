/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/console_buffer.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/sem.h>
#include <pthread.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdio.h>

#ifdef BOARD_ENABLE_CONSOLE_BUFFER

static int console_fd = -1;

int px4_console_buffer_init()
{
	console_fd = open(CONSOLE_BUFFER_DEVICE, O_RDWR);

	if (console_fd < 0) {
		return ERROR;
	}

	return OK;
}

int px4_console_buffer_size()
{
	int size;

	if (ioctl(console_fd, FIONSPACE, &size) < 0) {
		return 0;
	}

	return size;
}

int px4_console_buffer_read(char *buffer, int buffer_length, int *offset)
{
	FILE *fp;
	ssize_t nread;

	/* Open a file stream to keep track of offset */
	fp = fdopen(dup(console_fd), "r");

	if (fp == NULL) {
		return -1;
	}

	/* The driver does not utilize file position, we have to do it for it */
	fseek(fp, *offset, SEEK_SET);
	nread = read(console_fd, buffer, buffer_length);
	*offset = fseek(fp, 0, SEEK_CUR);

	/* Now we can close the file */
	fclose(fp);

	return (int)nread;
}

#endif /* BOARD_ENABLE_CONSOLE_BUFFER */
