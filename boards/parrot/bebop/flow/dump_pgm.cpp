/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

#include "dump_pgm.h"

#include <string.h>
#include <sys/fcntl.h>
#include <sys/stat.h>

#include <px4_platform_common/posix.h>

#define HRES_STR "320"
#define VRES_STR "240"

char pgm_header[] = "P5\n#99999999999999 usec \n" HRES_STR " " VRES_STR "\n255\n";
char pgm_dumpname[] = "image";
char pgm_path[] = "/home/root/images/";

void dump_pgm(const void *data, uint32_t size, uint32_t seq, uint32_t timestamp)
{
	// Check if dump directory exists
	struct stat sb = {};

	if (!(stat(pgm_path, &sb) == 0 && S_ISDIR(sb.st_mode))) {
		PX4_ERR("Dump directory does not exist: %s", pgm_path);
		PX4_ERR("No images are written!");
		return;
	}

	// Construct the absolute filename
	char file_path[100] = {0};
	snprintf(file_path, sizeof(file_path), "%s%s%08u.pgm", pgm_path, pgm_dumpname, seq);
	PX4_INFO("%s", file_path);

	int fd = open(file_path, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

	if (fd < 0) {
		PX4_ERR("Dump: Unable to open file");
		return;
	}

	// Write pgm header
	snprintf(&pgm_header[4], 15, "%014d", (int)timestamp);
	size_t written = write(fd, pgm_header, sizeof(pgm_header));

	// Write image data
	size_t total = 0;

	do {
		written = write(fd, data, size);
		total += written;
	} while (total < size);

	PX4_INFO("Wrote %d bytes\n", total);

	close(fd);
}
