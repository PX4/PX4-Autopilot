/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file bl_update.c
 *
 * STM32F4 bootloader update tool.
 */

#include <px4_config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#include <arch/board/board.h>
#include <nuttx/progmem.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"

#define BL_FILE_SIZE_LIMIT	16384

__EXPORT int bl_update_main(int argc, char *argv[]);

static void setopt(void);

int
bl_update_main(int argc, char *argv[])
{
	if (argc != 2) {
		errx(1, "missing firmware filename or command");
	}

	if (!strcmp(argv[1], "setopt")) {
		setopt();
	}

	int fd = open(argv[1], O_RDONLY);

	if (fd < 0) {
		err(1, "open %s", argv[1]);
	}

	struct stat s;

	if (stat(argv[1], &s) != 0) {
		err(1, "stat %s", argv[1]);
	}

	/* sanity-check file size */
	if (s.st_size > BL_FILE_SIZE_LIMIT) {
		errx(1, "%s: file too large (limit: %u, actual: %d)", argv[1], BL_FILE_SIZE_LIMIT, s.st_size);
	}

	uint8_t *buf = malloc(s.st_size);

	if (buf == NULL) {
		errx(1, "failed to allocate %u bytes for firmware buffer", s.st_size);
	}

	if (read(fd, buf, s.st_size) != s.st_size) {
		err(1, "firmware read error");
	}

	close(fd);

	uint32_t *hdr = (uint32_t *)buf;

	if ((hdr[0] < 0x20000000) ||			/* stack not below RAM */
	    (hdr[0] > (0x20000000 + (128 * 1024))) ||	/* stack not above RAM */
	    (hdr[1] < 0x08000000) ||			/* entrypoint not below flash */
	    ((hdr[1] - 0x08000000) > 16384)) {		/* entrypoint not outside bootloader */
		free(buf);
		errx(1, "not a bootloader image");
	}

	warnx("image validated, erasing bootloader...");
	usleep(10000);

	/* prevent other tasks from running while we do this */
	sched_lock();

    up_progmem_erasepage(0);
    
	warnx("flashing...");

    up_progmem_write(up_progmem_getaddress(0), buf, s.st_size);

	warnx("bootloader update complete");

	/* unlock the scheduler */
	sched_unlock();

	free(buf);
	exit(0);
}

static void
setopt(void)
{
	volatile uint32_t *optcr = (volatile uint32_t *)0x40023c14;

	const uint16_t opt_mask = (3 << 2);		/* BOR_LEV bitmask */
	const uint16_t opt_bits = (0 << 2);		/* BOR = 0, setting for 2.7-3.6V operation */

	if ((*optcr & opt_mask) == opt_bits) {
		errx(0, "option bits are already set as required");
	}

	/* unlock the control register */
	volatile uint32_t *optkeyr = (volatile uint32_t *)0x40023c08;
	*optkeyr = 0x08192a3bU;
	*optkeyr = 0x4c5d6e7fU;

	if (*optcr & 1) {
		errx(1, "option control register unlock failed");
	}

	/* program the new option value */
	*optcr = (*optcr & ~opt_mask) | opt_bits | (1 << 1);

	usleep(1000);

	if ((*optcr & opt_mask) == opt_bits) {
		errx(0, "option bits set");
	}

	errx(1, "option bits setting failed; readback 0x%04x", *optcr);

}
