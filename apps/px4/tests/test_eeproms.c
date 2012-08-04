/****************************************************************************
 * px4/eeproms/test_eeproms.c
 *
 *  Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name NuttX nor the names of its contributors may be
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "tests.h"

#include <arch/board/drv_eeprom.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  onboard_eeprom(int argc, char *argv[]);
static int	baseboard_eeprom(int argc, char *argv[]);

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct {
	const char	*name;
	const char	*path;
	int	(* test)(int argc, char *argv[]);
} eeproms[] = {
	{"onboard_eeprom",	"/dev/eeprom",	onboard_eeprom},
	{"baseboard_eeprom",	"/dev/baseboard_eeprom",	baseboard_eeprom},
	{NULL, NULL, NULL}
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int
onboard_eeprom(int argc, char *argv[])
{
	printf("\tonboard_eeprom: test start\n");
	fflush(stdout);

	int		fd;
	uint8_t	buf1[210] = {' ', 'P', 'X', '4', ' ', 'E', 'E', 'P', 'R', 'O', 'M', ' ', 'T', 'E', 'S', 'T', ' '};
	int		ret;
	bool force_write = false;
	if (strcmp(argv[0], "jig") == 0) force_write = true;

	/* fill with spaces */
	//memset(buf1+16, 'x', sizeof(buf1-16));

	/* fill in some magic values at magic positions */
	buf1[63] = 'E';
	buf1[64] = 'S';
	buf1[127] = 'F';
	buf1[128] = 'T';

	/* terminate string */
	buf1[sizeof(buf1) - 1] = '\0';

	fd = open("/dev/eeprom", O_RDWR | O_NONBLOCK);

	if (fd < 0) {
		printf("onboard eeprom: open fail\n");
		return ERROR;
	}

	/* read data */
	ret = read(fd, buf1, 1);

	if (ret != 1) {
		printf("\tonboard eeprom: ERROR: reading first byte fail: %d\n", ret);

		switch (-ret) {
		case EPERM:
			printf("\treason: %s\n", EPERM_STR);
			break;

		case ENOENT:
			printf("\treason: %s\n", ENOENT_STR);
			break;

		case ESRCH:
			printf("\treason: %s\n", ESRCH_STR);
			break;

		case EINTR:
			printf("\treason: %s\n", EINTR_STR);
			break;

		}
	}

	printf("\tonboard eeprom: first byte: %d\n", buf1[0]);
	if (!force_write) {
		printf("\tonboard eeprom: WARNING: FURTHER TEST STEPS WILL DESTROY YOUR FLIGHT PARAMETER CONFIGURATION. PROCEED? (y/N)\n");

		printf("Input: ");
		char c = getchar();
		printf("%c\n", c);
		if (c != 'y' && c != 'Y') {
			/* not yes, abort */
			close(fd);

			/* Let user know everything is ok */
			printf("\tOK: onboard eeprom test aborted by user, read test successful\r\n");
			return OK;
		}
	}

	printf("\tonboard eeprom: proceeding with write test\r\n");

	/* increment counter */
	buf1[0] = buf1[0] + 1;

	/* rewind to the start of the file */
	lseek(fd, 0, SEEK_SET);

	/* write data */
	ret = write(fd, buf1, sizeof(buf1));

	if (ret != sizeof(buf1)) {
		printf("\tonboard eeprom: ERROR: write fail: %d\n", (char)ret);

		switch (-ret) {
		case EPERM:
			printf("\treason: %s\n", EPERM_STR);
			break;

		case ENOENT:
			printf("\treason: %s\n", ENOENT_STR);
			break;

		case ESRCH:
			printf("\treason: %s\n", ESRCH_STR);
			break;

		case EINTR:
			printf("\treason: %s\n", EINTR_STR);
			break;

		}

		//return ERROR;
	}

	/* rewind to the start of the file */
	lseek(fd, 0, SEEK_SET);

	/* read data */
	ret = read(fd, buf1, sizeof(buf1));

	if (ret != sizeof(buf1)) {
		printf("\tonboard eeprom: ERROR: read fail: %d\n", ret);

		switch (-ret) {
		case EPERM:
			printf("\treason: %s\n", EPERM_STR);
			break;

		case ENOENT:
			printf("\treason: %s\n", ENOENT_STR);
			break;

		case ESRCH:
			printf("\treason: %s\n", ESRCH_STR);
			break;

		case EINTR:
			printf("\treason: %s\n", EINTR_STR);
			break;

		}

		return ERROR;

	} else {
		/* enforce null termination and print as string */
		if (buf1[sizeof(buf1) - 1] != 0) {
			printf("\tWARNING: Null termination in file not present as expected, enforcing it now..\r\n");
			buf1[sizeof(buf1) - 1] = '\0';
		}

		/* read out counter and replace val */
		int counter = buf1[0];
		printf("\tonboard eeprom: count: #%d, read values: %s\n", counter, (char *)buf1 + 1);
		printf("\tAll %d bytes:\n\n\t", sizeof(buf1));

		for (int i = 0; i < sizeof(buf1); i++) {
			printf("0x%02x ", buf1[i]);

			if (i % 8 == 7) printf("\n\t");

			if (i % 64 == 63) printf("\n\t");
		}

		/* end any open line */
		printf("\n\n");
	}

	close(fd);

	/* Let user know everything is ok */
	printf("\tOK: onboard eeprom passed all tests successfully\n");
	return ret;
}

static int
baseboard_eeprom(int argc, char *argv[])
{
	printf("\tbaseboard eeprom: test start\n");
	fflush(stdout);

	int		fd;
	uint8_t	buf[128] = {'R', 'E', 'A', 'D', ' ', 'F', 'A', 'I', 'L', 'E', 'D', '\0'};
	int		ret;

	fd = open("/dev/baseboard_eeprom", O_RDONLY | O_NONBLOCK);

	if (fd < 0) {
		printf("\tbaseboard eeprom: open fail\n");
		return ERROR;
	}

	/* read data */
	ret = read(fd, buf, sizeof(buf));
	/* set last char to string termination */
	buf[127] = '\0';

	if (ret != sizeof(buf)) {
		printf("\tbaseboard eeprom: ERROR: read fail\n", ret);
		return ERROR;

	} else {
		printf("\tbaseboard eeprom: string: %s\n", (char *)buf);
	}

	close(fd);

	/* XXX more tests here */

	/* Let user know everything is ok */
	printf("\tOK: baseboard eeprom passed all tests successfully\n");
	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_eeproms
 ****************************************************************************/

int test_eeproms(int argc, char *argv[])
{
	unsigned	i;

	printf("Running EEPROMs tests:\n\n");
	fflush(stdout);

	for (i = 0; eeproms[i].name; i++) {
		printf("  eeprom: %s\n", eeproms[i].name);
		eeproms[i].test(argc, argv);
		fflush(stdout);
		/* wait 100 ms to make sure buffer is emptied */
		usleep(100000);
	}

	return 0;
}
