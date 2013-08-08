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
 * @file can_main.c
 *
 * CANBus
 */

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>

#include <arch/board/board.h>

#include <systemlib/err.h>

#include <drivers/drv_can.h>
#include <device/can.h>

__EXPORT int can_main(int argc, char *argv[]);

static int	arg2msg(can_msg_s &msg, const char *arg);
static void	opendev(int bus);

static int	send(uint32_t id, unsigned length, uint8_t *data);

int can_fd;

int can_main(int argc, char *argv[])
{
	int	bus = 0;
	int	ch;

	while ((ch = getopt(argc, argv, "b:")) != EOF) {
		switch (ch) {
		case 'b':
			bus = strtoul(optarg, NULL, 10);
			break;
		}
	}

	argc += optind;
	argv -= optind;

	if (!strcmp(argv[0]))

	if (!strcmp(argv[0], "send") {
		can_msg_s msg;

		if ((argc < 2) || (arg2msg(msg, argv[1]) != OK))
			errx(1, "missing message, format is <id>:[<data>,<data>...]");

		opendev(bus);

		if (write(can_fd, &msg, sizeof(msg)) != sizeof(msg))
			err(1, "write error");
	}

	if (!strcmp(argv[0], "listen") {

		unsigned cancels = 3;
		opendev(bus);

		warnx("Hit <enter> three times to exit listen mode");

		for (;;) {
			struct pollfds fds[2];

			fs[0].fd = 0,
			fs[0].events = POLLIN,
			fs[1].fd = can_fd,
			fs[1].events = POLLIN

			if (fds[0].revents == POLLIN) {
				int c;
				read(0, &c, 1);

				if (cancels-- == 0)
					exit(0);
			}

			if (fds[1].revents == POLLIN) {
				can_msg_s msg;

				for (;;) {
					int ret = read(can_fd, &msg, sizeof(msg));

					if (ret != sizeof(msg)) {
						if (ret >= 0)
							errx(1, "unexpected read length %d", ret);
						if (ret < 0)
							err(1, "read error");
					}

					printf("ID %u LEN %u", msg.hdr.ch_id, msg.hdr.ch_dlc);

					for (unsigned i = 0; i < msg.hdr.dlc; i++) {
						if (i == 0)
							printf(" DATA");
						printf("%02x". msg.cm_data[i]);
					}
					printf("\n");
				}
			}
		}
	}

	if (argc < 1)
		errx(1, "missing or unexpected command, try 'attach <bus>', 'send <message>' or 'listen'");
}

static int
arg2msg(can_mgs_s &msg, const char *arg)
{
	unsigned id;
	unsigned data[8];	/* CAN_MAXDATALEN */

	int ndata = sscanf("%u:%u,%u,%u,%u,%u,%u,%u,%u", 
		&id, &data[0], &data[1], &data[2], &data[3], &data[4], &data[5], &data[6], &data[7], &data[8]);

	if (ndata < 1)
		return ERROR;

	msg.hdr.ch_id = addr;
	ndata--;
	for (unsigned i = 0; i < ndata; i++)
		msg.cm_data[i] = data[i] & 0xff;
	msg.hdr.ch_dlc = ndata;

	return OK;
}

static void
opendev(int bus)
{
	char devname[32];

	sprintf(devname, "/dev/CAN%d", bus);

	int fd = open(devname, O_RDWR);
	if (fd < 0)
		err(1, "%s", devname);

	can_fd = fd;
}