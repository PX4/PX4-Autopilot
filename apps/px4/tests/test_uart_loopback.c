/****************************************************************************
 * px4/sensors/test_gpio.c
 *
 *  Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *            Lorenz Meier <lm@inf.ethz.ch>
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
#include <fcntl.h>
#include <errno.h>
#include <debug.h>

#include <arch/board/board.h>

#include "tests.h"

#include <math.h>
#include <float.h>


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: test_led
 ****************************************************************************/

int test_uart_loopback(int argc, char *argv[])
{

	int uart5_nread = 0;
	int uart2_nread = 0;
	int uart5_nwrite = 0;
	int uart2_nwrite = 0;

	int uart1 = open("/dev/ttyS0", O_RDWR | O_NOCTTY); //

	/* assuming NuttShell is on UART1 (/dev/ttyS0) */
	int uart2 = open("/dev/ttyS1", O_RDWR | O_NONBLOCK | O_NOCTTY); //
	int uart5 = open("/dev/ttyS2", O_RDWR | O_NONBLOCK | O_NOCTTY); //

	if (uart2 < 0) {
		printf("ERROR opening UART2, aborting..\n");
		return uart2;
	}

	if (uart5 < 0) {
		printf("ERROR opening UART5, aborting..\n");
		exit(uart5);
	}

	uint8_t sample_uart1[] = {'C', 'O', 'U', 'N', 'T', ' ', '#', '\n'};
	uint8_t sample_uart2[] = {'C', 'O', 'U', 'N', 'T', ' ', '#', 0};
	uint8_t sample_uart5[] = {'C', 'O', 'U', 'N', 'T', ' ', '#', 0};

	int i, r;

	for (i = 0; i < 1000; i++) {
//		printf("TEST #%d\n",i);
		write(uart1, sample_uart1, sizeof(sample_uart1));

		/* uart2 -> uart5 */
		r = write(uart2, sample_uart2, sizeof(sample_uart2));

		if (r > 0)
			uart2_nwrite += r;

//		printf("TEST #%d\n",i);
		write(uart1, sample_uart1, sizeof(sample_uart1));

		/* uart2 -> uart5 */
		r = write(uart5, sample_uart5, sizeof(sample_uart5));

		if (r > 0)
			uart5_nwrite += r;

//		printf("TEST #%d\n",i);
		write(uart1, sample_uart1, sizeof(sample_uart1));

		/* try to read back values */
		do {
			r = read(uart5, sample_uart2, sizeof(sample_uart2));

			if (r > 0)
				uart5_nread += r;
		} while (r > 0);

//		printf("TEST #%d\n",i);
		write(uart1, sample_uart1, sizeof(sample_uart1));

		do {
			r = read(uart2, sample_uart5, sizeof(sample_uart5));

			if (r > 0)
				uart2_nread += r;
		} while (r > 0);

//		printf("TEST #%d\n",i);
//		write(uart1, sample_uart1, sizeof(sample_uart5));
	}

	for (i = 0; i < 200000; i++) {

		/* try to read back values */
		r = read(uart5, sample_uart2, sizeof(sample_uart2));

		if (r > 0)
			uart5_nread += r;

		r = read(uart2, sample_uart5, sizeof(sample_uart5));

		if (r > 0)
			uart2_nread += r;

		if ((uart2_nread == uart2_nwrite) && (uart5_nread == uart5_nwrite))
			break;
	}


	close(uart1);
	close(uart2);
	close(uart5);

	printf("uart2_nwrite %d\n", uart2_nwrite);
	printf("uart5_nwrite %d\n", uart5_nwrite);
	printf("uart2_nread  %d\n", uart2_nread);
	printf("uart5_nread  %d\n", uart5_nread);


	return 0;
}
