/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#pragma once

#include <errno.h>

#include <px4_platform_common/board_common.h>

#if defined(TIOCSSWAP) || defined(TIOCSSINGLEWIRE)
# include <sys/ioctl.h>
#endif

/**
 * Apply board RC UART wiring (RX/TX swap and/or single-wire).
 * Boards that cannot HW-swap use single-wire as the swap.
 */
static inline void rc_uart_configure(int fd, const char *device)
{
	if ((fd < 0) || (device == nullptr)) {
		return;
	}

	if (board_rc_swap_rxtx(device)) {
		int rv = -ENOTTY;
#if defined(TIOCSSWAP)
		rv = ioctl(fd, TIOCSSWAP, SER_SWAP_ENABLED);
#endif
#if defined(RC_SERIAL_SWAP_USING_SINGLEWIRE) && defined(TIOCSSINGLEWIRE)

		if (rv != 0) {
			ioctl(fd, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);
		}

#endif
		(void)rv;
	}

	if (board_rc_singlewire(device)) {
#if defined(TIOCSSINGLEWIRE)
		ioctl(fd, TIOCSSINGLEWIRE, SER_SINGLEWIRE_ENABLED);
#endif
	}
}
