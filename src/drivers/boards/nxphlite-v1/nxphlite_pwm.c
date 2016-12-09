/************************************************************************************
 *
 *   Copyright (C) 2013, 2015, 2016 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Jordan MacIntyre <jordanroymax@gmail.com>
 *            David Sidrane <david_s5@nscdg.com>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>

#include <sys/types.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/drivers/pwm.h>

#include <arch/board/board.h>

#include "chip.h"
#include "up_arch.h"
#include "kinetis_pwm.h"

#ifdef CONFIG_PWM

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: board_pwm_setup
 *
 * Description:
 *   All Kinetis K architectures must provide the following interface to work with
 *   examples/pwm.
 *
 ************************************************************************************/

int board_pwm_setup(void)
{
	FAR struct pwm_lowerhalf_s *pwm;
	static bool initialized = false;
	int ret;

	/* Have we already initialized? */

	if (!initialized) {
		/* Call nxphlite_pwminitialize() to get an instance of the PWM interface */

		pwm = kinetis_pwminitialize(0);

		if (!pwm) {
			aerr("ERROR: Failed to get the kinetis PWM lower half\n");
			return -ENODEV;
		}

		/* Register the PWM driver at "/dev/pwm0" */

		ret = pwm_register("/dev/pwm0", pwm);

		if (ret < 0) {
			aerr("ERROR: pwm_register failed: %d\n", ret);
			return ret;
		}

		/* Now we are initialized */

		initialized = true;
	}

	return OK;
}

#endif /* CONFIG_PWM */
