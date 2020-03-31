/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file rpm_simulator.c
 * This simple app produces RPM message.
 * Usage: rpm_simulator 10
 *
 * @author Roman Dvorak <dvorakroman@thunderfly.cz>
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/rpm.h>

__EXPORT int rpm_simulator_main(int argc, char *argv[]);

int rpm_simulator_main(int argc, char *argv[])
{
	struct rpm_s rpm;
	memset(&rpm, 0, sizeof(rpm));
	orb_advert_t rpm_pub = orb_advertise(ORB_ID(rpm), &rpm);

	uint64_t timestamp_us = hrt_absolute_time();
	float frequency = atof(argv[1]);

	rpm.timestamp = timestamp_us;
	rpm.indicated_frequency_rpm = frequency;
	rpm.estimated_accurancy_rpm = frequency/100.0f;

	orb_publish(ORB_ID(rpm), rpm_pub, &rpm);

	PX4_INFO("RPM message with %f RPM was published", (double)frequency);

	return 0;
}
