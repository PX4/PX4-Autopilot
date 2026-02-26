/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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
 * @file esc_calibration.cpp
 *
 * Definition of esc calibration
 *
 * @author Roman Bapst <roman@px4.io>
 */

#include "esc_calibration.h"
#include "calibration_messages.h"
#include "calibration_routines.h"

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/time.h>
#include <systemlib/mavlink_log.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_test.h>
#include <parameters/param.h>

using namespace time_literals;

bool check_battery_disconnected(orb_advert_t *mavlink_log_pub)
{
	uORB::SubscriptionData<battery_status_s> battery_status_sub{ORB_ID(battery_status)};
	battery_status_sub.update();

	const bool recent_battery_measurement = hrt_absolute_time() < (battery_status_sub.get().timestamp + 1_s);

	if (recent_battery_measurement && battery_status_sub.get().connected) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Disconnect battery and try again");
		return false;

	} else {
		return true;
	}
}

static void set_motor_actuators(uORB::Publication<actuator_test_s> &publisher, float value, bool release_control)
{
	actuator_test_s actuator_test{};
	actuator_test.timestamp = hrt_absolute_time();
	actuator_test.value = value;
	actuator_test.action = release_control ? actuator_test_s::ACTION_RELEASE_CONTROL : actuator_test_s::ACTION_DO_CONTROL;
	actuator_test.timeout_ms = 0;

	for (int i = 0; i < actuator_test_s::MAX_NUM_MOTORS; ++i) {
		actuator_test.function = actuator_test_s::FUNCTION_MOTOR1 + i;
		publisher.publish(actuator_test);
	}
}

int do_esc_calibration(orb_advert_t *mavlink_log_pub)
{
	// 1 Initialization
	bool calibration_failed = false;

	uORB::Publication<actuator_test_s> actuator_test_pub{ORB_ID(actuator_test)};
	// since we publish multiple at once, make sure the output driver subscribes before we publish
	actuator_test_pub.advertise();

	uORB::SubscriptionData<battery_status_s> battery_status_sub{ORB_ID(battery_status)};
	battery_status_sub.update();
	const bool battery_connected_before_calibration = battery_status_sub.get().connected;
	const float current_before_calibration = battery_status_sub.get().current_a;

	calibration_log_info(mavlink_log_pub, CAL_QGC_STARTED_MSG, "esc");

	px4_usleep(10_ms);

	// 2 Set motors to high
	set_motor_actuators(actuator_test_pub, 1.f, false);
	calibration_log_info(mavlink_log_pub, "[cal] Connect battery now");

	hrt_abstime timeout_start = hrt_absolute_time();

	// 3 Wait for user to connect power
	while (true) {
		hrt_abstime now = hrt_absolute_time();
		battery_status_sub.update();

		if (now > (timeout_start + 1_s) && (battery_status_sub.get().current_a > current_before_calibration + 1.f)) {
			// Safety termination, current rises immediately, user didn't unplug power before
			calibration_failed = true;
			break;
		}

		if (!battery_connected_before_calibration && battery_status_sub.get().connected) {
			// Battery connection detected we can go to the next step immediately
			break;
		}

		if (now > (timeout_start + 6_s)) {
			// Timeout, we continue since maybe the battery cannot be detected properly
			// If we abort here and the ESCs are infact connected and started calibrating
			// they will measure the disarmed value as the lower limit instead of the fixed 1000us
			break;
		}

		px4_usleep(50_ms);
	}

	// 4 Wait for ESCs to measure high signal
	if (!calibration_failed) {
		calibration_log_info(mavlink_log_pub, "[cal] Battery connected");
		px4_usleep(3_s);
	}

	// 5 Set motors to low
	if (!calibration_failed) {
		set_motor_actuators(actuator_test_pub, 0.f, false);
	}

	// 6 Wait for ESCs to measure low signal
	if (!calibration_failed) {
		px4_usleep(5_s);
	}

	// 7 release control
	set_motor_actuators(actuator_test_pub, 0.f, true);

	// 8 Report
	if (calibration_failed) {
		calibration_log_critical(mavlink_log_pub, CAL_QGC_FAILED_MSG, "Timeout waiting for battery");
		return PX4_ERROR;

	} else {
		calibration_log_info(mavlink_log_pub, CAL_QGC_DONE_MSG, "esc");
		return PX4_OK;
	}
}
