/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <thread>
#include <chrono>

#include "autopilot_tester_failure.h"

TEST_CASE("Failure Injection - Reject mid-air when it is disabled", "[multicopter]")
{
	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.set_param_int("SYS_FAILURE_EN", 0);
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off, 1,
			      mavsdk::Failure::Result::Disabled);
	tester.execute_rtl();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

// Soft motor failure (SYS_FAIL_MOT_OFF = 0, default): the injected motor is reported
// as a failed motor, so the control allocator removes it from the allocation and, with
// CA_FAILURE_MODE = 1, also stops the geometrically opposite motor. Runs on the SIH
// hexarotor, where motor i's opposite is motor i+3 (0-based).
TEST_CASE("Failure Injection - Soft motor failure removes motor and opposite from allocation",
	  "[motor_failure_injection]")
{
	const float flight_altitude = 10.0f;
	const float altitude_tolerance = 1.0f;
	const float hover_speed_tolerance = 1.0f;
	const unsigned num_motors = 6;
	const int motor_instance = 1;        // 1-based
	const uint32_t stopped_mask = 0x9;   // motor 0 and its opposite, motor 3 (0-based)

	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.set_param_sys_failure_en(true);
	tester.set_param_mc_airmode(1);      // Enable airmode for control allocation with motor failure
	tester.set_param_ca_failure_mode(1); // Remove failed motor from allocation and stop opposite
	tester.set_takeoff_altitude(flight_altitude);
	tester.enable_actuator_output_status();
	tester.sleep_for(std::chrono::seconds(1)); // Necessary for the takeoff altitude to be applied properly

	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(flight_altitude, std::chrono::seconds(30));
	tester.wait_until_speed_lower_than(hover_speed_tolerance, std::chrono::seconds(30));

	// The vehicle must hold altitude through the failure and the recovery
	tester.start_checking_altitude(altitude_tolerance);

	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off,
			      motor_instance, mavsdk::Failure::Result::Success);
	tester.sleep_for(std::chrono::seconds(5));
	tester.ensure_motors_stopped(stopped_mask, num_motors);

	// Clearing the failure restores all motors
	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Ok,
			      motor_instance, mavsdk::Failure::Result::Success);
	tester.sleep_for(std::chrono::seconds(5));
	tester.ensure_motors_stopped(0, num_motors);

	tester.stop_checking_altitude();

	tester.execute_rtl();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

// Hard motor failure (SYS_FAIL_MOT_OFF = 1, set at boot via PX4_PARAM_SYS_FAIL_MOT_OFF):
// only the injected motor stops, the allocator is not informed and the opposite motor
// keeps running.
TEST_CASE("Failure Injection - Hard motor failure stops only the failed motor",
	  "[motor_failure_injection_hard]")
{
	const float flight_altitude = 10.0f;
	const float altitude_tolerance = 1.0f;
	const float hover_speed_tolerance = 1.0f;
	const unsigned num_motors = 6;
	const int motor_instance = 1; // 1-based

	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.set_param_sys_failure_en(true);
	tester.set_param_mc_airmode(1);
	tester.set_param_ca_failure_mode(1);
	tester.set_takeoff_altitude(flight_altitude);
	tester.enable_actuator_output_status();
	tester.sleep_for(std::chrono::seconds(1)); // Necessary for the takeoff altitude to be applied properly

	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(flight_altitude, std::chrono::seconds(30));
	tester.wait_until_speed_lower_than(hover_speed_tolerance, std::chrono::seconds(30));

	// The vehicle must hold altitude through the unannounced failure and the recovery
	tester.start_checking_altitude(altitude_tolerance);

	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off,
			      motor_instance, mavsdk::Failure::Result::Success);
	tester.sleep_for(std::chrono::seconds(5));
	tester.ensure_motors_stopped(1u << (motor_instance - 1), num_motors);

	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Ok,
			      motor_instance, mavsdk::Failure::Result::Success);
	tester.sleep_for(std::chrono::seconds(5));
	tester.ensure_motors_stopped(0, num_motors);

	tester.stop_checking_altitude();

	tester.execute_rtl();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
}
