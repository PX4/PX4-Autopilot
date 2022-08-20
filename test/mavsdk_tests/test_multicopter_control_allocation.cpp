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
#include <math.h>

#include "autopilot_tester_failure.h"

TEST_CASE("Control Allocation - Remove one motor", "[controlallocation]")
{
	const float flight_altitude = 10.0f;
	const float altitude_tolerance = 4.0f;
	const float hover_speed_tolerance = 1.0f;

	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = false;
	mission_options.relative_altitude_m = flight_altitude;

	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	// Configuration
	tester.set_param_sys_failure_en(true);  // Enable failure injection
	tester.set_param_fd_act_en(true);	// Enable motor failure detection
	tester.set_param_mc_airmode(1);		// Enable airmode for control allocation with motor failure
	tester.set_param_ca_failure_mode(1);	// Enable control allocation handling of failed motor
	tester.prepare_square_mission(mission_options);
	tester.set_takeoff_altitude(flight_altitude);
	tester.set_rtl_altitude(flight_altitude);
	tester.check_tracks_mission(5.f);
	tester.store_home();
	tester.enable_actuator_output_status();
	std::this_thread::sleep_for(std::chrono::seconds(
					    1));  // This is necessary for the takeoff altitude to be applied properly

	// Takeoff
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(flight_altitude, std::chrono::seconds(30));
	tester.wait_until_speed_lower_than(hover_speed_tolerance, std::chrono::seconds(30));

	// Motor failure mid-air
	tester.start_checking_altitude(altitude_tolerance);
	const int motor_instance = 1;
	const unsigned num_motors = 6; // TODO: get from model
	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off, motor_instance,
			      mavsdk::Failure::Result::Success);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	tester.ensure_motor_stopped(motor_instance - 1, num_motors);

	tester.execute_mission();
	tester.stop_checking_altitude();
	tester.ensure_motor_stopped(motor_instance - 1, num_motors); // just to be sure

	// RTL
	tester.execute_rtl();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
	tester.check_home_within(5.f);
}

TEST_CASE("Control Allocation - Remove two motors", "[controlallocation]")
{
	const float flight_altitude = 10.0f;
	const float altitude_tolerance = 4.0f;
	const float hover_speed_tolerance = 1.0f;

	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.set_param_sys_failure_en(true);  // Enable failure injection
	tester.set_param_fd_act_en(true);	// Enable motor failure detection
	tester.set_param_mc_airmode(1);		// Enable airmode for control allocation with motor failure
	tester.set_param_ca_failure_mode(1);	// Enable control allocation handling of failed motor

	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = false;
	mission_options.relative_altitude_m = flight_altitude;

	tester.prepare_square_mission(mission_options);
	tester.set_takeoff_altitude(flight_altitude);
	tester.set_rtl_altitude(flight_altitude);
	tester.check_tracks_mission(5.f);
	tester.store_home();
	std::this_thread::sleep_for(std::chrono::seconds(
					    1));  // This is necessary for the takeoff altitude to be applied properly

	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(flight_altitude, std::chrono::seconds(30));
	tester.wait_until_speed_lower_than(hover_speed_tolerance, std::chrono::seconds(30));

	// Remove two motors opposite of one another on the hexa airframe
	const int first_motor_instance = 1;
	const int second_motor_instance = 2;
	tester.start_checking_altitude(altitude_tolerance);
	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off,
			      first_motor_instance,
			      mavsdk::Failure::Result::Success);
	std::this_thread::sleep_for(std::chrono::seconds(1));
	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off,
			      second_motor_instance,
			      mavsdk::Failure::Result::Success);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	tester.execute_mission();
	tester.stop_checking_altitude();

	// RTL with two motors out won't work because navigator will wait forever until
	// the yaw setpoint is reached during RTL, and it won't land.
	tester.land();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

TEST_CASE("Control Allocation - Remove and restore every motor once", "[controlallocation]")
{
	const float flight_altitude = 10.0f;
	const float altitude_tolerance = 4.0f;
	const float hover_speed_tolerance = 1.0f;

	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();
	tester.set_param_sys_failure_en(true);  // Enable failure injection
	tester.set_param_fd_act_en(true);	// Enable motor failure detection
	tester.set_param_mc_airmode(1);		// Enable airmode for control allocation with motor failure
	tester.set_param_ca_failure_mode(1);	// Enable control allocation handling of failed motor

	AutopilotTester::MissionOptions mission_options;
	mission_options.rtl_at_end = false;
	mission_options.relative_altitude_m = flight_altitude;

	tester.prepare_square_mission(mission_options);
	tester.set_takeoff_altitude(flight_altitude);
	tester.set_rtl_altitude(flight_altitude);
	tester.check_tracks_mission(5.f);
	tester.store_home();
	std::this_thread::sleep_for(std::chrono::seconds(
					    1));  // This is necessary for the takeoff altitude to be applied properly

	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(flight_altitude, std::chrono::seconds(30));
	tester.wait_until_speed_lower_than(hover_speed_tolerance, std::chrono::seconds(30));

	tester.start_checking_altitude(altitude_tolerance);

	for (int m = 1; m <= 6; m++) {
		tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off, m,
				      mavsdk::Failure::Result::Success);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Ok, m,
				      mavsdk::Failure::Result::Success);
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	tester.execute_mission();
	tester.stop_checking_altitude();

	tester.execute_rtl();
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
	tester.check_home_within(5.f);
}

TEST_CASE("Control Allocation - Return home on motor failure", "[controlallocation]")
{
	const float flight_altitude = 10.0f;
	const float hover_speed_tolerance = 1.0f;

	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	// Configuration
	tester.set_param_sys_failure_en(true);  // Enable failure injection
	tester.set_param_fd_act_en(true);	// Enable motor failure detection
	tester.set_param_mc_airmode(1);		// Enable airmode for control allocation with motor failure
	tester.set_param_ca_failure_mode(1);	// Enable control allocation handling of failed motor
	tester.set_param_com_act_fail_act(3);	// RTL on motor failure
	tester.set_takeoff_altitude(flight_altitude);
	tester.store_home();
	std::this_thread::sleep_for(std::chrono::seconds(
					    1));  // This is necessary for the takeoff altitude to be applied properly

	// Takeoff
	tester.arm();
	tester.takeoff();
	tester.wait_until_hovering();
	tester.wait_until_altitude(flight_altitude, std::chrono::seconds(30));
	tester.wait_until_speed_lower_than(hover_speed_tolerance, std::chrono::seconds(30));

	// TODO: Minor improvement, fly forward for a little bit before triggering motor failure to distinguish "RTL" and "Land only"

	// Motor failure mid-air
	const int motor_instance = 1;
	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off, motor_instance,
			      mavsdk::Failure::Result::Success);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// Wait for RTL to trigger automatically
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(180);
	tester.wait_until_disarmed(until_disarmed_timeout);
	tester.check_home_within(5.f);
}

TEST_CASE("Control Allocation - Terminate on motor failure", "[controlallocation]")
{
	const float flight_altitude = 100.0f;
	const float hover_speed_tolerance = 1.0f;

	AutopilotTesterFailure tester;
	tester.connect(connection_url);
	tester.wait_until_ready();

	// Configuration
	tester.set_param_sys_failure_en(true);  // Enable failure injection
	tester.set_param_fd_act_en(true);	// Enable motor failure detection
	tester.set_param_mc_airmode(1);		// Enable airmode for control allocation with motor failure
	tester.set_param_ca_failure_mode(1);	// Enable control allocation handling of failed motor
	tester.set_param_com_act_fail_act(4);	// Terminate on motor failure
	tester.set_takeoff_altitude(flight_altitude);
	std::this_thread::sleep_for(std::chrono::seconds(
					    1));  // This is necessary for the takeoff altitude to be applied properly

	// Takeoff
	tester.arm();
	tester.takeoff();
	tester.wait_until_altitude(flight_altitude, std::chrono::seconds(60));
	tester.wait_until_speed_lower_than(hover_speed_tolerance, std::chrono::seconds(60));

	// Motor failure mid-air
	const int motor_instance = 1;
	tester.inject_failure(mavsdk::Failure::FailureUnit::SystemMotor, mavsdk::Failure::FailureType::Off, motor_instance,
			      mavsdk::Failure::Result::Success);
	std::this_thread::sleep_for(std::chrono::seconds(1));

	// Wait for disarm with a low enough timeout such that it's necessary for the
	// drone to freefall (terminate) in order to disarm quickly enough:
	// h = g/2 * t^2 -> solve for t
	const int seconds_to_touchdown = 2 + sqrt(flight_altitude * 2 / 10.0);
	std::cout << "seconds_to_touchdown: " << seconds_to_touchdown << std::endl;
	std::chrono::seconds until_disarmed_timeout = std::chrono::seconds(seconds_to_touchdown);
	tester.wait_until_disarmed(until_disarmed_timeout);
}

#if 0
// This is for checking that the SITL test is actually capable of detecting the drone crash
// when not reallocating the control allocation on a motor failure
TEST_CASE("Control Allocation - Remove two motors and expect crash", "[controlallocation]")
{
	// TODO
}
#endif

#if 0
TEST_CASE("Control Allocation with multiple sequential motor failures", "[controlallocation]")
{
	// TODO
}
#endif

#if 0
TEST_CASE("Control Allocation with multiple simultaneous motor failures", "[controlallocation]")
{
	// TODO
}
#endif
