/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: 	James Goppert
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
 * @file RoboClawDevice.cpp
 * RoboClaw Driver
 *
 * @author James Goppert
 */

#pragma once

// system includes
#include <inttypes.h> // uint16_t
#include <stdlib.h> // strtoul
#include <string.h> // strcmp
#include <unistd.h> // usleep
#include <sched.h> // task_create
#include <fcntl.h> // open
#include <termios.h> // tcgetattr etc.
#include <math.h> // fabs
#include <stdio.h> // snprintf

// px4 includes
#include <systemlib/err.h> // errx
#include <systemlib/systemlib.h> // SCHED_DEFAULT
#include <systemlib/scheduling_priorities.h> // SCHED_PRIORITY
#include <drivers/device/device.h> // device::CDev
#include <drivers/drv_sensor.h> // ioctl flags
#include <drivers/drv_hrt.h> // hrt_absolute_time
#include <modules/uORB/topics/actuator_controls.h> // actuator_controls
#include <modules/uORB/topics/encoders.h> // encoders
#include <mavlink/mavlink_log.h>

// roboclaw
#include "RoboClawDevice.h"
#define V_INT(V) (V*10) // note that in docs V_MIN has different equation, but doesn't match firmware
#define DUTY_INT(DUTY) (DUTY*1500)

static const uint8_t scaleVolts = 10;
static const uint16_t scaleDuty = 1500;

RoboClawDevice::RoboClawDevice(const char *port, uint8_t address,
			       uint32_t tout, bool doack) :
	CDev("roboclaw", "/dev/roboclaw"),
	m_roboclaw(port, tout, doack),
	// subscriptions
	m_controls(nullptr, ORB_ID(actuator_controls_1), 1),
	// publication
	m_outputs(nullptr, ORB_ID(actuator_outputs_1)),
	m_encoders(nullptr, ORB_ID(encoders)),
	m_mavlink_fd(-1),
	m_address(address),
	m_period(1e6 / 400), // 400 Hz
	m_error(ROBO_ERROR_NONE),
	m_timeErrorSent(0),
	m_timeActuatorCommand(0),
	m_timeUpdate(0),
	m_motor1Overflows(0),
	m_motor2Overflows(0),
	m_accel(DUTY_INT(20))
{
	init();
	// stop motors
	m_roboclaw.DutyAccelM1M2(m_address, 0, m_accel, 0, m_accel);
}

RoboClawDevice::~RoboClawDevice()
{
	// stop motors
	m_roboclaw.DutyAccelM1M2(m_address, 0, m_accel, 0, m_accel);
	// close mavlink port
	::close(m_mavlink_fd);
}

int RoboClawDevice::init()
{
	int ret = device::CDev::init();

	if (ret < 0) { return ret; }

	if (m_mavlink_fd == -1) {
		m_mavlink_fd = ::open(MAVLINK_LOG_DEVICE, 0);

		if (m_mavlink_fd < 0) { return m_mavlink_fd; }
	}

	m_roboclaw.SetLogicVoltages(m_address, 6.0 * scaleVolts, 13.0 * scaleVolts);
	m_roboclaw.SetMainVoltages(m_address, 10.0 * scaleVolts, 13.0 * scaleVolts);
	m_roboclaw.WriteNVM(m_address);
	m_roboclaw.ResetEncoders(m_address);
	char version[200];
	m_roboclaw.ReadVersion(m_address, version);
	mavlink_log_critical(m_mavlink_fd, version);
	//warnx(version);
	return OK;
}

void RoboClawDevice::update()
{

	// get current time
	uint64_t now = hrt_absolute_time();

	// ack
	bool error_status_valid = false;

	// read error status
	uint8_t error = m_roboclaw.ReadError(m_address,
					     &error_status_valid);

	// if failed to read error status, use last
	if (!error_status_valid) {
		warnx("read error failed");
		error = m_error;
	}

	// if there is no error now
	if (error == ROBO_ERROR_NONE) {
		// if there was an error, notify ground station
		if (m_error != ROBO_ERROR_NONE) {
			mavlink_log_critical(m_mavlink_fd, "[ROBO] OK");
		}

		// if there is an error now

	} else {
		// if it is new or we haven't sent message for a second,
		// send to ground station
		if (error != m_error || ((now - m_timeErrorSent) > 1000000)) {

			// general handling of errors
			char error_msg[200];
			errorToString(error, error_msg, 200);
			mavlink_log_critical(m_mavlink_fd, error_msg);
			m_timeErrorSent = now;
			//warnx(error_msg);

			// handle logic battery voltage error
			if (error & ROBO_ERROR_LOGIC_BATTERY_LOW ||
			    error & ROBO_ERROR_LOGIC_BATTERY_HIGH) {
				uint16_t voltage = m_roboclaw.ReadLogicBattVoltage(m_address);
				uint16_t min = 0;
				uint16_t max = 0;

				if (m_roboclaw.ReadMinMaxLogicVoltages(m_address, min, max)) {
					warnx("logic voltage out of range: "
					      "min: %5.1f max: %5.1f V: %5.1f",
					      min / 10.0, max / 10.0, voltage / 10.0);
				}
			}

			// handle main battery voltage error
			if (error & ROBO_ERROR_MAIN_BATTERY_LOW ||
			    error & ROBO_ERROR_MAIN_BATTERY_HIGH) {
				uint16_t voltage = m_roboclaw.ReadMainBatteryVoltage(m_address);
				uint16_t min = 0;
				uint16_t max = 0;

				if (m_roboclaw.ReadMinMaxMainVoltages(m_address, min, max)) {
					warnx("main voltage out of range: "
					      "min: %5.1f max: %5.1f V: %5.1f",
					      min / 10.0, max / 10.0, voltage / 10.0);
				}
			}

			// handle overcurrent error
			if (error & ROBO_ERROR_M1_OVERCURRENT ||
			    error & ROBO_ERROR_M2_OVERCURRENT) {
				uint8_t current1;
				uint8_t current2;

				if (m_roboclaw.ReadCurrents(m_address, current1, current2)) {
					warnx("motor overcurrent: "
					      "c1: %6.2f c2: %6.2f", current1 / 100.0, current2 / 100.0);
				}
			}

			// handle temperature error
			if (error & ROBO_ERROR_TEMPERATURE) {
				uint16_t temperature;

				if (m_roboclaw.ReadTemp(m_address, temperature)) {
					warnx("temperature over max: %7.1f", temperature / 10.0);
				}
			}
		}
	}

	// save error for next update
	m_error = error;

	// sleep to set rate
	int32_t timeElapsed = now - m_timeUpdate;
	m_timeUpdate = now;
	int32_t sleepTime = m_period - timeElapsed;

	if (sleepTime > 5000) {
		usleep(sleepTime - 5000);

	} else if (sleepTime < -20000 && m_timeUpdate != 0) {
		mavlink_log_critical(m_mavlink_fd,
				     "[ROBO] slow, %10.2f Hz", 1.0e6 / timeElapsed);
	}

	// read encoders
	uint8_t status = 0;
	uint32_t counts = 0;
	bool valid_encoder1 = false;
	bool valid_encoder2 = false;

	// position 1
	counts = m_roboclaw.ReadEncM1(m_address, &status,
				      &valid_encoder1);

	if (valid_encoder1) {
		m_encoders.counts[0] = encoderToInt64(counts, status,
						      &m_motor1Overflows);

	} else {
		warnx("encoder 1 read failed");
	}

	// position 2
	counts = m_roboclaw.ReadEncM2(m_address, &status,
				      &valid_encoder2);

	if (valid_encoder2) {
		m_encoders.counts[1] = encoderToInt64(counts, status,
						      &m_motor2Overflows);

	} else {
		warnx("encoder 2 read failed");
	}

	bool valid_speed1 = false;
	bool valid_speed2 = false;
	int32_t speedRes = 0; // counts/125 of a second
	// note doesn't match docs, uint32_t in docs and
	// stat doesn't matter
	
	uint8_t speedSampleRate = 125; // from docs, measures counts/125 of a second
	float vel_factor = 2.45; // XXX this is required for counts/sec to match vel

	// speed 1
	speedRes =  m_roboclaw.ReadISpeedM1(m_address, &status, &valid_speed1);

	if (valid_speed1) {
		m_encoders.velocity[0] = speedRes * speedSampleRate * vel_factor;
	}

	// speed 2
	speedRes =  m_roboclaw.ReadISpeedM2(m_address, &status, &valid_speed2);

	if (valid_speed2) {
		m_encoders.velocity[1] = speedRes * speedSampleRate * vel_factor;
	}

	// test for all data valid
	bool all_data_valid = valid_encoder1 &&
			      valid_encoder2 && valid_speed1 && valid_speed2;

	// publish new data if it is all valid
	if (all_data_valid) {
		m_encoders.timestamp = now;
		m_encoders.update();
		// let waiting processes know the driver
		// has new information
		poll_notify(POLLIN);
	}

	// default commands to zero unless we receive data
	float controlPitch = 0;
	float controlYaw = 0;

	// send new actuator commands to motors if there is no error
	if ((m_error == ROBO_ERROR_NONE) && all_data_valid) {
		if (m_controls.updated()) {
			m_timeActuatorCommand = now;
			m_controls.update(); // get data
			// NOTE channels (0 roll, 1 pitch, 2 yaw)
			controlPitch = m_controls.control[1];
			controlYaw = m_controls.control[2];
		}
	}

	// warn if not getting control packets
	if (now - m_timeActuatorCommand > 1000000) {
		warnx("not receiving control packets");
		m_timeActuatorCommand = now;
	}

	// do mixing
	float control[2];
	control[0] = controlPitch + controlYaw;
	control[1] = controlPitch - controlYaw;

	// limit
	for (int i = 0; i < 2; i++) {
		if (control[i] > 1) { control[i] = 1; }

		if (control[i] < -1) { control[i] = -1; }
	}

	int16_t duty[2];

	for (int i = 0; i < 2; i++) {
		duty[i] = scaleDuty * control[i];
	}

	// publish duty cycles
	m_outputs.timestamp = now;
	m_outputs.output[0] = duty[0] + scaleDuty; // have to add scale so positive (like servo)
	m_outputs.output[1] = duty[1] + scaleDuty;
	m_outputs.noutputs = 2;
	m_outputs.update(); // publish

	// send command to motor
	m_roboclaw.DutyAccelM1M2(m_address,
				 duty[0], m_accel,
				 duty[1], m_accel); // note, don't want to add scaleDuty here
}

void RoboClawDevice::errorToString(uint8_t error,
				   char *msg, size_t n)
{
	char buf[200] = "[ROBO] ";
	const char *m1oc = "m1 oc,";
	const char *m2oc = "m2 oc,";
	const char *estop = "estop,";
	const char *temp = " temp,";
	const char *mbatl = " main low,";
	const char *mbath = " main high,";
	const char *lbatl = " logic low,";
	const char *lbath = " logic high,";

	if (error & ROBO_ERROR_M1_OVERCURRENT) {
		strncat(buf, m1oc, strnlen(m1oc, 20));
	}

	if (error & ROBO_ERROR_M2_OVERCURRENT) {
		strncat(buf, m2oc, strnlen(m2oc, 20));
	}

	if (error & ROBO_ERROR_ESTOP) {
		strncat(buf, estop, strnlen(estop, 20));
	}

	if (error & ROBO_ERROR_TEMPERATURE) {
		strncat(buf, temp, strnlen(temp, 20));
	}

	if (error & ROBO_ERROR_MAIN_BATTERY_LOW) {
		strncat(buf, mbatl, strnlen(mbatl, 20));
	}

	if (error & ROBO_ERROR_MAIN_BATTERY_HIGH) {
		strncat(buf, mbath, strnlen(mbath, 20));
	}

	if (error & ROBO_ERROR_LOGIC_BATTERY_LOW) {
		strncat(buf, lbatl, strnlen(lbatl, 20));
	}

	if (error & ROBO_ERROR_LOGIC_BATTERY_HIGH) {
		strncat(buf, lbath, strnlen(lbath, 20));
	}

	strncpy(msg, buf, n);
}

int64_t RoboClawDevice::encoderToInt64(uint32_t count, uint8_t status,
				       int32_t *overflows)
{
	static int64_t overflowAmount = 0x100000000LL;

	if (status & ROBO_ENCODER_OVERFLOW) {
		(*overflows) += 1;
	}

	if (status & ROBO_ENCODER_UNDERFLOW) {
		(*overflows) -= 1;
	}

	return int64_t(count) + (*overflows) * overflowAmount;
}

// vi:noet:smarttab:autoindent:ts=4:sw=4:tw=78
