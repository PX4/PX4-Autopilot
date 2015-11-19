/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file base.cpp
 *
 * Tests for the estimator base class
 */

#include <cstdio>
#include <random>
#include "../../estimator_base.h"

extern "C" __EXPORT int base_main(int argc, char *argv[]);

int base_main(int argc, char *argv[])
{
	EstimatorBase *base = new EstimatorBase();

	// Test1: feed in fake imu data and check if delta angles are summed correclty
	float delta_vel[3] = { 0.002f, 0.002f, 0.002f};
	float delta_ang[3] = { -0.1f, 0.2f, 0.3f};
	uint32_t time_usec = 1000;


	// simulate 400 Hz imu rate, filter should downsample to 100Hz
	// feed in 2 seconds of data
	for (int i = 0; i < 800; i++) {
		base->setIMUData(time_usec, 2500, 2500, delta_ang, delta_vel);
		time_usec += 2500;
	}

	//base->printStoredIMU();


	// Test2: feed fake imu data and check average imu delta t
	// simulate 400 Hz imu rate, filter should downsample to 100Hz
	// feed in 2 seconds of data
	for (int i = 0; i < 800; i++) {
		base->setIMUData(time_usec, 2500, 2500, delta_ang, delta_vel);
		//base->print_imu_avg_time();
		time_usec += 2500;
	}

	// Test3: feed in slow imu data, filter should now take every sample
	for (int i = 0; i < 800; i++) {
		base->setIMUData(time_usec, 2500, 2500, delta_ang, delta_vel);
		time_usec += 30000;
	}

	//base->printStoredIMU();

	// Test4: Feed in mag data at 50 Hz (too fast), check if filter samples correctly
	float mag[3] = {0.2f, 0.0f, 0.4f};

	for (int i = 0; i < 100; i++) {
		base->setMagData(time_usec, mag);
		time_usec += 20000;
	}

	//base->printStoredMag();

	// Test5: Feed in baro data at 50 Hz (too fast), check if filter samples correctly
	float baro = 120.22f;;
	time_usec = 100000;

	for (int i = 0; i < 100; i++) {
		base->setBaroData(time_usec, &baro);
		baro += 10.0f;
		time_usec += 20000;
	}

	//base->printStoredBaro();

	// Test 5: Run everything rogether in one run
	std::default_random_engine generator;
	std::uniform_int_distribution<int> distribution(-200, 200);

	int imu_sample_period = 2500;
	uint64_t timer = 2000;		// simulation start time
	uint64_t timer_last = timer;

	float airspeed = 0.0f;

	struct gps_message gps = {};
	gps.lat = 40 * 1e7;			// Latitude in 1E-7 degrees
	gps.lon = 5 * 1e7;			// Longitude in 1E-7 degrees
	gps.alt = 200 * 1e3;			// Altitude in 1E-3 meters (millimeters) above MSL
	gps.fix_type = 4;		// 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time
	gps.eph = 5.0f;			// GPS HDOP horizontal dilution of position in m
	gps.epv = 5.0f;			// GPS VDOP horizontal dilution of position in m
	gps.vel_ned_valid = true;	// GPS ground speed is valid

	// simulate two seconds
	for (int i = 0; i < 800; i++) {
		timer += (imu_sample_period + distribution(generator));

		if ((timer - timer_last) > 70000) {
			base->setAirspeedData(timer, &airspeed);
		}

		gps.time_usec = timer;
		gps.time_usec_vel = timer;
		base->setIMUData(timer, timer - timer_last, timer - timer_last, delta_ang, delta_vel);
		base->setMagData(timer, mag);
		base->setBaroData(timer, &baro);
		base->setGpsData(timer, &gps);
		base->print_imu_avg_time();

		timer_last = timer;

	}

	base->printStoredIMU();
	base->printStoredBaro();
	base->printStoredMag();
	base->printStoredGps();

	return 0;
}