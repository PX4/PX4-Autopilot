/****************************************************************************
 *
 *   Copyright (c) 2019 ECL Development Team. All rights reserved.
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
 * This class is providing methods to feed the ECL EKF with measurement.
 * It takes a pointer to the Ekf object and will manipulate the object
 * by call set*Data functions.
 * It simulates the time to allow for sensor data being set at certain rate
 * and also calls the update method of the EKF
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#ifndef EKF_SENSOR_SIMULATOR_H
#define EKF_SENSOR_SIMULATOR_H

#include <memory>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <array>
#include <motion_planning/VelocitySmoothing.hpp>

#include "imu.h"
#include "mag.h"
#include "baro.h"
#include "gps.h"
#include "flow.h"
#include "range_finder.h"
#include "vio.h"
#include "airspeed.h"
#include "EKF/ekf.h"

using namespace sensor_simulator::sensor;

struct sensor_info {
	uint64_t timestamp{};
	enum measurement_t {IMU, MAG, BARO, GPS, AIRSPEED, RANGE, FLOW, VISION, LANDING_STATUS} sensor_type = IMU;
	std::array<double, 10> sensor_data{};
};

class SensorSimulator
{

public:
	SensorSimulator(std::shared_ptr<Ekf> ekf);
	~SensorSimulator() = default;

	uint64_t getTime() const { return _time; };

	void runSeconds(float duration_seconds);
	void runMicroseconds(uint32_t duration);

	void runReplaySeconds(float duration_seconds);
	void runReplayMicroseconds(uint32_t duration);

	void setTrajectoryTargetVelocity(const Vector3f &velocity_target);
	void runTrajectorySeconds(float duration_seconds);
	void runTrajectoryMicroseconds(uint32_t duration);

	void startBaro() { _baro.start(); }
	void stopBaro() { _baro.stop(); }

	void startGps() { _gps.start(); }
	void stopGps() { _gps.stop(); }

	void startFlow() { _flow.start(); }
	void stopFlow() { _flow.stop(); }

	void startRangeFinder() { _rng.start(); }
	void stopRangeFinder() { _rng.stop(); }

	void startExternalVision() { _vio.start(); }
	void stopExternalVision() { _vio.stop(); }

	void startAirspeedSensor() { _airspeed.start(); }
	void stopAirspeedSensor() { _airspeed.stop(); }

	void setGpsLatitude(const double latitude);
	void setGpsLongitude(const double longitude);
	void setGpsAltitude(const float altitude);

	void setImuBias(Vector3f accel_bias, Vector3f gyro_bias);

	void simulateOrientation(Quatf orientation);
	void setOrientation(const Quatf &orientation) { _R_body_to_world = Dcmf(orientation); }
	void setOrientation(const Dcmf &orientation) { _R_body_to_world = orientation; }

	void loadSensorDataFromFile(std::string filename);

	Airspeed    _airspeed;
	Baro        _baro;
	Flow        _flow;
	Gps         _gps;
	Imu         _imu;
	Mag         _mag;
	RangeFinder _rng;
	Vio         _vio;

	VelocitySmoothing _trajectory[3];

private:
	void setSensorDataToDefault();
	void setSensorDataFromReplayData();
	void setSensorRateToDefault();
	void setSingleReplaySample(const sensor_info &sample);
	void setSensorDataFromTrajectory();
	void startBasicSensor();
	void updateSensors();

	std::shared_ptr<Ekf> _ekf{nullptr};

	std::vector<sensor_info> _replay_data{};

	bool _has_replay_data{false};

	uint64_t _current_replay_data_index{0};
	uint64_t _time{0};	// microseconds

	Dcmf _R_body_to_world{};
};
#endif // !EKF_SENSOR_SIMULATOR_H
