/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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
 * Feeds Ekf with Gnss data
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */
#ifndef EKF_GNSS_H
#define EKF_GNSS_H

#include "sensor.h"

#include <lib/gnss/gnss_checks.hpp>

namespace sensor_simulator
{
namespace sensor
{

class Gnss: public Sensor
{
public:
	Gnss(std::shared_ptr<Ekf> ekf);
	~Gnss();

	// Sets both the health time of the checks (GNSS_REQ_TIME) and the EKF's waits (EKF2_REQ_GPS_H)
	void setMinRequiredGnssHealthTime(uint64_t time_us);
	void setCheckMask(int32_t check_mask);
	void setData(const gnssSample &gps);
	void stepHeightByMeters(const float hgt_change);
	void stepHorizontalPositionByMeters(const Vector2f hpos_change);
	void setPositionRateNED(const Vector3f &rate);
	void setAltitude(const float alt);
	void setLatitude(const double lat);
	void setLongitude(const double lon);
	void setVelocity(const Vector3f &vel);
	void setFixType(const int fix_type);
	void setNumberOfSatellites(const int num_satellites);
	void setPdop(const float pdop);

	gnssSample getDefaultGnssData();
	const gnssSample &getData() const { return _gnss_data; }

private:
	void send(uint64_t time) override;

	static constexpr uint64_t kGnssDelayUs{110000};

	gnssSample _gnss_data{};
	Vector3f _gnss_pos_rate{};

	// The sensors module's checks, which set usable on every sample
	GnssChecks _checks{};
	GnssChecks::Params _check_params{};
};

} // namespace sensor
} // namespace sensor_simulator
#endif // EKF_GNSS_H
