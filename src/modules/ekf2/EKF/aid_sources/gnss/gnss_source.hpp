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


#ifndef EKF_GNSS_SOURCE_HPP
#define EKF_GNSS_SOURCE_HPP

#include "../../common.h"

#if defined(CONFIG_EKF2_GNSS)

#include "gnss_checks.hpp"

#include <lib/ringbuffer/TimestampedRingBuffer.hpp>
#include <uORB/topics/estimator_aid_source2d.h>
#include <uORB/topics/estimator_aid_source3d.h>

class Ekf;

class GnssSource
{
public:
	struct Params {
		int32_t ctrl{0}; ///< EKF2_GPS<i>_CTRL bitmask (GnssCtrl)
	};

	GnssSource(estimator::parameters &p, uint32_t &min_health_time_us, estimator::filter_control_status_u &control_status) :
		_checks{p.ekf2_gps_check, p.ekf2_req_nsats, p.ekf2_req_pdop, p.ekf2_req_eph, p.ekf2_req_epv, p.ekf2_req_sacc,
			p.ekf2_req_hdrift, p.ekf2_req_vdrift, p.ekf2_req_fix, p.ekf2_vel_lim, min_health_time_us, control_status}
	{}

	~GnssSource() { delete _buffer; }

	Params params{};

	void setSlot(uint8_t slot) { _slot = slot; }

	void setData(const estimator::gnssSample &sample, uint8_t buffer_length, uint64_t min_obs_interval_us, float dt_ekf_avg,
		     uint64_t time_latest_us);

	bool ctrl(estimator::GnssCtrl bit) const { return params.ctrl & static_cast<int32_t>(bit); }

	bool isVelFusing(const Ekf &ekf) const;
	bool isPosFusing(const Ekf &ekf) const;

	// fusing any of its data (height and dual antenna yaw only from the receiver selected for them)
	bool isActive(const estimator::filter_control_status_u &cs) const
	{
		return _vel_active || _pos_active || (_hgt_source && cs.flags.gps_hgt) || (_yaw_source && cs.flags.gnss_yaw);
	}

	const estimator::GnssChecks &checks() const { return _checks; }
	const estimator::gnssSample &sampleDelayed() const { return _sample_delayed; }

private:
	friend class Ekf;
	friend class GnssAiding;

	// other_slot_vel/pos_fusing: another receiver currently constrains the velocity/position drift
	void update(Ekf &ekf, const estimator::imuSample &imu_delayed, bool other_slot_vel_fusing, bool other_slot_pos_fusing);

	void controlVelFusion(Ekf &ekf, bool force_reset, bool other_slot_fusing);
	void controlPosFusion(Ekf &ekf, bool force_reset, bool other_slot_fusing);
	bool isVelResetAllowed(const Ekf &ekf, bool other_slot_fusing) const;
	bool isPosResetAllowed(const Ekf &ekf, bool other_slot_fusing) const;

	// stop velocity and position fusion, and whatever else this receiver feeds (height, yaw, yaw estimator)
	void stop(Ekf &ekf);
	void stopVel();
	void stopPos();

	TimestampedRingBuffer<estimator::gnssSample> *_buffer{nullptr};
	uint64_t _time_last_buffer_push{0};
	uint64_t _time_last_yaw_buffer_push{0};
	estimator::gnssSample _sample_delayed{};
	bool _data_ready{false};	///< new data has fallen behind the fusion time horizon
	bool _intermittent{true};

	estimator::GnssChecks _checks;

	estimator_aid_source2d_s _aid_src_pos{};
	estimator_aid_source3d_s _aid_src_vel{};

	bool _vel_active{false};
	bool _pos_active{false};
	bool _fault{false};

	// roles of this receiver, assigned by GnssAiding
	bool _hgt_source{false};
	bool _yaw_source{false};
	bool _gsf_source{false};

	uint8_t _slot{0};
};

class GnssAiding
{
public:
	GnssAiding(estimator::parameters &p, uint32_t &min_health_time_us, estimator::filter_control_status_u &control_status) :
		_sources{{p, min_health_time_us, control_status}, {p, min_health_time_us, control_status}}
	{
		static_assert(estimator::MAX_GNSS_INSTANCES == 2, "initialise every source");

		for (uint8_t i = 0; i < estimator::MAX_GNSS_INSTANCES; i++) {
			_sources[i].setSlot(i);
		}

		_sources[0].params.ctrl = static_cast<int32_t>(estimator::GnssCtrl::HPOS) | static_cast<int32_t>(estimator::GnssCtrl::VEL);
	}

	// run all receiver state machines, every receiver may fuse in the same update
	void update(Ekf &ekf, const estimator::imuSample &imu_delayed);

	void setData(const estimator::gnssSample &sample, uint8_t instance, uint8_t buffer_length, uint64_t min_obs_interval_us,
		     float dt_ekf_avg, uint64_t time_latest_us)
	{
		if (instance < estimator::MAX_GNSS_INSTANCES) {
			_sources[instance].setData(sample, buffer_length, min_obs_interval_us, dt_ekf_avg, time_latest_us);
		}
	}

	GnssSource &source(uint8_t instance) { return _sources[instance]; }
	const GnssSource &source(uint8_t instance) const { return _sources[instance]; }

	// lowest slot currently fusing, otherwise lowest slot with data (for single-instance consumers)
	uint8_t primarySlot() const;

	// receiver used for GNSS height, -1 if none
	int8_t heightSlot() const { return _hgt_slot; }

	// stop every receiver
	void stop(Ekf &ekf);

	// filter (re)initialisation
	void reset();

	float maxActiveVelTestRatioXY() const;
	float maxActiveVelTestRatioZ() const;
	float maxActivePosTestRatio() const;
	float maxActiveVelInnovNormXY() const;
	float maxActivePosInnovNorm() const;
	bool anyInnovationBad() const;

private:
	bool intended(const Ekf &ekf, uint8_t slot) const;
	int8_t selectSlot(const Ekf &ekf, estimator::GnssCtrl bit, bool fallback_to_any) const;
	int8_t selectGsfSlot(const Ekf &ekf) const;
	void updateStatusFlags(Ekf &ekf) const;

	GnssSource _sources[estimator::MAX_GNSS_INSTANCES];

	int8_t _hgt_slot{-1};
	int8_t _yaw_slot{-1};
};

#endif // CONFIG_EKF2_GNSS

#endif // !EKF_GNSS_SOURCE_HPP
