/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/hysteresis/hysteresis.h>
#include <lib/perf/perf_counter.h>
#include <lib/mathlib/math/WelfordMeanVector.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/auto_trim_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class FwAutoTrim : public ModuleParams
{
public:
	FwAutoTrim(ModuleParams *parent);
	~FwAutoTrim();

	void reset();
	void update(const matrix::Vector3f &torque_sp, float dt);

	const matrix::Vector3f &getTrim() const { return _trim_validated; }

protected:
	void updateParams() override;

private:
	void publishStatus(const hrt_abstime now);

	uORB::Publication<auto_trim_status_s> _auto_trim_status_pub{ORB_ID(auto_trim_status)};

	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	enum class state {
		idle = auto_trim_status_s::STATE_IDLE,
		sampling = auto_trim_status_s::STATE_SAMPLING,
		sampling_test = auto_trim_status_s::STATE_SAMPLING_TEST,
		verification = auto_trim_status_s::STATE_VERIFICATION,
		complete = auto_trim_status_s::STATE_COMPLETE,
		fail = auto_trim_status_s::STATE_FAIL,
	} _state{state::idle};

	hrt_abstime _state_start_time{0};

	bool _armed{false};
	bool _landed{false};
	bool _fixed_wing{false};
	bool _tailsitter{false};
	float _calibrated_airspeed_m_s{0.f};
	float _cos_tilt{0.f};

	math::WelfordMeanVector<float, 3> _trim_estimate{};
	math::WelfordMeanVector<float, 3> _trim_test{};

	matrix::Vector3f _trim_validated{};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, "[AutoTrim] cycle time")};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_HAS_NUM_ASPD>) _param_sys_has_num_aspd,
		(ParamFloat<px4::params::FW_AIRSPD_MIN>) _param_fw_airspd_min,
		(ParamFloat<px4::params::FW_AIRSPD_MAX>) _param_fw_airspd_max
	)

	static constexpr float _kTiltMaxDeg = 10.f;
};
