/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#ifndef EKF_MOCAP_HPP
#define EKF_MOCAP_HPP

#include "common.h"
#include "RingBuffer.h"

#if defined(CONFIG_EKF2_MOCAP) && defined(MODULE_NAME)

#if defined(MODULE_NAME)
# include <px4_platform_common/module_params.h>
# include <uORB/PublicationMulti.hpp>
# include <uORB/Subscription.hpp>
# include <uORB/topics/estimator_aid_source1d.h>
# include <uORB/topics/estimator_aid_source2d.h>
# include <uORB/topics/vehicle_odometry.h>
#endif // MODULE_NAME

class Ekf;

class Mocap : public ModuleParams
{
public:
	Mocap() : ModuleParams(nullptr) {}
	~Mocap() = default;

	void update(Ekf &ekf, const estimator::imuSample &imu_delayed);

	void updateParameters()
	{
		updateParams();
	}

private:

	struct MocapSample {
		uint64_t    time_us{};     ///< timestamp of the measurement (uSec)
		Vector3f    pos{};         ///< XYZ position in external vision's local reference frame (m) - Z must be aligned with down axis
		Quatf       quat{};        ///< quaternion defining rotation from body to earth frame
		Vector3f    position_var{};    ///< XYZ position variances (m**2)
		Vector3f    orientation_var{}; ///< orientation variance (rad**2)
		PositionFrame pos_frame = PositionFrame::LOCAL_FRAME_FRD;
		uint8_t     reset_counter{};
		int8_t     quality{};     ///< quality indicator between 0 and 100
	};

	RingBuffer<MocapSample> _mocap_buffer{20}; // TODO: size with _obs_buffer_length and actual publication rate
	uint64_t _time_last_buffer_push{0};

	enum state {
		stopped,
		starting,
		active,
		stopping,
		resetting,
	};

#if defined(MODULE_NAME)
	uORB::PublicationMulti<estimator_aid_source2d_s> _estimator_aid_src_mocap_xy_pub {ORB_ID(estimator_aid_src_mocap_xy)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_mocap_z_pub{ORB_ID(estimator_aid_src_mocap_z)};
	uORB::PublicationMulti<estimator_aid_source1d_s> _estimator_aid_src_mocap_yaw_pub{ORB_ID(estimator_aid_src_mocap_yaw)};

	uORB::Subscription _mocap_sub{ORB_ID(vehicle_mocap_odometry)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::EKF2_MOCAP_CTRL>) _param_ekf2_mocap_ctrl,
		(ParamFloat<px4::params::EKF2_MOCAP_DELAY>) _param_ekf2_mocap_delay,
		(ParamFloat<px4::params::EKF2_MOCAP_NOISE>) _param_ekf2_mocap_noise,
		(ParamFloat<px4::params::EKF2_MOCAP_GATE>) _param_ekf2_mocap_gate
	)

#endif // MODULE_NAME
};

#endif // CONFIG_EKF2_MOCAP

#endif // !EKF_MOCAP_HPP
