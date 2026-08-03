/****************************************************************************
 *
 *   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
 * @file navputer.hpp
 * Implementation of the attitude and position estimator.
 *
 * @author Roman Bapst
 */

#ifndef NAVPUTER_HPP
#define NAVPUTER_HPP

#include "EKF/ekf.h"

#include <float.h>

#include <containers/LockGuard.hpp>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/time.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/estimator_bias.h>
#include <uORB/topics/estimator_bias3d.h>
#include <uORB/topics/estimator_event_flags.h>
#include <uORB/topics/estimator_innovations.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/estimator_fusion_control.h>
#include <uORB/topics/launch_detection_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/yaw_estimator_status.h>

#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/ranging_beacon.h>

#include <uORB/topics/navput_attitude.h>
#include <uORB/topics/navput_local_position.h>
#include <uORB/topics/navput_status_flags.h>
#include <uORB/topics/navput_fusion_control.h>

using namespace time_literals;

extern pthread_mutex_t navputer_module_mutex;

class Navputer final : public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Navputer() = delete;
	Navputer(const px4::wq_config_t &config, bool replay_mode);
	~Navputer() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	int print_status(bool verbose = false);

	bool should_exit() const { return _task_should_exit.load(); }

	void request_stop() { _task_should_exit.store(true); }

	static void lock_module() { pthread_mutex_lock(&navputer_module_mutex); }
	static bool trylock_module() { return (pthread_mutex_trylock(&navputer_module_mutex) == 0); }
	static void unlock_module() { pthread_mutex_unlock(&navputer_module_mutex); }

private:
	// Used to check, save and use learned accel/gyro/mag biases
	struct InFlightCalibration {
		hrt_abstime last_us{0};         ///< last time the EKF was operating a mode that estimates accelerometer biases (uSec)
		hrt_abstime total_time_us{0};   ///< accumulated calibration time since the last save
		matrix::Vector3f bias{};
		bool cal_available{false};      ///< true when an unsaved valid calibration for the XYZ accelerometer bias is available
	};

	void Run() override;

	void AdvertiseTopics();
	void VerifyParams();

	void PublishAttitude(const hrt_abstime &timestamp);
	void PublishLocalPosition(const hrt_abstime &timestamp);
	void PublishStatusFlags(const hrt_abstime &timestamp);
	void PublishFusionControl(const hrt_abstime &timestamp);

	void UpdateMagSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateBaroSample(ekf2_timestamps_s &ekf2_timestamps);
	void UpdateRangingBeaconSample(ekf2_timestamps_s &ekf2_timestamps);

	void UpdateCalibration(const hrt_abstime &timestamp, InFlightCalibration &cal, const matrix::Vector3f &bias,
			       const matrix::Vector3f &bias_variance, float bias_limit, bool bias_valid, bool learning_valid);
	void UpdateAccelCalibration(const hrt_abstime &timestamp);
	void UpdateGyroCalibration(const hrt_abstime &timestamp);
	void UpdateMagCalibration(const hrt_abstime &timestamp);

	px4::atomic_bool _task_should_exit{false};

	// time slip monitoring
	uint64_t _integrated_time_us = 0;	///< integral of gyro delta time from start (uSec)
	uint64_t _start_time_us = 0;		///< system time at EKF start (uSec)
	int64_t _last_time_slip_us = 0;		///< Last time slip (uSec)

	InFlightCalibration _accel_cal{};
	InFlightCalibration _gyro_cal{};

	uint8_t _accel_calibration_count{0};
	uint8_t _gyro_calibration_count{0};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_combined_sub{this, ORB_ID(sensor_combined)};
	uORB::SubscriptionCallbackWorkItem _vehicle_imu_sub{this, ORB_ID(vehicle_imu)};
	// Baro
	uint32_t _device_id_baro{0};
	uint8_t _baro_calibration_count {0};
	uORB::Subscription _airdata_sub{ORB_ID(vehicle_air_data)};

	// Magnetometer
	uint32_t _device_id_mag {0};
	uint8_t _mag_calibration_count{0};
	InFlightCalibration _mag_cal{};
	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};

	// Beacons
	uORB::Subscription _ranging_beacon_sub {ORB_ID(ranging_beacon)};

	uORB::Publication<navput_attitude_s>           		_attitude_pub{ORB_ID(navput_attitude)};
	uORB::Publication<navput_local_position_s>     		_local_position_pub{ORB_ID(navput_local_position)};

	hrt_abstime _last_status_flags_publish{0};
	uint64_t _filter_control_status{0};
	uint32_t _filter_fault_status{0};
	uint32_t _filter_control_status_changes{0};
	uint32_t _filter_fault_status_changes{0};
	uORB::PublicationMulti<navput_status_flags_s>     	_status_flags_pub{ORB_ID(navput_status_flags)};

	uORB::PublicationMulti<navput_fusion_control_s>   	_fc_pub{ORB_ID(navput_fusion_control)};

	bool _callback_registered{false};


	Ekf _ekf;

	parameters *_params;
	FusionControl &_fc;

	DEFINE_PARAMETERS(
		// ranging beacon fusion
		(ParamExtInt<px4::params::NPT_RNGBC_CTRL>) _param_npt_rngbc_ctrl,
		(ParamExtFloat<px4::params::NPT_RNGBC_DELAY>) _param_npt_rngbc_delay,
		(ParamExtFloat<px4::params::NPT_RNGBC_NOISE>) _param_npt_rngbc_noise,
		(ParamExtFloat<px4::params::NPT_RNGBC_GATE>) _param_npt_rngbc_gate
	)
};
#endif // !NAVPUTER_HPP
