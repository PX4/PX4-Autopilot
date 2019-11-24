/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file VehicleModelEstimator.hpp
 *
 * Vehicle model estimator.
 *
 * To estimate the vehicle inertia, the estimator low-pass filters and
 * computes the time derivative of the vehicle angular acceleration
 * and of the vehicle torque setpoint, then applies a least-mean-square.
 * Similarly, to estimate the vehicle mass, the estimator low-pass
 * filters and computes the time derivative of the vehicle acceleration
 * and of the vehicle thrust setpoint, then applies a least-mean-square.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#pragma once

#include <lib/mathlib/math/filter/LowPassFilter2pVector3f.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/ecl/EKF/ekf.h>	// included for RingBuffer
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_model.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>


class VehicleModelEstimator : public ModuleBase<VehicleModelEstimator>, public ModuleParams, public px4::WorkItem
{
public:
	VehicleModelEstimator();

	virtual ~VehicleModelEstimator();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	void Run() override;

	bool init();

private:

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void		parameters_updated();

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};			/**< parameter updates subscription */
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};		/**< vehicle control mode subscription */
	uORB::Subscription _vehicle_torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint)}; 	/**< vehicle torque setpoint subscription */
	uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)}; 	/**< vehicle thrust setpoint subscription */
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};	  	/**< vehicle land detected subscription */

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_acceleration_sub{this, ORB_ID(vehicle_angular_acceleration)};
	uORB::SubscriptionCallbackWorkItem _vehicle_acceleration_sub{this, ORB_ID(vehicle_acceleration)};

	uORB::Publication<vehicle_model_s>		   _vehicle_model_pub{ORB_ID(vehicle_model)};				/**< vehicle model publication */

	bool _armed{false};
	bool _landed{true};
	bool _maybe_landed{true};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	static constexpr const float initial_update_rate_hz = 1000.f; /**< loop update rate used for initialization */
	hrt_abstime _task_start{hrt_absolute_time()};
	hrt_abstime _prev_publish_time{0};

	// Inertia estimation data
	matrix::SquareMatrix3f _I;  		/**< Inertia matrix entered via parameters */
	matrix::SquareMatrix3f _I_est_inv;      /**< Inverse of estimated inertia matrix */

	// Angular acceleration data
	math::LowPassFilter2pVector3f _lpf_ang_accel{0.f, 0.f}; 	/**< low-pass filters for angular acceleration */
	matrix::Vector3f _ang_accel_filtered;				/**< low-pass filtered angular acceleration */
	matrix::Vector3f _ang_accel_filtered_diff;			/**< time derivative of the low-pass filtered angular acceleration */
	hrt_abstime _timestamp_ang_accel{0};
	float _loop_update_rate_hz_ang_accel{initial_update_rate_hz};
	int _loop_counter_ang_accel{0};
	float _dt_accumulator_ang_accel{0.0f};

	// Torque setpoint data
	RingBuffer<vehicle_torque_setpoint_s> _torque_sp_buffer;
	math::LowPassFilter2pVector3f _lpf_torque_sp{0.f, 0.f}; 	/**< low-pass filters for torque setpoint */
	matrix::Vector3f _torque_sp_filtered;				/**< low-pass filtered torque setpoint */
	matrix::Vector3f _torque_sp_filtered_diff;			/**< time derivative of the low-pass filtered torque setpoint */
	hrt_abstime _timestamp_torque_sp{0};
	float _loop_update_rate_hz_torque_sp{initial_update_rate_hz};
	int _loop_counter_torque_sp{0};
	float _dt_accumulator_torque_sp{0.0f};

	// Mass estimation data
	float _mass{0.0f};		/**< Mass entered via parameters */
	float _mass_est_inv{1.0f};	/**< Inverse of estimated mass */

	// Linear acceleration data
	math::LowPassFilter2pVector3f _lpf_lin_accel{0.f, 0.f}; 	/**< low-pass filters for linear acceleration */
	matrix::Vector3f _lin_accel_filtered;				/**< low-pass filtered linear acceleration */
	matrix::Vector3f _lin_accel_filtered_diff;			/**< time derivative of the low-pass filtered linear acceleration */
	hrt_abstime _timestamp_lin_accel{0};
	float _loop_update_rate_hz_lin_accel{initial_update_rate_hz};
	int _loop_counter_lin_accel{0};
	float _dt_accumulator_lin_accel{0.0f};

	// Thrust setpoint data
	RingBuffer<vehicle_thrust_setpoint_s> _thrust_sp_buffer;
	math::LowPassFilter2pVector3f _lpf_thrust_sp{0.f, 0.f}; 	/**< low-pass filters for thrust setpoint */
	matrix::Vector3f _thrust_sp_filtered;				/**< low-pass filtered thrust setpoint */
	matrix::Vector3f _thrust_sp_filtered_diff;			/**< time derivative of the low-pass filtered thrust setpoint */
	hrt_abstime _timestamp_thrust_sp{0};
	float _loop_update_rate_hz_thrust_sp{initial_update_rate_hz};
	int _loop_counter_thrust_sp{0};
	float _dt_accumulator_thrust_sp{0.0f};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::VM_MASS>) _param_vm_mass,			/**< vehicle mass */

		(ParamFloat<px4::params::VM_INERTIA_XX>) _param_vm_inertia_xx,		/**< vehicle inertia */
		(ParamFloat<px4::params::VM_INERTIA_YY>) _param_vm_inertia_yy,
		(ParamFloat<px4::params::VM_INERTIA_ZZ>) _param_vm_inertia_zz,
		(ParamFloat<px4::params::VM_INERTIA_XY>) _param_vm_inertia_xy,
		(ParamFloat<px4::params::VM_INERTIA_XZ>) _param_vm_inertia_xz,
		(ParamFloat<px4::params::VM_INERTIA_YZ>) _param_vm_inertia_yz,

		(ParamInt<px4::params::VM_EST_EN>) _param_vm_est_en,
		(ParamFloat<px4::params::VM_EST_PUBRATE>) _param_vm_est_pubrate,
		(ParamFloat<px4::params::VM_EST_CUTOFF>) _param_vm_est_cutoff,
		(ParamFloat<px4::params::VM_EST_GAIN>) _param_vm_est_gain,
		(ParamFloat<px4::params::VM_EST_DELAY>) _param_vm_est_delay
	)
};

