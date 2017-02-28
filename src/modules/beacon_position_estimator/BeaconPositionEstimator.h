/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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

/*
 * @file BeaconPositionEstimator.h
 * Beacon position estimator. Filter and publish the position of a ground beacon as observed by an onboard sensor.
 *
 * @author Nicolas de Palezieux <ndepal@gmail.com>
 */

#pragma once

#include <px4_workqueue.h>
#include <drivers/drv_hrt.h>
#include <systemlib/param/param.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/irlock_report.h>
#include <uORB/topics/beacon_position.h>
#include <uORB/topics/parameter_update.h>
#include <matrix/math.hpp>
#include <mathlib/mathlib.h>
#include <matrix/Matrix.hpp>
#include "KalmanFilter.h"


namespace beacon_position_estimator
{

class BeaconPositionEstimator
{
public:

	BeaconPositionEstimator();
	virtual ~BeaconPositionEstimator();

	/*
	 * @return true if this task is currently running.
	 */
	inline bool is_running() const
	{
		return _taskIsRunning;
	}

	/*
	 * Tells the task that it should exit.
	 */
	void stop();

	/*
	 * Get the work queue going.
	 */
	int start();

protected:
	/*
	 * Called once to initialize uORB topics.
	 */
	void _initialize_topics();

	/*
	 * Update uORB topics.
	 */
	void _update_topics();

	/*
	 * Update parameters.
	 */
	void _update_params();

	/*
	 * Convenience function for polling uORB subscriptions.
	 *
	 * @return true if there was new data and it was successfully copied
	 */
	static bool _orb_update(const struct orb_metadata *meta, int handle, void *buffer);

	/* Run main loop at this rate in Hz. */
	static constexpr uint32_t beacon_position_estimator_UPDATE_RATE_HZ = 50;
	/* timeout after which filter is reset if beacon not seen */
	static constexpr uint32_t beacon_position_estimator_TIMEOUT_US = 2000000;

	orb_advert_t _beaconPositionPub;
	struct beacon_position_s _beacon_position;

	int _parameterSub;

private:

	enum class BeaconMode {
		Moving = 0,
		Stationary,
		KnownLocation
	};

	/**
	* Handles for parameters
	**/
	struct {
		param_t acc_unc;
		param_t meas_unc;
		param_t pos_unc_init;
		param_t vel_unc_init;
		param_t mode;
	} _paramHandle;

	struct {
		float acc_unc;
		float meas_unc;
		float pos_unc_init;
		float vel_unc_init;
		BeaconMode mode;
	} _params;

	int _vehicleLocalPositionSub;
	int _attitudeSub;
	int _sensorCombinedSub;
	int _irlockReportSub;

	struct vehicle_local_position_s		_vehicleLocalPosition,
			      _vehicleLocalPosition_last; // store two most recent to compute acceleration
	struct vehicle_attitude_s			_vehicleAttitude;
	struct sensor_combined_s			_sensorCombined;
	struct irlock_report_s				_irlockReport;

	// keep track of which topics we have received
	bool _vehicleLocalPosition_valid;
	bool _vehicleLocalPosition_last_valid;
	bool _vehicleAttitude_valid;
	bool _sensorCombined_valid;
	bool _new_irlockReport;
	bool _estimator_initialized;

	matrix::Dcm<float> _R_att;
	matrix::Vector<float, 2> _rel_pos;
	KalmanFilter _kalman_filter_x;
	KalmanFilter _kalman_filter_y;
	hrt_abstime _last_predict; // timestamp of last filter prediction
	hrt_abstime _last_update; // timestamp of last filter update (used to check timeout)

	static void _cycle_trampoline(void *arg);

	void _cycle();

	void _check_params(const bool force);

	void _update_state();

	bool _taskShouldExit;
	bool _taskIsRunning;

	struct work_s	_work;
};


} // namespace beacon_position_estimator
