/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <ecl/airdata/WindEstimator.hpp>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_workqueue.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/wind_estimate.h>

using namespace time_literals;

#define SCHEDULE_INTERVAL	100000	/**< The schedule interval in usec (10 Hz) */

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

class WindEstimatorModule : public ModuleBase<WindEstimatorModule>, public ModuleParams
{
public:

	WindEstimatorModule();

	~WindEstimatorModule();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	// run the main loop
	void cycle();

	int print_status() override;

private:
	static struct work_s	_work;

	WindEstimator _wind_estimator;
	orb_advert_t 	_wind_est_pub{nullptr};			/**< wind estimate topic */

	int _vehicle_attitude_sub{-1};
	int _vehicle_local_position_sub{-1};
	int _airspeed_sub{-1};
	int _param_sub{-1};

	perf_counter_t _perf_elapsed{};
	perf_counter_t _perf_interval{};

	int	_instance{-1};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::WEST_W_P_NOISE>) _param_west_w_p_noise,
		(ParamFloat<px4::params::WEST_SC_P_NOISE>) _param_west_sc_p_noise,
		(ParamFloat<px4::params::WEST_TAS_NOISE>) _param_west_tas_noise,
		(ParamFloat<px4::params::WEST_BETA_NOISE>) _param_west_beta_noise,
		(ParamInt<px4::params::WEST_TAS_GATE>) _param_west_tas_gate,
		(ParamInt<px4::params::WEST_BETA_GATE>) _param_west_beta_gate
	)

	static void	cycle_trampoline(void *arg);
	int 		start();

	void		update_params();

	bool 		subscribe_topics();
};

work_s	WindEstimatorModule::_work = {};

WindEstimatorModule::WindEstimatorModule():
	ModuleParams(nullptr)
{
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_param_sub = orb_subscribe(ORB_ID(parameter_update));

	// initialise parameters
	update_params();

	_perf_elapsed = perf_alloc_once(PC_ELAPSED, "wind_estimator elapsed");
	_perf_interval = perf_alloc_once(PC_INTERVAL, "wind_estimator interval");
}

WindEstimatorModule::~WindEstimatorModule()
{
	orb_unsubscribe(_vehicle_attitude_sub);
	orb_unsubscribe(_vehicle_local_position_sub);
	orb_unsubscribe(_airspeed_sub);
	orb_unsubscribe(_param_sub);

	orb_unadvertise(_wind_est_pub);

	perf_free(_perf_elapsed);
	perf_free(_perf_interval);
}

int
WindEstimatorModule::task_spawn(int argc, char *argv[])
{
	/* schedule a cycle to start things */
	work_queue(LPWORK, &_work, (worker_t)&WindEstimatorModule::cycle_trampoline, nullptr, 0);

	// wait until task is up & running
	if (wait_until_running() < 0) {
		_task_id = -1;

	} else {
		_task_id = task_id_is_work_queue;
		return PX4_OK;
	}

	return PX4_ERROR;
}

void
WindEstimatorModule::cycle_trampoline(void *arg)
{
	WindEstimatorModule *dev = reinterpret_cast<WindEstimatorModule *>(arg);

	// check if the trampoline is called for the first time
	if (!dev) {
		dev = new WindEstimatorModule();

		if (!dev) {
			PX4_ERR("alloc failed");
			return;
		}

		_object.store(dev);
	}

	if (dev) {
		dev->cycle();
	}
}

void
WindEstimatorModule::cycle()
{
	perf_count(_perf_interval);
	perf_begin(_perf_elapsed);

	bool param_updated;
	orb_check(_param_sub, &param_updated);

	if (param_updated) {
		update_params();
	}

	bool lpos_valid = false;
	bool att_valid = false;
	bool airspeed_valid = false;

	const hrt_abstime time_now_usec = hrt_absolute_time();

	// validate required conditions for the filter to fuse measurements

	vehicle_attitude_s att = {};

	if (orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &att) == PX4_OK) {
		att_valid = (time_now_usec - att.timestamp < 1_s) && (att.timestamp > 0);
	}

	vehicle_local_position_s lpos = {};

	if (orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &lpos) == PX4_OK) {
		lpos_valid = (time_now_usec - lpos.timestamp < 1_s) && (lpos.timestamp > 0) && lpos.v_xy_valid;
	}

	// update wind and airspeed estimator
	_wind_estimator.update(time_now_usec);

	if (lpos_valid && att_valid) {

		Vector3f vI(lpos.vx, lpos.vy, lpos.vz);
		Quatf q(att.q);

		// sideslip fusion
		_wind_estimator.fuse_beta(time_now_usec, vI, q);

		// additionally, for airspeed fusion we need to have recent measurements
		airspeed_s airspeed = {};

		if (orb_copy(ORB_ID(airspeed), _airspeed_sub, &airspeed) == PX4_OK) {
			airspeed_valid = (time_now_usec - airspeed.timestamp < 1_s) && (airspeed.timestamp > 0);
		}

		if (airspeed_valid) {
			Vector3f vel_var{Dcmf(q) *Vector3f{lpos.evh, lpos.evh, lpos.evv}};

			// airspeed fusion
			_wind_estimator.fuse_airspeed(time_now_usec, airspeed.indicated_airspeed_m_s, vI, Vector2f{vel_var(0), vel_var(1)});
		}

		// if we fused either airspeed or sideslip we publish a wind_estimate message
		wind_estimate_s wind_est = {};

		wind_est.timestamp = time_now_usec;
		float wind[2];
		_wind_estimator.get_wind(wind);
		wind_est.windspeed_north = wind[0];
		wind_est.windspeed_east = wind[1];
		float wind_cov[2];
		_wind_estimator.get_wind_var(wind_cov);
		wind_est.variance_north = wind_cov[0];
		wind_est.variance_east = wind_cov[1];
		wind_est.tas_innov = _wind_estimator.get_tas_innov();
		wind_est.tas_innov_var = _wind_estimator.get_tas_innov_var();
		wind_est.beta_innov = _wind_estimator.get_beta_innov();
		wind_est.beta_innov_var = _wind_estimator.get_beta_innov_var();
		wind_est.tas_scale = _wind_estimator.get_tas_scale();

		orb_publish_auto(ORB_ID(wind_estimate), &_wind_est_pub, &wind_est, &_instance, ORB_PRIO_DEFAULT);
	}

	perf_end(_perf_elapsed);

	if (should_exit()) {
		exit_and_cleanup();

	} else {
		/* schedule next cycle */
		work_queue(LPWORK, &_work, (worker_t)&WindEstimatorModule::cycle_trampoline, this, USEC2TICK(SCHEDULE_INTERVAL));
	}
}

void WindEstimatorModule::update_params()
{
	updateParams();

	// update wind & airspeed scale estimator parameters
	_wind_estimator.set_wind_p_noise(_param_west_w_p_noise.get());
	_wind_estimator.set_tas_scale_p_noise(_param_west_sc_p_noise.get());
	_wind_estimator.set_tas_noise(_param_west_tas_noise.get());
	_wind_estimator.set_beta_noise(_param_west_beta_noise.get());
	_wind_estimator.set_tas_gate(_param_west_tas_gate.get());
	_wind_estimator.set_beta_gate(_param_west_beta_gate.get());
}

int WindEstimatorModule::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = WindEstimatorModule::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int WindEstimatorModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module runs a combined wind and airspeed scale factor estimator.
If provided the vehicle NED speed and attitude it can estimate the horizontal wind components based on a zero
sideslip assumption. This makes the estimator only suitable for fixed wing vehicles.
If provided an airspeed measurement this module also estimates an airspeed scale factor based on the following model:
measured_airspeed = scale_factor * real_airspeed.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wind_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int WindEstimatorModule::print_status()
{
	perf_print_counter(_perf_elapsed);
	perf_print_counter(_perf_interval);

	if (_instance > -1) {
		uORB::Subscription<wind_estimate_s> est{ORB_ID(wind_estimate), (unsigned)_instance};
		est.update();

		print_message(est.get());
	} else {
		PX4_INFO("Running, but never published");
	}

	return 0;
}

extern "C" __EXPORT int wind_estimator_main(int argc, char *argv[]);

int
wind_estimator_main(int argc, char *argv[])
{
	return WindEstimatorModule::main(argc, argv);
}
