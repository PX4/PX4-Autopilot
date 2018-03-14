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

#include <ecl/airdata/WindEstimator.hpp>

#include <px4_module.h>
#include <px4_workqueue.h>
#include <uORB/topics/vehicle_local_position.h>

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <systemlib/param/param.h>

#define SCHEDULE_INTERVAL	100000	/**< The schedule interval in usec (10 Hz) */

class WindEstimatorModule : public ModuleBase<WindEstimatorModule>
{
public:

	WindEstimatorModule();

	~WindEstimatorModule();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static WindEstimatorModule *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * run the main loop: if running as task, continuously iterate, otherwise execute only one single cycle
	 */
	void cycle();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	static struct work_s	_work;

	WindEstimator _wind_estimator;
	orb_advert_t 	_wind_est_pub{nullptr};			/**< wind estimate topic */

	hrt_abstime _time_last_airspeed_fused{0};		/* timestamp of when last time airspeed measurement was fused into wind estimator */
	hrt_abstime _time_last_beta_fused{0};		/* timestamp of when last time sideslip measurement was fused into wind estimator */

	int _vehicle_attitude_sub{-1};
	int _vehicle_local_position_sub{-1};
	int _airspeed_sub{-1};

	static void	cycle_trampoline(void *arg);
	int 		start();

	void		update_params();

	bool 		subscribe_topics();
};

work_s	WindEstimatorModule::_work = {};

WindEstimatorModule::WindEstimatorModule()
{
	_vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
}

WindEstimatorModule::~WindEstimatorModule()
{
	orb_unsubscribe(_vehicle_attitude_sub);
	orb_unsubscribe(_vehicle_local_position_sub);
	orb_unsubscribe(_airspeed_sub);

	orb_unadvertise(_wind_est_pub);
}

int
WindEstimatorModule::task_spawn(int argc, char *argv[])
{
	/* schedule a cycle to start things */
	work_queue(LPWORK, &_work, (worker_t)&WindEstimatorModule::cycle_trampoline, nullptr, 0);

	// wait until task is up & running (the mode_* commands depend on it)
	if (wait_until_running() < 0) {
		_task_id = -1;
		return -1;
	}

	return PX4_OK;
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

		_object = dev;
	}

	dev->cycle();
}

void
WindEstimatorModule::cycle()
{
	vehicle_attitude_s vehicle_attitude = {};
	vehicle_local_position_s vehicle_local_position = {};
	airspeed_s airspeed = {};


	hrt_abstime time_now_usec = hrt_absolute_time();

	// update wind and airspeed estimator
	_wind_estimator.update(time_now_usec);

	// limit airspeed and sideslip fusion rate to 20Hz
	bool fuse_airspeed = (time_now_usec - _time_last_airspeed_fused) > 50 * 1000;
	bool fuse_beta = (time_now_usec - _time_last_beta_fused) > 50 * 1000;

	if (fuse_beta || fuse_airspeed) {

		// we need to fuse, so update topics
		orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &vehicle_attitude);
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &airspeed);
		orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &vehicle_local_position);

		matrix::Dcmf R_to_earth(matrix::Quatf(vehicle_attitude.q));
		matrix::Vector3f vI(vehicle_local_position.vx, vehicle_local_position.vy, vehicle_local_position.vz);

		if (fuse_beta && vehicle_local_position.v_xy_valid) {
			_wind_estimator.fuse_beta(time_now_usec, &vI._data[0][0], vehicle_attitude.q);
			_time_last_beta_fused = time_now_usec;
		}

		if (fuse_airspeed && vehicle_local_position.v_xy_valid) {
			matrix::Vector3f vel_var(vehicle_local_position.evh, vehicle_local_position.evh, vehicle_local_position.evv);
			vel_var = R_to_earth * vel_var;
			_wind_estimator.fuse_airspeed(time_now_usec, airspeed.indicated_airspeed_m_s, &vI._data[0][0],
						      &vel_var._data[0][0]);
			_time_last_airspeed_fused = time_now_usec;
		}
	}


	// if we fused either airspeed or sideslip we publish a wind_estimate message
	if (_time_last_beta_fused == time_now_usec || _time_last_airspeed_fused == time_now_usec) {
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

		int instance;
		orb_publish_auto(ORB_ID(wind_estimate), &_wind_est_pub, &wind_est, &instance, ORB_PRIO_DEFAULT);
	}

	if (should_exit()) {
		exit_and_cleanup();

	} else {
		/* schedule next cycle */
		work_queue(HPWORK, &_work, (worker_t)&WindEstimatorModule::cycle_trampoline, this, USEC2TICK(SCHEDULE_INTERVAL));
	}
}

void WindEstimatorModule::update_params()
{
	param_t p_wind_p_noise = param_find("WEST_W_P_NOISE");
	param_t p_tas_scale_p_noise = param_find("WEST_SC_P_NOISE");
	param_t p_tas_noise = param_find("WEST_TAS_NOISE");
	param_t p_beta_noise = param_find("WEST_BETA_NOISE");

	float wind_p_noise = 0.0f;
	param_get(p_wind_p_noise, &wind_p_noise);

	float tas_scale_p_noise = 0.0f;
	param_get(p_tas_scale_p_noise, &tas_scale_p_noise);

	float tas_noise = 0.0f;
	param_get(p_tas_noise, &tas_noise);

	float beta_noise = 0.0f;
	param_get(p_beta_noise, &beta_noise);

	// update wind & airspeed scale estimator parameters
	_wind_estimator.set_wind_p_noise(wind_p_noise);
	_wind_estimator.set_tas_scale_p_noise(tas_scale_p_noise);
	_wind_estimator.set_tas_noise(tas_noise);
	_wind_estimator.set_beta_noise(beta_noise);

}

WindEstimatorModule *WindEstimatorModule::instantiate(int argc, char *argv[])
{
	// No arguments to parse. We also know that we should run as task
	return new WindEstimatorModule();
}

int WindEstimatorModule::custom_command(int argc, char *argv[])
{
	/* start the FMU if not running */
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
If provided the vehicle NED speed and the vehicle attitude it can estimate the horizontal wind components based on a zero
sideslip assumption. This makes the estimator only suitable for fixed wing vehicles.
If additionally provided an airspeed measurment this module also estimates an airspeed scale factor based on the following model:
measured_airspeed = scale_factor * real_airspeed.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wind_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int WindEstimatorModule::print_status()
{
	return 0;
}

extern "C" __EXPORT int wind_estimator_main(int argc, char *argv[]);

int
wind_estimator_main(int argc, char *argv[])
{
	return WindEstimatorModule::main(argc, argv);
}
