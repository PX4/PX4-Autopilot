/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file ekf_att_pos_estimator_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Johan Jansen <jnsn.johan@gmail.com>
 */

#include "AttitudePositionEstimatorEKF.h"
#include "estimator_22states.h"

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <float.h>

#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <mavlink/mavlink_log.h>
#include <platforms/px4_defines.h>

static uint64_t IMUmsec = 0;
static uint64_t IMUusec = 0;

//Constants
static constexpr float rc = 10.0f;	// RC time constant of 1st order LPF in seconds
static constexpr uint64_t FILTER_INIT_DELAY = 1 * 1000 * 1000; 	///< units: microseconds
static constexpr float POS_RESET_THRESHOLD = 5.0f; 				///< Seconds before we signal a total GPS failure
static constexpr unsigned MAG_SWITCH_HYSTERESIS = 10;	///< Ignore the first few mag failures (which amounts to a few milliseconds)
static constexpr unsigned GYRO_SWITCH_HYSTERESIS = 5;	///< Ignore the first few gyro failures (which amounts to a few milliseconds)
static constexpr unsigned ACCEL_SWITCH_HYSTERESIS = 5;	///< Ignore the first few accel failures (which amounts to a few milliseconds)
static constexpr float EPH_LARGE_VALUE = 1000.0f;
static constexpr float EPV_LARGE_VALUE = 1000.0f;

/**
 * estimator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int ekf_att_pos_estimator_main(int argc, char *argv[]);

__EXPORT uint32_t millis();

__EXPORT uint64_t getMicros();

uint32_t millis()
{
	return IMUmsec;
}

uint64_t getMicros()
{
	return IMUusec;
}

namespace estimator
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

AttitudePositionEstimatorEKF	*g_estimator = nullptr;
}

AttitudePositionEstimatorEKF::AttitudePositionEstimatorEKF() :
	_task_should_exit(false),
	_task_running(false),
	_estimator_task(-1),

/* subscriptions */
	_sensor_combined_sub(-1),
	_distance_sub(-1),
	_airspeed_sub(-1),
	_baro_sub(-1),
	_gps_sub(-1),
	_vstatus_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),
	_mission_sub(-1),
	_home_sub(-1),
	_landDetectorSub(-1),
	_armedSub(-1),

/* publications */
	_att_pub(-1),
	_global_pos_pub(-1),
	_local_pos_pub(-1),
	_estimator_status_pub(-1),
	_wind_pub(-1),

	_att({}),
    _gyro({}),
    _accel({}),
    _mag({}),
    _airspeed({}),
    _baro({}),
    _vstatus({}),
    _global_pos({}),
    _local_pos({}),
    _gps({}),
    _wind({}),
    _distance {},
    _landDetector {},
    _armed {},

    _gyro_offsets({}),
    _accel_offsets({}),
    _mag_offsets({}),

    _sensor_combined {},

    _pos_ref{},
    _filter_ref_offset(0.0f),
    _baro_gps_offset(0.0f),

    /* performance counters */
    _loop_perf(perf_alloc(PC_ELAPSED, "ekf_att_pos_estimator")),
    _loop_intvl(perf_alloc(PC_INTERVAL, "ekf_att_pos_est_interval")),
    _perf_gyro(perf_alloc(PC_INTERVAL, "ekf_att_pos_gyro_upd")),
    _perf_mag(perf_alloc(PC_INTERVAL, "ekf_att_pos_mag_upd")),
    _perf_gps(perf_alloc(PC_INTERVAL, "ekf_att_pos_gps_upd")),
    _perf_baro(perf_alloc(PC_INTERVAL, "ekf_att_pos_baro_upd")),
    _perf_airspeed(perf_alloc(PC_INTERVAL, "ekf_att_pos_aspd_upd")),
    _perf_reset(perf_alloc(PC_COUNT, "ekf_att_pos_reset")),

    /* states */
    _gps_alt_filt(0.0f),
    _baro_alt_filt(0.0f),
    _covariancePredictionDt(0.0f),
    _gpsIsGood(false),
    _previousGPSTimestamp(0),
    _baro_init(false),
    _gps_initialized(false),
    _filter_start_time(0),
    _last_sensor_timestamp(0),
    _last_run(0),
    _distance_last_valid(0),
    _gyro_valid(false),
    _accel_valid(false),
    _mag_valid(false),
    _gyro_main(0),
    _accel_main(0),
    _mag_main(0),
    _ekf_logging(true),
    _debug(0),

    _newHgtData(false),
    _newAdsData(false),
    _newDataMag(false),
    _newRangeData(false),

    _mavlink_fd(-1),
    _parameters {},
    _parameter_handles {},
    _ekf(nullptr)
{
	_last_run = hrt_absolute_time();

	_parameter_handles.vel_delay_ms = param_find("PE_VEL_DELAY_MS");
	_parameter_handles.pos_delay_ms = param_find("PE_POS_DELAY_MS");
	_parameter_handles.height_delay_ms = param_find("PE_HGT_DELAY_MS");
	_parameter_handles.mag_delay_ms = param_find("PE_MAG_DELAY_MS");
	_parameter_handles.tas_delay_ms = param_find("PE_TAS_DELAY_MS");
	_parameter_handles.velne_noise = param_find("PE_VELNE_NOISE");
	_parameter_handles.veld_noise = param_find("PE_VELD_NOISE");
	_parameter_handles.posne_noise = param_find("PE_POSNE_NOISE");
	_parameter_handles.posd_noise = param_find("PE_POSD_NOISE");
	_parameter_handles.mag_noise = param_find("PE_MAG_NOISE");
	_parameter_handles.gyro_pnoise = param_find("PE_GYRO_PNOISE");
	_parameter_handles.acc_pnoise = param_find("PE_ACC_PNOISE");
	_parameter_handles.gbias_pnoise = param_find("PE_GBIAS_PNOISE");
	_parameter_handles.abias_pnoise = param_find("PE_ABIAS_PNOISE");
	_parameter_handles.mage_pnoise = param_find("PE_MAGE_PNOISE");
	_parameter_handles.magb_pnoise = param_find("PE_MAGB_PNOISE");
	_parameter_handles.eas_noise = param_find("PE_EAS_NOISE");
	_parameter_handles.pos_stddev_threshold = param_find("PE_POSDEV_INIT");

	/* indicate consumers that the current position data is not valid */
	_gps.eph = 10000.0f;
	_gps.epv = 10000.0f;

	/* fetch initial parameter values */
	parameters_update();

	/* get offsets */
	int fd, res;

	for (unsigned s = 0; s < 3; s++) {
		char str[30];
		(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);
		fd = open(str, O_RDONLY);

		if (fd >= 0) {
			res = ioctl(fd, GYROIOCGSCALE, (long unsigned int)&_gyro_offsets[s]);
			close(fd);

			if (res) {
				warnx("G%u SCALE FAIL", s);
			}
		}

		(void)sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, s);
		fd = open(str, O_RDONLY);

		if (fd >= 0) {
			res = ioctl(fd, ACCELIOCGSCALE, (long unsigned int)&_accel_offsets[s]);
			close(fd);

			if (res) {
				warnx("A%u SCALE FAIL", s);
			}
		}

		(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, s);
		fd = open(str, O_RDONLY);

		if (fd >= 0) {
			res = ioctl(fd, MAGIOCGSCALE, (long unsigned int)&_mag_offsets[s]);
			close(fd);

			if (res) {
				warnx("M%u SCALE FAIL", s);
			}
		}
	}
}

AttitudePositionEstimatorEKF::~AttitudePositionEstimatorEKF()
{
	if (_estimator_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_estimator_task);
				break;
			}
		} while (_estimator_task != -1);
	}

	delete _ekf;

	estimator::g_estimator = nullptr;
}

int AttitudePositionEstimatorEKF::enable_logging(bool logging)
{
	_ekf_logging = logging;

	return 0;
}

int AttitudePositionEstimatorEKF::parameters_update()
{

	param_get(_parameter_handles.vel_delay_ms, &(_parameters.vel_delay_ms));
	param_get(_parameter_handles.pos_delay_ms, &(_parameters.pos_delay_ms));
	param_get(_parameter_handles.height_delay_ms, &(_parameters.height_delay_ms));
	param_get(_parameter_handles.mag_delay_ms, &(_parameters.mag_delay_ms));
	param_get(_parameter_handles.tas_delay_ms, &(_parameters.tas_delay_ms));
	param_get(_parameter_handles.velne_noise, &(_parameters.velne_noise));
	param_get(_parameter_handles.veld_noise, &(_parameters.veld_noise));
	param_get(_parameter_handles.posne_noise, &(_parameters.posne_noise));
	param_get(_parameter_handles.posd_noise, &(_parameters.posd_noise));
	param_get(_parameter_handles.mag_noise, &(_parameters.mag_noise));
	param_get(_parameter_handles.gyro_pnoise, &(_parameters.gyro_pnoise));
	param_get(_parameter_handles.acc_pnoise, &(_parameters.acc_pnoise));
	param_get(_parameter_handles.gbias_pnoise, &(_parameters.gbias_pnoise));
	param_get(_parameter_handles.abias_pnoise, &(_parameters.abias_pnoise));
	param_get(_parameter_handles.mage_pnoise, &(_parameters.mage_pnoise));
	param_get(_parameter_handles.magb_pnoise, &(_parameters.magb_pnoise));
	param_get(_parameter_handles.eas_noise, &(_parameters.eas_noise));
	param_get(_parameter_handles.pos_stddev_threshold, &(_parameters.pos_stddev_threshold));

	if (_ekf) {
		// _ekf->yawVarScale = 1.0f;
		// _ekf->windVelSigma = 0.1f;
		_ekf->dAngBiasSigma = _parameters.gbias_pnoise;
		_ekf->dVelBiasSigma = _parameters.abias_pnoise;
		_ekf->magEarthSigma = _parameters.mage_pnoise;
		_ekf->magBodySigma  = _parameters.magb_pnoise;
		// _ekf->gndHgtSigma  = 0.02f;
		_ekf->vneSigma = _parameters.velne_noise;
		_ekf->vdSigma = _parameters.veld_noise;
		_ekf->posNeSigma = _parameters.posne_noise;
		_ekf->posDSigma = _parameters.posd_noise;
		_ekf->magMeasurementSigma = _parameters.mag_noise;
		_ekf->gyroProcessNoise = _parameters.gyro_pnoise;
		_ekf->accelProcessNoise = _parameters.acc_pnoise;
		_ekf->airspeedMeasurementSigma = _parameters.eas_noise;
		_ekf->rngFinderPitch = 0.0f; // XXX base on SENS_BOARD_Y_OFF
	}

	return OK;
}

void AttitudePositionEstimatorEKF::vehicle_status_poll()
{
	bool vstatus_updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_vstatus_sub, &vstatus_updated);

	if (vstatus_updated) {

		orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus);

		//Tell EKF that the vehicle is a fixed wing or multi-rotor
		_ekf->setIsFixedWing(!_vstatus.is_rotary_wing);
	}
}

int AttitudePositionEstimatorEKF::check_filter_state()
{
	/*
	 *    CHECK IF THE INPUT DATA IS SANE
	 */

	struct ekf_status_report ekf_report;

	int check = _ekf->CheckAndBound(&ekf_report);

	const char *const feedback[] = { 0,
					 "NaN in states, resetting",
					 "stale sensor data, resetting",
					 "got initial position lock",
					 "excessive gyro offsets",
					 "velocity diverted, check accel config",
					 "excessive covariances",
					 "unknown condition, resetting"
				       };

	// Print out error condition
	if (check) {
		unsigned warn_index = static_cast<unsigned>(check);
		unsigned max_warn_index = (sizeof(feedback) / sizeof(feedback[0]));

		if (max_warn_index < warn_index) {
			warn_index = max_warn_index;
		}

		// Do not warn about accel offset if we have no position updates
		if (!(warn_index == 5 && _ekf->staticMode)) {
			warnx("reset: %s", feedback[warn_index]);
			mavlink_log_critical(_mavlink_fd, "[ekf check] %s", feedback[warn_index]);
		}
	}

	struct estimator_status_s rep;

	memset(&rep, 0, sizeof(rep));

	// If error flag is set, we got a filter reset
	if (check && ekf_report.error) {

		// Count the reset condition
		perf_count(_perf_reset);
		// GPS is in scaled integers, convert
		double lat = _gps.lat / 1.0e7;
		double lon = _gps.lon / 1.0e7;
		float gps_alt = _gps.alt / 1e3f;

		// Set up height correctly
		orb_copy(ORB_ID(sensor_baro), _baro_sub, &_baro);

		initReferencePosition(_gps.timestamp_position, _gpsIsGood, lat, lon, gps_alt, _baro.altitude);

	} else if (_ekf_logging) {
		_ekf->GetFilterState(&ekf_report);
	}

	if (_ekf_logging || check) {
		rep.timestamp = hrt_absolute_time();

		rep.nan_flags |= (((uint8_t)ekf_report.angNaN)		<< 0);
		rep.nan_flags |= (((uint8_t)ekf_report.summedDelVelNaN)	<< 1);
		rep.nan_flags |= (((uint8_t)ekf_report.KHNaN)		<< 2);
		rep.nan_flags |= (((uint8_t)ekf_report.KHPNaN)		<< 3);
		rep.nan_flags |= (((uint8_t)ekf_report.PNaN)		<< 4);
		rep.nan_flags |= (((uint8_t)ekf_report.covarianceNaN)	<< 5);
		rep.nan_flags |= (((uint8_t)ekf_report.kalmanGainsNaN)	<< 6);
		rep.nan_flags |= (((uint8_t)ekf_report.statesNaN)	<< 7);

		rep.health_flags |= (((uint8_t)ekf_report.velHealth)	<< 0);
		rep.health_flags |= (((uint8_t)ekf_report.posHealth)	<< 1);
		rep.health_flags |= (((uint8_t)ekf_report.hgtHealth)	<< 2);
		rep.health_flags |= (((uint8_t)!ekf_report.gyroOffsetsExcessive)	<< 3);
		rep.health_flags |= (((uint8_t)ekf_report.onGround)	<< 4);
		rep.health_flags |= (((uint8_t)ekf_report.staticMode)	<< 5);
		rep.health_flags |= (((uint8_t)ekf_report.useCompass)	<< 6);
		rep.health_flags |= (((uint8_t)ekf_report.useAirspeed)	<< 7);

		rep.timeout_flags |= (((uint8_t)ekf_report.velTimeout)	<< 0);
		rep.timeout_flags |= (((uint8_t)ekf_report.posTimeout)	<< 1);
		rep.timeout_flags |= (((uint8_t)ekf_report.hgtTimeout)	<< 2);
		rep.timeout_flags |= (((uint8_t)ekf_report.imuTimeout)	<< 3);

		if (_debug > 10) {

			if (rep.health_flags < ((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3))) {
				warnx("health: VEL:%s POS:%s HGT:%s OFFS:%s",
				      ((rep.health_flags & (1 << 0)) ? "OK" : "ERR"),
				      ((rep.health_flags & (1 << 1)) ? "OK" : "ERR"),
				      ((rep.health_flags & (1 << 2)) ? "OK" : "ERR"),
				      ((rep.health_flags & (1 << 3)) ? "OK" : "ERR"));
			}

			if (rep.timeout_flags) {
				warnx("timeout: %s%s%s%s",
				      ((rep.timeout_flags & (1 << 0)) ? "VEL " : ""),
				      ((rep.timeout_flags & (1 << 1)) ? "POS " : ""),
				      ((rep.timeout_flags & (1 << 2)) ? "HGT " : ""),
				      ((rep.timeout_flags & (1 << 3)) ? "IMU " : ""));
			}
		}

		// Copy all states or at least all that we can fit
		size_t ekf_n_states = ekf_report.n_states;
		size_t max_states = (sizeof(rep.states) / sizeof(rep.states[0]));
		rep.n_states = (ekf_n_states < max_states) ? ekf_n_states : max_states;

		for (size_t i = 0; i < rep.n_states; i++) {
			rep.states[i] = ekf_report.states[i];
		}



		if (_estimator_status_pub > 0) {
			orb_publish(ORB_ID(estimator_status), _estimator_status_pub, &rep);

		} else {
			_estimator_status_pub = orb_advertise(ORB_ID(estimator_status), &rep);
		}
	}

	return check;
}

void AttitudePositionEstimatorEKF::task_main_trampoline(int argc, char *argv[])
{
	estimator::g_estimator->task_main();
}

void AttitudePositionEstimatorEKF::task_main()
{
	_mavlink_fd = open(MAVLINK_LOG_DEVICE, 0);

	_ekf = new AttPosEKF();

	_filter_start_time = hrt_absolute_time();

	if (!_ekf) {
		errx(1, "OUT OF MEM!");
	}

	/*
	 * do subscriptions
	 */
	_distance_sub = orb_subscribe(ORB_ID(sensor_range_finder));
	_baro_sub = orb_subscribe_multi(ORB_ID(sensor_baro), 0);
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_home_sub = orb_subscribe(ORB_ID(home_position));
	_landDetectorSub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_armedSub = orb_subscribe(ORB_ID(actuator_armed));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vstatus_sub, 200);

	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	/* XXX remove this!, BUT increase the data buffer size! */
	orb_set_interval(_sensor_combined_sub, 9);

	/* sets also parameters in the EKF object */
	parameters_update();

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;

	fds[1].fd = _sensor_combined_sub;
	fds[1].events = POLLIN;

	_gps.vel_n_m_s = 0.0f;
	_gps.vel_e_m_s = 0.0f;
	_gps.vel_d_m_s = 0.0f;

	_task_running = true;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("POLL ERR %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);
		perf_count(_loop_intvl);

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run estimator if gyro updated */
		if (fds[1].revents & POLLIN) {

			/* check vehicle status for changes to publication state */
			bool prev_hil = (_vstatus.hil_state == vehicle_status_s::HIL_STATE_ON);
			vehicle_status_poll();

			perf_count(_perf_gyro);

			/* Reset baro reference if switching to HIL, reset sensor states */
			if (!prev_hil && (_vstatus.hil_state == vehicle_status_s::HIL_STATE_ON)) {
				/* system is in HIL now, wait for measurements to come in one last round */
				usleep(60000);

				/* now read all sensor publications to ensure all real sensor data is purged */
				orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);

				/* set sensors to de-initialized state */
				_gyro_valid = false;
				_accel_valid = false;
				_mag_valid = false;

				_baro_init = false;
				_gps_initialized = false;

				_last_sensor_timestamp = hrt_absolute_time();
				_last_run = _last_sensor_timestamp;

				_ekf->ZeroVariables();
				_ekf->dtIMU = 0.01f;
				_filter_start_time = _last_sensor_timestamp;

				/* now skip this loop and get data on the next one, which will also re-init the filter */
				continue;
			}

			/**
			 *    PART ONE: COLLECT ALL DATA
			 **/
			pollData();

			/*
			 *    CHECK IF ITS THE RIGHT TIME TO RUN THINGS ALREADY
			 */
			if (hrt_elapsed_time(&_filter_start_time) < FILTER_INIT_DELAY) {
				continue;
			}

			/**
			 *    PART TWO: EXECUTE THE FILTER
			 *
			 *    We run the filter only once all data has been fetched
			 **/

			if (_baro_init && _gyro_valid && _accel_valid && _mag_valid) {

				// maintain filtered baro and gps altitudes to calculate weather offset
				// baro sample rate is ~70Hz and measurement bandwidth is high
				// gps sample rate is 5Hz and altitude is assumed accurate when averaged over 30 seconds
				// maintain heavily filtered values for both baro and gps altitude
				// Assume the filtered output should be identical for both sensors

				if (_gpsIsGood) {
					_baro_gps_offset = _baro_alt_filt - _gps_alt_filt;
				}
//				if (hrt_elapsed_time(&_last_debug_print) >= 5e6) {
//					_last_debug_print = hrt_absolute_time();
//					perf_print_counter(_perf_baro);
//					perf_reset(_perf_baro);
//					warnx("gpsoff: %5.1f, baro_alt_filt: %6.1f, gps_alt_filt: %6.1f, gpos.alt: %5.1f, lpos.z: %6.1f",
//							(double)_baro_gps_offset,
//							(double)_baro_alt_filt,
//							(double)_gps_alt_filt,
//							(double)_global_pos.alt,
//							(double)_local_pos.z);
//				}

				/* Initialize the filter first */
				if (!_ekf->statesInitialised) {
					// North, East Down position (m)
					float initVelNED[3] = {0.0f, 0.0f, 0.0f};

					_ekf->posNE[0] = 0.0f;
					_ekf->posNE[1] = 0.0f;

					_local_pos.ref_alt = 0.0f;
					_baro_gps_offset = 0.0f;
					_baro_alt_filt = _baro.altitude;

					_ekf->InitialiseFilter(initVelNED, 0.0, 0.0, 0.0f, 0.0f);

					_filter_ref_offset = -_baro.altitude;

					warnx("filter ref off: baro_alt: %8.4f", (double)_filter_ref_offset);

				} else {

					if (!_gps_initialized && _gpsIsGood) {
						initializeGPS();
						continue;
					}

					// Check if on ground - status is used by covariance prediction
					_ekf->setOnGround(_landDetector.landed);

					// We're apparently initialized in this case now
					// check (and reset the filter as needed)
					int check = check_filter_state();

					if (check) {
						// Let the system re-initialize itself
						continue;
					}

					// Run EKF data fusion steps
					updateSensorFusion(_gpsIsGood, _newDataMag, _newRangeData, _newHgtData, _newAdsData);

					// Publish attitude estimations
					publishAttitude();

					// Publish Local Position estimations
					publishLocalPosition();

					// Publish Global Position, it will have a large uncertainty
					// set if only altitude is known
					publishGlobalPosition();

					// Publish wind estimates
					if (hrt_elapsed_time(&_wind.timestamp) > 99000) {
						publishWindEstimate();
					}
				}
			}

		}

		perf_end(_loop_perf);
	}

	_task_running = false;

	warnx("exiting.\n");

	_estimator_task = -1;
	_exit(0);
}

void AttitudePositionEstimatorEKF::initReferencePosition(hrt_abstime timestamp,
	bool gps_valid, double lat, double lon, float gps_alt, float baro_alt)
{
	// Reference altitude
	if (isfinite(_ekf->states[9])) {
		_filter_ref_offset = _ekf->states[9];
	} else if (isfinite(-_ekf->hgtMea)) {
		_filter_ref_offset = -_ekf->hgtMea;
	} else {
		_filter_ref_offset = -_baro.altitude;
	}

	// init filtered gps and baro altitudes
	_baro_alt_filt = baro_alt;

	if (gps_valid) {
		_gps_alt_filt = gps_alt;

		// Initialize projection
		_local_pos.ref_lat = lat;
		_local_pos.ref_lon = lon;
		_local_pos.ref_alt = gps_alt;
		_local_pos.ref_timestamp = timestamp;

		map_projection_init(&_pos_ref, lat, lon);
		mavlink_and_console_log_info(_mavlink_fd, "[ekf] ref: LA %.4f,LO %.4f,ALT %.2f", lat, lon, (double)gps_alt);
	}
}

void AttitudePositionEstimatorEKF::initializeGPS()
{
	// GPS is in scaled integers, convert
	double lat = _gps.lat / 1.0e7;
	double lon = _gps.lon / 1.0e7;
	float gps_alt = _gps.alt / 1e3f;

	// Set up height correctly
	orb_copy(ORB_ID(sensor_baro), _baro_sub, &_baro);

	_ekf->baroHgt = _baro.altitude;
	_ekf->hgtMea = _ekf->baroHgt;

	// Set up position variables correctly
	_ekf->GPSstatus = _gps.fix_type;

	_ekf->gpsLat = math::radians(lat);
	_ekf->gpsLon = math::radians(lon) - M_PI;
	_ekf->gpsHgt = gps_alt;

	// Look up mag declination based on current position
	float declination = math::radians(get_mag_declination(lat, lon));

	float initVelNED[3];
	initVelNED[0] = _gps.vel_n_m_s;
	initVelNED[1] = _gps.vel_e_m_s;
	initVelNED[2] = _gps.vel_d_m_s;

	_ekf->InitialiseFilter(initVelNED, math::radians(lat), math::radians(lon) - M_PI, gps_alt, declination);

	initReferencePosition(_gps.timestamp_position, _gpsIsGood, lat, lon, gps_alt, _baro.altitude);

#if 0
	warnx("HOME/REF: LA %8.4f,LO %8.4f,ALT %8.2f V: %8.4f %8.4f %8.4f", lat, lon, (double)gps_alt,
	      (double)_ekf->velNED[0], (double)_ekf->velNED[1], (double)_ekf->velNED[2]);
	warnx("BARO: %8.4f m / ref: %8.4f m / gps offs: %8.4f m", (double)_ekf->baroHgt, (double)_baro_ref,
	      (double)_filter_ref_offset);
	warnx("GPS: eph: %8.4f, epv: %8.4f, declination: %8.4f", (double)_gps.eph, (double)_gps.epv,
	      (double)math::degrees(declination));
#endif

	_gps_initialized = true;
}

void AttitudePositionEstimatorEKF::publishAttitude()
{
	// Output results
	math::Quaternion q(_ekf->states[0], _ekf->states[1], _ekf->states[2], _ekf->states[3]);
	math::Matrix<3, 3> R = q.to_dcm();
	math::Vector<3> euler = R.to_euler();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			PX4_R(_att.R, i, j) = R(i, j);
		}
	}

	_att.timestamp = _last_sensor_timestamp;
	_att.q[0] = _ekf->states[0];
	_att.q[1] = _ekf->states[1];
	_att.q[2] = _ekf->states[2];
	_att.q[3] = _ekf->states[3];
	_att.q_valid = true;
	_att.R_valid = true;

	_att.timestamp = _last_sensor_timestamp;
	_att.roll = euler(0);
	_att.pitch = euler(1);
	_att.yaw = euler(2);

	_att.rollspeed = _ekf->angRate.x - _ekf->states[10] / _ekf->dtIMUfilt;
	_att.pitchspeed = _ekf->angRate.y - _ekf->states[11] / _ekf->dtIMUfilt;
	_att.yawspeed = _ekf->angRate.z - _ekf->states[12] / _ekf->dtIMUfilt;

	// gyro offsets
	_att.rate_offsets[0] = _ekf->states[10] / _ekf->dtIMUfilt;
	_att.rate_offsets[1] = _ekf->states[11] / _ekf->dtIMUfilt;
	_att.rate_offsets[2] = _ekf->states[12] / _ekf->dtIMUfilt;

	/* lazily publish the attitude only once available */
	if (_att_pub > 0) {
		/* publish the attitude setpoint */
		orb_publish(ORB_ID(vehicle_attitude), _att_pub, &_att);

	} else {
		/* advertise and publish */
		_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &_att);
	}
}

void AttitudePositionEstimatorEKF::publishLocalPosition()
{
	_local_pos.timestamp = _last_sensor_timestamp;
	_local_pos.x = _ekf->states[7];
	_local_pos.y = _ekf->states[8];

	// XXX need to announce change of Z reference somehow elegantly
	_local_pos.z = _ekf->states[9] - _filter_ref_offset;
	//_local_pos.z_stable = _ekf->states[9];

	_local_pos.vx = _ekf->states[4];
	_local_pos.vy = _ekf->states[5];
	_local_pos.vz = _ekf->states[6];

	_local_pos.xy_valid = _gps_initialized && _gpsIsGood;
	_local_pos.z_valid = true;
	_local_pos.v_xy_valid = _gps_initialized && _gpsIsGood;
	_local_pos.v_z_valid = true;
	_local_pos.xy_global = _gps_initialized; //TODO: Handle optical flow mode here

	_local_pos.z_global = false;
	_local_pos.yaw = _att.yaw;

	if (!isfinite(_local_pos.x) ||
		!isfinite(_local_pos.y) ||
		!isfinite(_local_pos.z) ||
		!isfinite(_local_pos.vx) ||
		!isfinite(_local_pos.vy) ||
		!isfinite(_local_pos.vz))
	{
		// bad data, abort publication
		return;
	}

	/* lazily publish the local position only once available */
	if (_local_pos_pub > 0) {
		/* publish the attitude setpoint */
		orb_publish(ORB_ID(vehicle_local_position), _local_pos_pub, &_local_pos);

	} else {
		/* advertise and publish */
		_local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &_local_pos);
	}
}

void AttitudePositionEstimatorEKF::publishGlobalPosition()
{
	_global_pos.timestamp = _local_pos.timestamp;

	if (_local_pos.xy_global) {
		double est_lat, est_lon;
		map_projection_reproject(&_pos_ref, _local_pos.x, _local_pos.y, &est_lat, &est_lon);
		_global_pos.lat = est_lat;
		_global_pos.lon = est_lon;
		_global_pos.time_utc_usec = _gps.time_utc_usec;
	} else {
		_global_pos.lat = 0.0;
		_global_pos.lon = 0.0;
		_global_pos.time_utc_usec = 0;
	}

	if (_local_pos.v_xy_valid) {
		_global_pos.vel_n = _local_pos.vx;
		_global_pos.vel_e = _local_pos.vy;

	} else {
		_global_pos.vel_n = 0.0f;
		_global_pos.vel_e = 0.0f;
	}

	/* local pos alt is negative, change sign and add alt offsets */
	_global_pos.alt = (-_local_pos.z) - _filter_ref_offset -  _baro_gps_offset;

	if (_local_pos.v_z_valid) {
		_global_pos.vel_d = _local_pos.vz;
	} else {
		_global_pos.vel_d = 0.0f;
	}

	/* terrain altitude */
	_global_pos.terrain_alt = _ekf->hgtRef - _ekf->flowStates[1];
	_global_pos.terrain_alt_valid = (_distance_last_valid > 0) &&
					(hrt_elapsed_time(&_distance_last_valid) < 20 * 1000 * 1000);

	_global_pos.yaw = _local_pos.yaw;

	const float dtLastGoodGPS = static_cast<float>(hrt_absolute_time() - _previousGPSTimestamp) / 1e6f;

	if (!_local_pos.xy_global ||
	    !_local_pos.v_xy_valid ||
		_gps.timestamp_position == 0 ||
		(dtLastGoodGPS >= POS_RESET_THRESHOLD)) {

		_global_pos.eph = EPH_LARGE_VALUE;
		_global_pos.epv = EPV_LARGE_VALUE;

	} else {
		_global_pos.eph = _gps.eph;
		_global_pos.epv = _gps.epv;
	}

	if (!isfinite(_global_pos.lat) ||
		!isfinite(_global_pos.lon) ||
		!isfinite(_global_pos.alt) ||
		!isfinite(_global_pos.vel_n) ||
		!isfinite(_global_pos.vel_e) ||
		!isfinite(_global_pos.vel_d))
	{
		// bad data, abort publication
		return;
	}

	/* lazily publish the global position only once available */
	if (_global_pos_pub > 0) {
		/* publish the global position */
		orb_publish(ORB_ID(vehicle_global_position), _global_pos_pub, &_global_pos);

	} else {
		/* advertise and publish */
		_global_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &_global_pos);
	}
}

void AttitudePositionEstimatorEKF::publishWindEstimate()
{
	_wind.timestamp = _global_pos.timestamp;
	_wind.windspeed_north = _ekf->states[14];
	_wind.windspeed_east = _ekf->states[15];

	// XXX we need to do something smart about the covariance here
	// but we default to the estimate covariance for now
	_wind.covariance_north = _ekf->P[14][14];
	_wind.covariance_east = _ekf->P[15][15];

	/* lazily publish the wind estimate only once available */
	if (_wind_pub > 0) {
		/* publish the wind estimate */
		orb_publish(ORB_ID(wind_estimate), _wind_pub, &_wind);

	} else {
		/* advertise and publish */
		_wind_pub = orb_advertise(ORB_ID(wind_estimate), &_wind);
	}

}

void AttitudePositionEstimatorEKF::updateSensorFusion(const bool fuseGPS, const bool fuseMag,
		const bool fuseRangeSensor, const bool fuseBaro, const bool fuseAirSpeed)
{
	// Run the strapdown INS equations every IMU update
	_ekf->UpdateStrapdownEquationsNED();

	// store the predicted states for subsequent use by measurement fusion
	_ekf->StoreStates(IMUmsec);

	// sum delta angles and time used by covariance prediction
	_ekf->summedDelAng = _ekf->summedDelAng + _ekf->correctedDelAng;
	_ekf->summedDelVel = _ekf->summedDelVel + _ekf->dVelIMU;
	_covariancePredictionDt += _ekf->dtIMU;

	// perform a covariance prediction if the total delta angle has exceeded the limit
	// or the time limit will be exceeded at the next IMU update
	if ((_covariancePredictionDt >= (_ekf->covTimeStepMax - _ekf->dtIMU))
	    || (_ekf->summedDelAng.length() > _ekf->covDelAngMax)) {
		_ekf->CovariancePrediction(_covariancePredictionDt);
		_ekf->summedDelAng.zero();
		_ekf->summedDelVel.zero();
		_covariancePredictionDt = 0.0f;
	}

	// Fuse GPS Measurements
	if (fuseGPS && _gps_initialized) {
		// Convert GPS measurements to Pos NE, hgt and Vel NED

		// set fusion flags
		_ekf->fuseVelData = _gps.vel_ned_valid;
		_ekf->fusePosData = true;

		// recall states stored at time of measurement after adjusting for delays
		_ekf->RecallStates(_ekf->statesAtVelTime, (IMUmsec - _parameters.vel_delay_ms));
		_ekf->RecallStates(_ekf->statesAtPosTime, (IMUmsec - _parameters.pos_delay_ms));

		// run the fusion step
		_ekf->FuseVelposNED();

	} else if (!_gps_initialized) {

		// force static mode
		_ekf->staticMode = true;

		// Convert GPS measurements to Pos NE, hgt and Vel NED
		_ekf->velNED[0] = 0.0f;
		_ekf->velNED[1] = 0.0f;
		_ekf->velNED[2] = 0.0f;

		_ekf->posNE[0] = 0.0f;
		_ekf->posNE[1] = 0.0f;

		// set fusion flags
		_ekf->fuseVelData = true;
		_ekf->fusePosData = true;

		// recall states stored at time of measurement after adjusting for delays
		_ekf->RecallStates(_ekf->statesAtVelTime, (IMUmsec - _parameters.vel_delay_ms));
		_ekf->RecallStates(_ekf->statesAtPosTime, (IMUmsec - _parameters.pos_delay_ms));

		// run the fusion step
		_ekf->FuseVelposNED();

	} else {
		_ekf->fuseVelData = false;
		_ekf->fusePosData = false;
	}

	if (fuseBaro) {
		// Could use a blend of GPS and baro alt data if desired
		_ekf->hgtMea = _ekf->baroHgt;
		_ekf->fuseHgtData = true;

		// recall states stored at time of measurement after adjusting for delays
		_ekf->RecallStates(_ekf->statesAtHgtTime, (IMUmsec - _parameters.height_delay_ms));

		// run the fusion step
		_ekf->FuseVelposNED();

	} else {
		_ekf->fuseHgtData = false;
	}

	// Fuse Magnetometer Measurements
	if (fuseMag) {
		_ekf->fuseMagData = true;
		_ekf->RecallStates(_ekf->statesAtMagMeasTime,
				   (IMUmsec - _parameters.mag_delay_ms)); // Assume 50 msec avg delay for magnetometer data

		_ekf->magstate.obsIndex = 0;
		_ekf->FuseMagnetometer();
		_ekf->FuseMagnetometer();
		_ekf->FuseMagnetometer();

	} else {
		_ekf->fuseMagData = false;
	}

	// Fuse Airspeed Measurements
	if (fuseAirSpeed && _airspeed.true_airspeed_m_s > 5.0f) {
		_ekf->fuseVtasData = true;
		_ekf->RecallStates(_ekf->statesAtVtasMeasTime,
				   (IMUmsec - _parameters.tas_delay_ms)); // assume 100 msec avg delay for airspeed data
		_ekf->FuseAirspeed();

	} else {
		_ekf->fuseVtasData = false;
	}

	// Fuse Rangefinder Measurements
	if (fuseRangeSensor) {
		if (_ekf->Tnb.z.z > 0.9f) {
			// _ekf->rngMea is set in sensor readout already
			_ekf->fuseRngData = true;
			_ekf->fuseOptFlowData = false;
			_ekf->RecallStates(_ekf->statesAtRngTime, (IMUmsec - 100.0f));
			_ekf->OpticalFlowEKF();
			_ekf->fuseRngData = false;
		}
	}
}

int AttitudePositionEstimatorEKF::start()
{
	ASSERT(_estimator_task == -1);

	/* start the task */
	_estimator_task = task_spawn_cmd("ekf_att_pos_estimator",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 40,
					 4800,
					 (main_t)&AttitudePositionEstimatorEKF::task_main_trampoline,
					 nullptr);

	if (_estimator_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void AttitudePositionEstimatorEKF::print_status()
{
	math::Quaternion q(_ekf->states[0], _ekf->states[1], _ekf->states[2], _ekf->states[3]);
	math::Matrix<3, 3> R = q.to_dcm();
	math::Vector<3> euler = R.to_euler();

	printf("attitude: roll: %8.4f, pitch %8.4f, yaw: %8.4f degrees\n",
	       (double)math::degrees(euler(0)), (double)math::degrees(euler(1)), (double)math::degrees(euler(2)));

	// State vector:
	// 0-3: quaternions (q0, q1, q2, q3)
	// 4-6: Velocity - m/sec (North, East, Down)
	// 7-9: Position - m (North, East, Down)
	// 10-12: Delta Angle bias - rad (X,Y,Z)
	// 13:    Delta Velocity Bias - m/s (Z)
	// 14-15: Wind Vector  - m/sec (North,East)
	// 16-18: Earth Magnetic Field Vector - gauss (North, East, Down)
	// 19-21: Body Magnetic Field Vector - gauss (X,Y,Z)

	printf("dtIMU: %8.6f filt: %8.6f IMUmsec: %d\n", (double)_ekf->dtIMU, (double)_ekf->dtIMUfilt, (int)IMUmsec);
	printf("alt RAW: baro alt: %8.4f GPS alt: %8.4f\n", (double)_baro.altitude, (double)_ekf->gpsHgt);
	printf("alt EST: local alt: %8.4f (NED), AMSL alt: %8.4f (ENU)\n", (double)(_local_pos.z), (double)_global_pos.alt);
	printf("filter ref offset: %8.4f baro GPS offset: %8.4f\n", (double)_filter_ref_offset,
	       (double)_baro_gps_offset);
	printf("dvel: %8.6f %8.6f %8.6f accel: %8.6f %8.6f %8.6f\n", (double)_ekf->dVelIMU.x, (double)_ekf->dVelIMU.y,
	       (double)_ekf->dVelIMU.z, (double)_ekf->accel.x, (double)_ekf->accel.y, (double)_ekf->accel.z);
	printf("dang: %8.4f %8.4f %8.4f dang corr: %8.4f %8.4f %8.4f\n" , (double)_ekf->dAngIMU.x, (double)_ekf->dAngIMU.y,
	       (double)_ekf->dAngIMU.z, (double)_ekf->correctedDelAng.x, (double)_ekf->correctedDelAng.y,
	       (double)_ekf->correctedDelAng.z);
	printf("states (quat)        [0-3]: %8.4f, %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[0], (double)_ekf->states[1],
	       (double)_ekf->states[2], (double)_ekf->states[3]);
	printf("states (vel m/s)     [4-6]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[4], (double)_ekf->states[5],
	       (double)_ekf->states[6]);
	printf("states (pos m)      [7-9]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[7], (double)_ekf->states[8],
	       (double)_ekf->states[9]);
	printf("states (delta ang) [10-12]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[10], (double)_ekf->states[11],
	       (double)_ekf->states[12]);

	if (EKF_STATE_ESTIMATES == 23) {
		printf("states (accel offs)   [13]: %8.4f\n", (double)_ekf->states[13]);
		printf("states (wind)      [14-15]: %8.4f, %8.4f\n", (double)_ekf->states[14], (double)_ekf->states[15]);
		printf("states (earth mag) [16-18]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[16], (double)_ekf->states[17],
		       (double)_ekf->states[18]);
		printf("states (body mag)  [19-21]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[19], (double)_ekf->states[20],
		       (double)_ekf->states[21]);
		printf("states (terrain)      [22]: %8.4f\n", (double)_ekf->states[22]);

	} else {
		printf("states (wind)      [13-14]: %8.4f, %8.4f\n", (double)_ekf->states[13], (double)_ekf->states[14]);
		printf("states (earth mag) [15-17]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[15], (double)_ekf->states[16],
		       (double)_ekf->states[17]);
		printf("states (body mag)  [18-20]: %8.4f, %8.4f, %8.4f\n", (double)_ekf->states[18], (double)_ekf->states[19],
		       (double)_ekf->states[20]);
	}

	printf("states: %s %s %s %s %s %s %s %s %s %s\n",
	       (_ekf->statesInitialised) ? "INITIALIZED" : "NON_INIT",
	       (_landDetector.landed) ? "ON_GROUND" : "AIRBORNE",
	       (_ekf->fuseVelData) ? "FUSE_VEL" : "INH_VEL",
	       (_ekf->fusePosData) ? "FUSE_POS" : "INH_POS",
	       (_ekf->fuseHgtData) ? "FUSE_HGT" : "INH_HGT",
	       (_ekf->fuseMagData) ? "FUSE_MAG" : "INH_MAG",
	       (_ekf->fuseVtasData) ? "FUSE_VTAS" : "INH_VTAS",
	       (_ekf->useAirspeed) ? "USE_AIRSPD" : "IGN_AIRSPD",
	       (_ekf->useCompass) ? "USE_COMPASS" : "IGN_COMPASS",
	       (_ekf->staticMode) ? "STATIC_MODE" : "DYNAMIC_MODE");
}

void AttitudePositionEstimatorEKF::pollData()
{
	//Update arming status
	bool armedUpdate;
	orb_check(_armedSub, &armedUpdate);

	if (armedUpdate) {
		orb_copy(ORB_ID(actuator_armed), _armedSub, &_armed);
	}

	//Update Gyro and Accelerometer
	static Vector3f lastAngRate;
	static Vector3f lastAccel;
	bool accel_updated = false;

	orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);

	static hrt_abstime last_accel = 0;
	static hrt_abstime last_mag = 0;

	if (last_accel != _sensor_combined.accelerometer_timestamp) {
		accel_updated = true;

	} else {
		accel_updated = false;
	}

	last_accel = _sensor_combined.accelerometer_timestamp;


	// Copy gyro and accel
	_last_sensor_timestamp = _sensor_combined.timestamp;
	IMUmsec = _sensor_combined.timestamp / 1e3;
	IMUusec = _sensor_combined.timestamp;

	float deltaT = (_sensor_combined.timestamp - _last_run) / 1e6f;

	/* guard against too large deltaT's */
	if (!isfinite(deltaT) || deltaT > 1.0f || deltaT < 0.000001f) {
		deltaT = 0.01f;
	}

	_last_run = _sensor_combined.timestamp;

	// Always store data, independent of init status
	/* fill in last data set */
	_ekf->dtIMU = deltaT;

	int last_gyro_main = _gyro_main;

	if (isfinite(_sensor_combined.gyro_rad_s[0]) &&
	    isfinite(_sensor_combined.gyro_rad_s[1]) &&
	    isfinite(_sensor_combined.gyro_rad_s[2]) &&
	    (_sensor_combined.gyro_errcount <= (_sensor_combined.gyro1_errcount + GYRO_SWITCH_HYSTERESIS))) {

		_ekf->angRate.x = _sensor_combined.gyro_rad_s[0];
		_ekf->angRate.y = _sensor_combined.gyro_rad_s[1];
		_ekf->angRate.z = _sensor_combined.gyro_rad_s[2];
		_gyro_main = 0;
		_gyro_valid = true;

	} else if (isfinite(_sensor_combined.gyro1_rad_s[0]) &&
		   isfinite(_sensor_combined.gyro1_rad_s[1]) &&
		   isfinite(_sensor_combined.gyro1_rad_s[2])) {

		_ekf->angRate.x = _sensor_combined.gyro1_rad_s[0];
		_ekf->angRate.y = _sensor_combined.gyro1_rad_s[1];
		_ekf->angRate.z = _sensor_combined.gyro1_rad_s[2];
		_gyro_main = 1;
		_gyro_valid = true;

	} else {
		_gyro_valid = false;
	}

	if (last_gyro_main != _gyro_main) {
		mavlink_and_console_log_emergency(_mavlink_fd, "GYRO FAILED! Switched from #%d to %d", last_gyro_main, _gyro_main);
	}

	if (!_gyro_valid) {
		/* keep last value if he hit an out of band value */
		lastAngRate = _ekf->angRate;

	} else {
		perf_count(_perf_gyro);
	}

	if (accel_updated) {

		int last_accel_main = _accel_main;

		/* fail over to the 2nd accel if we know the first is down */
		if (_sensor_combined.accelerometer_errcount <= (_sensor_combined.accelerometer1_errcount + ACCEL_SWITCH_HYSTERESIS)) {
			_ekf->accel.x = _sensor_combined.accelerometer_m_s2[0];
			_ekf->accel.y = _sensor_combined.accelerometer_m_s2[1];
			_ekf->accel.z = _sensor_combined.accelerometer_m_s2[2];
			_accel_main = 0;

		} else {
			_ekf->accel.x = _sensor_combined.accelerometer1_m_s2[0];
			_ekf->accel.y = _sensor_combined.accelerometer1_m_s2[1];
			_ekf->accel.z = _sensor_combined.accelerometer1_m_s2[2];
			_accel_main = 1;
		}

		if (!_accel_valid) {
			lastAccel = _ekf->accel;
		}

		if (last_accel_main != _accel_main) {
			mavlink_and_console_log_emergency(_mavlink_fd, "ACCEL FAILED! Switched from #%d to %d", last_accel_main, _accel_main);
		}

		_accel_valid = true;
	}

	_ekf->dAngIMU = 0.5f * (_ekf->angRate + lastAngRate) * _ekf->dtIMU;
	lastAngRate = _ekf->angRate;
	_ekf->dVelIMU = 0.5f * (_ekf->accel + lastAccel) * _ekf->dtIMU;
	lastAccel = _ekf->accel;

	if (last_mag != _sensor_combined.magnetometer_timestamp) {
		_newDataMag = true;

	} else {
		_newDataMag = false;
	}

	last_mag = _sensor_combined.magnetometer_timestamp;

	//warnx("dang: %8.4f %8.4f dvel: %8.4f %8.4f", _ekf->dAngIMU.x, _ekf->dAngIMU.z, _ekf->dVelIMU.x, _ekf->dVelIMU.z);

	//Update Land Detector
	bool newLandData;
	orb_check(_landDetectorSub, &newLandData);

	if (newLandData) {
		orb_copy(ORB_ID(vehicle_land_detected), _landDetectorSub, &_landDetector);
	}

	//Update AirSpeed
	orb_check(_airspeed_sub, &_newAdsData);

	if (_newAdsData) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
		perf_count(_perf_airspeed);

		_ekf->VtasMeas = _airspeed.true_airspeed_unfiltered_m_s;
	}


	bool gps_update;
	orb_check(_gps_sub, &gps_update);

	if (gps_update) {
		orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &_gps);
		perf_count(_perf_gps);

		//We are more strict for our first fix
		float requiredAccuracy = _parameters.pos_stddev_threshold;

		if (_gpsIsGood) {
			requiredAccuracy = _parameters.pos_stddev_threshold * 2.0f;
		}

		//Check if the GPS fix is good enough for us to use
		if ((_gps.fix_type >= 3) && (_gps.eph < requiredAccuracy) && (_gps.epv < requiredAccuracy)) {
			_gpsIsGood = true;

		} else {
			_gpsIsGood = false;
		}

		if (_gpsIsGood) {

			//Calculate time since last good GPS fix
			const float dtLastGoodGPS = static_cast<float>(_gps.timestamp_position - _previousGPSTimestamp) / 1e6f;

			//Stop dead-reckoning mode
			if (_global_pos.dead_reckoning) {
				mavlink_log_info(_mavlink_fd, "[ekf] stop dead-reckoning");
			}

			_global_pos.dead_reckoning = false;

			//Fetch new GPS data
			_ekf->GPSstatus = _gps.fix_type;
			_ekf->velNED[0] = _gps.vel_n_m_s;
			_ekf->velNED[1] = _gps.vel_e_m_s;
			_ekf->velNED[2] = _gps.vel_d_m_s;

			_ekf->gpsLat = math::radians(_gps.lat / (double)1e7);
			_ekf->gpsLon = math::radians(_gps.lon / (double)1e7) - M_PI;
			_ekf->gpsHgt = _gps.alt / 1e3f;

			if (_previousGPSTimestamp != 0) {
				//Calculate average time between GPS updates
				_ekf->updateDtGpsFilt(math::constrain(dtLastGoodGPS, 0.01f, POS_RESET_THRESHOLD));

				// update LPF
				float filter_step = (dtLastGoodGPS / (rc + dtLastGoodGPS)) * (_ekf->gpsHgt - _gps_alt_filt);

				if (isfinite(filter_step)) {
					_gps_alt_filt += filter_step;
				}
			}

			//check if we had a GPS outage for a long time
			if (_gps_initialized) {

				//Convert from global frame to local frame
				map_projection_project(&_pos_ref, (_gps.lat / 1.0e7), (_gps.lon / 1.0e7), &_ekf->posNE[0], &_ekf->posNE[1]);

				if (dtLastGoodGPS > POS_RESET_THRESHOLD) {
					_ekf->ResetPosition();
					_ekf->ResetVelocity();
				}
			}

			//warnx("gps alt: %6.1f, interval: %6.3f", (double)_ekf->gpsHgt, (double)dtGoodGPS);

			// if (_gps.s_variance_m_s > 0.25f && _gps.s_variance_m_s < 100.0f * 100.0f) {
			// 	_ekf->vneSigma = sqrtf(_gps.s_variance_m_s);
			// } else {
			// 	_ekf->vneSigma = _parameters.velne_noise;
			// }

			// if (_gps.p_variance_m > 0.25f && _gps.p_variance_m < 100.0f * 100.0f) {
			// 	_ekf->posNeSigma = sqrtf(_gps.p_variance_m);
			// } else {
			// 	_ekf->posNeSigma = _parameters.posne_noise;
			// }

			// warnx("vel: %8.4f pos: %8.4f", _gps.s_variance_m_s, _gps.p_variance_m);

			_previousGPSTimestamp = _gps.timestamp_position;

		}
	}

	// If it has gone more than POS_RESET_THRESHOLD amount of seconds since we received a GPS update,
	// then something is very wrong with the GPS (possibly a hardware failure or comlink error)
	const float dtLastGoodGPS = static_cast<float>(hrt_absolute_time() - _previousGPSTimestamp) / 1e6f;

	if (dtLastGoodGPS >= POS_RESET_THRESHOLD) {

		if (_global_pos.dead_reckoning) {
			mavlink_log_info(_mavlink_fd, "[ekf] gave up dead-reckoning after long timeout");
		}

		_gpsIsGood = false;
		_global_pos.dead_reckoning = false;
	}

	//If we have no good GPS fix for half a second, then enable dead-reckoning mode while armed (for up to POS_RESET_THRESHOLD seconds)
	else if (dtLastGoodGPS >= 0.5f) {
		if (_armed.armed) {
			if (!_global_pos.dead_reckoning) {
				mavlink_log_info(_mavlink_fd, "[ekf] dead-reckoning enabled");
			}

			_global_pos.dead_reckoning = true;

		} else {
			_global_pos.dead_reckoning = false;
		}
	}

	//Update barometer
	orb_check(_baro_sub, &_newHgtData);

	if (_newHgtData) {
		static hrt_abstime baro_last = 0;

		orb_copy(ORB_ID(sensor_baro), _baro_sub, &_baro);

		// init lowpass filters for baro and gps altitude
		float baro_elapsed;

		if (baro_last == 0) {
			baro_elapsed = 0.0f;

		} else {
			baro_elapsed = (_baro.timestamp - baro_last) / 1e6f;
		}

		baro_last = _baro.timestamp;
		if (!_baro_init) {
			_baro_init = true;
			_baro_alt_filt = _baro.altitude;
		}

		_ekf->updateDtHgtFilt(math::constrain(baro_elapsed, 0.001f, 0.1f));

		_ekf->baroHgt = _baro.altitude;
		float filter_step = (baro_elapsed / (rc + baro_elapsed)) * (_baro.altitude - _baro_alt_filt);

		if (isfinite(filter_step)) {
			_baro_alt_filt += filter_step;
		}

		perf_count(_perf_baro);
	}

	//Update Magnetometer
	if (_newDataMag) {

		_mag_valid = true;

		perf_count(_perf_mag);

		int last_mag_main = _mag_main;

		Vector3f mag0(_sensor_combined.magnetometer_ga[0], _sensor_combined.magnetometer_ga[1],
			_sensor_combined.magnetometer_ga[2]);

		Vector3f mag1(_sensor_combined.magnetometer1_ga[0], _sensor_combined.magnetometer1_ga[1],
			_sensor_combined.magnetometer1_ga[2]);

		const unsigned mag_timeout_us = 200000;

		/* fail over to the 2nd mag if we know the first is down */
		if (hrt_elapsed_time(&_sensor_combined.magnetometer_timestamp) < mag_timeout_us &&
			_sensor_combined.magnetometer_errcount <= (_sensor_combined.magnetometer1_errcount + MAG_SWITCH_HYSTERESIS) &&
			mag0.length() > 0.1f) {
			_ekf->magData.x = mag0.x;
			_ekf->magBias.x = 0.000001f; // _mag_offsets.x_offset

			_ekf->magData.y = mag0.y;
			_ekf->magBias.y = 0.000001f; // _mag_offsets.y_offset

			_ekf->magData.z = mag0.z;
			_ekf->magBias.z = 0.000001f; // _mag_offsets.y_offset
			_mag_main = 0;

		} else if (hrt_elapsed_time(&_sensor_combined.magnetometer1_timestamp) < mag_timeout_us &&
			mag1.length() > 0.1f) {
			_ekf->magData.x = mag1.x;
			_ekf->magBias.x = 0.000001f; // _mag_offsets.x_offset

			_ekf->magData.y = mag1.y;
			_ekf->magBias.y = 0.000001f; // _mag_offsets.y_offset

			_ekf->magData.z = mag1.z;
			_ekf->magBias.z = 0.000001f; // _mag_offsets.y_offset
			_mag_main = 1;
		} else {
			_mag_valid = false;
		}

		if (last_mag_main != _mag_main) {
			mavlink_and_console_log_emergency(_mavlink_fd, "MAG FAILED! Failover from unit %d to unit %d", last_mag_main, _mag_main);
		}
	}

	//Update range data
	orb_check(_distance_sub, &_newRangeData);

	if (_newRangeData) {
		orb_copy(ORB_ID(sensor_range_finder), _distance_sub, &_distance);

		if (_distance.valid) {
			_ekf->rngMea = _distance.distance;
			_distance_last_valid = _distance.timestamp;

		} else {
			_newRangeData = false;
		}
	}
}

int AttitudePositionEstimatorEKF::trip_nan()
{

	int ret = 0;

	// If system is not armed, inject a NaN value into the filter
	if (_armed.armed) {
		warnx("ACTUATORS ARMED! NOT TRIPPING SYSTEM");
		ret = 1;

	} else {

		float nan_val = 0.0f / 0.0f;

		warnx("system not armed, tripping state vector with NaN");
		_ekf->states[5] = nan_val;
		usleep(100000);

		warnx("tripping covariance #1 with NaN");
		_ekf->KH[2][2] = nan_val; //  intermediate result used for covariance updates
		usleep(100000);

		warnx("tripping covariance #2 with NaN");
		_ekf->KHP[5][5] = nan_val; // intermediate result used for covariance updates
		usleep(100000);

		warnx("tripping covariance #3 with NaN");
		_ekf->P[3][3] = nan_val; // covariance matrix
		usleep(100000);

		warnx("tripping Kalman gains with NaN");
		_ekf->Kfusion[0] = nan_val; // Kalman gains
		usleep(100000);

		warnx("tripping stored states[0] with NaN");
		_ekf->storedStates[0][0] = nan_val;
		usleep(100000);

		warnx("tripping states[9] with NaN");
		_ekf->states[9] = nan_val;
		usleep(100000);

		warnx("\nDONE - FILTER STATE:");
		print_status();
	}

	return ret;
}

int ekf_att_pos_estimator_main(int argc, char *argv[])
{
	if (argc < 2) {
		errx(1, "usage: ekf_att_pos_estimator {start|stop|status|logging}");
	}

	if (!strcmp(argv[1], "start")) {

		if (estimator::g_estimator != nullptr) {
			errx(1, "already running");
		}

		estimator::g_estimator = new AttitudePositionEstimatorEKF();

		if (estimator::g_estimator == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != estimator::g_estimator->start()) {
			delete estimator::g_estimator;
			estimator::g_estimator = nullptr;
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (estimator::g_estimator == nullptr || !estimator::g_estimator->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}

		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (estimator::g_estimator == nullptr) {
			errx(1, "not running");
		}

		delete estimator::g_estimator;
		estimator::g_estimator = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (estimator::g_estimator) {
			warnx("running");

			estimator::g_estimator->print_status();

			exit(0);

		} else {
			errx(1, "not running");
		}
	}

	if (!strcmp(argv[1], "trip")) {
		if (estimator::g_estimator) {
			int ret = estimator::g_estimator->trip_nan();

			exit(ret);

		} else {
			errx(1, "not running");
		}
	}

	if (!strcmp(argv[1], "logging")) {
		if (estimator::g_estimator) {
			int ret = estimator::g_estimator->enable_logging(true);

			exit(ret);

		} else {
			errx(1, "not running");
		}
	}

	if (!strcmp(argv[1], "debug")) {
		if (estimator::g_estimator) {
			int debug = strtoul(argv[2], NULL, 10);
			int ret = estimator::g_estimator->set_debuglevel(debug);

			exit(ret);

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
