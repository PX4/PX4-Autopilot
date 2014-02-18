/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file fw_att_pos_estimator_main.cpp
 * Implementation of the attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

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
#include <drivers/drv_hrt.h>

#define SENSOR_COMBINED_SUB


#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#ifdef SENSOR_COMBINED_SUB
#include <uORB/topics/sensor_combined.h>
#endif
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include "estimator.h"



/**
 * estimator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_pos_estimator_main(int argc, char *argv[]);

__EXPORT uint32_t millis();

static uint64_t last_run = 0;
static uint64_t IMUmsec = 0;

uint32_t millis()
{
	return IMUmsec;
}

static void print_status();

class FixedwingEstimator
{
public:
	/**
	 * Constructor
	 */
	FixedwingEstimator();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedwingEstimator();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	int		_estimator_task;			/**< task handle for sensor task */
#ifndef SENSOR_COMBINED_SUB
	int		_gyro_sub;			/**< gyro sensor subscription */
	int		_accel_sub;			/**< accel sensor subscription */
	int		_mag_sub;			/**< mag sensor subscription */
#else
	int		_sensor_combined_sub;
#endif
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_baro_sub;			/**< barometer subscription */
	int		_gps_sub;			/**< GPS subscription */
	int		_vstatus_sub;			/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;		/**< notification of manual control updates */
	int		_mission_sub;

	orb_advert_t	_att_pub;			/**< vehicle attitude */
	orb_advert_t	_global_pos_pub;		/**< global position */
	orb_advert_t	_local_pos_pub;			/**< position in local frame */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct gyro_report				_gyro;
	struct accel_report				_accel;
	struct mag_report				_mag;
	struct airspeed_s				_airspeed;		/**< airspeed */
	struct baro_report				_baro;			/**< baro readings */
	struct vehicle_status_s				_vstatus;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;		/**< global vehicle position */
	struct vehicle_local_position_s			_local_pos;		/**< local vehicle position */
	struct vehicle_gps_position_s			_gps;			/**< GPS position */

	struct gyro_scale				_gyro_offsets;
	struct accel_scale				_accel_offsets;
	struct mag_scale				_mag_offsets;

#ifdef SENSOR_COMBINED_SUB
	struct sensor_combined_s			_sensor_combined;
#endif

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_perf_gyro;			///<local performance counter for gyro updates
	perf_counter_t	_perf_accel;			///<local performance counter for accel updates
	perf_counter_t	_perf_mag;			///<local performance counter for mag updates
	perf_counter_t	_perf_gps;			///<local performance counter for gps updates
	perf_counter_t	_perf_baro;			///<local performance counter for baro updates
	perf_counter_t	_perf_airspeed;			///<local performance counter for airspeed updates

	bool						_initialized;

	struct {
		float		throttle_cruise;
		uint32_t	vel_delay_ms;
		uint32_t	pos_delay_ms;
		uint32_t	height_delay_ms;
		uint32_t	mag_delay_ms;
		uint32_t	tas_delay_ms;
	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t throttle_cruise;

	}		_parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle status.
	 */
	void		vehicle_status_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));
};

namespace estimator
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

FixedwingEstimator	*g_estimator;
}

FixedwingEstimator::FixedwingEstimator() :

	_task_should_exit(false),
	_estimator_task(-1),

/* subscriptions */
#ifndef SENSOR_COMBINED_SUB
	_gyro_sub(-1),
	_accel_sub(-1),
	_mag_sub(-1),
#else
	_sensor_combined_sub(-1),
#endif
	_airspeed_sub(-1),
	_baro_sub(-1),
	_gps_sub(-1),
	_vstatus_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),

/* publications */
	_att_pub(-1),
	_global_pos_pub(-1),
	_local_pos_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_COUNT, "fw_att_pos_estimator")),
	_perf_gyro(perf_alloc(PC_COUNT, "fw_ekf_gyro_upd")),
	_perf_accel(perf_alloc(PC_COUNT, "fw_ekf_accel_upd")),
	_perf_mag(perf_alloc(PC_COUNT, "fw_ekf_mag_upd")),
	_perf_gps(perf_alloc(PC_COUNT, "fw_ekf_gps_upd")),
	_perf_baro(perf_alloc(PC_COUNT, "fw_ekf_baro_upd")),
	_perf_airspeed(perf_alloc(PC_COUNT, "fw_ekf_aspd_upd")),

/* states */
	_initialized(false)
{

	_parameter_handles.throttle_cruise = param_find("NAV_DUMMY");

	/* fetch initial parameter values */
	parameters_update();

	/* get offsets */

	int fd, res;

	fd = open(GYRO_DEVICE_PATH, O_RDONLY);

	if (fd > 0) {
		res = ioctl(fd, GYROIOCGSCALE, (long unsigned int)&_gyro_offsets);
		close(fd);
	}

	fd = open(ACCEL_DEVICE_PATH, O_RDONLY);

	if (fd > 0) {
		res = ioctl(fd, ACCELIOCGSCALE, (long unsigned int)&_accel_offsets);
		close(fd);
	}

	fd = open(MAG_DEVICE_PATH, O_RDONLY);

	if (fd > 0) {
		res = ioctl(fd, MAGIOCGSCALE, (long unsigned int)&_mag_offsets);
		close(fd);
	}
}

FixedwingEstimator::~FixedwingEstimator()
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

	estimator::g_estimator = nullptr;
}

int
FixedwingEstimator::parameters_update()
{

	// XXX NEED TO GET HANDLES FIRST! NEEDS PARAM INIT
	//param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));

	_parameters.vel_delay_ms = 230;
	_parameters.pos_delay_ms = 210;
	_parameters.height_delay_ms = 350;
	_parameters.mag_delay_ms = 30;
	_parameters.tas_delay_ms = 210;

	return OK;
}

void
FixedwingEstimator::vehicle_status_poll()
{
	bool vstatus_updated;

	/* Check HIL state if vehicle status has changed */
	orb_check(_vstatus_sub, &vstatus_updated);

	if (vstatus_updated) {

		orb_copy(ORB_ID(vehicle_status), _vstatus_sub, &_vstatus);
	}
}

void
FixedwingEstimator::task_main_trampoline(int argc, char *argv[])
{
	estimator::g_estimator->task_main();
}

float dt = 0.0f; // time lapsed since last covariance prediction

// Estimated time delays (msec)
uint32_t msecVelDelay = 230;
uint32_t msecPosDelay = 210;
uint32_t msecHgtDelay = 350;
uint32_t msecMagDelay = 30;
uint32_t msecTasDelay = 210;

void
FixedwingEstimator::task_main()
{

	/*
	 * do subscriptions
	 */
	_baro_sub = orb_subscribe(ORB_ID(sensor_baro));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vstatus_sub, 200);

#ifndef SENSOR_COMBINED_SUB

	_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	_mag_sub = orb_subscribe(ORB_ID(sensor_mag));

	/* rate limit gyro updates to 50 Hz */
	/* XXX remove this!, BUT increase the data buffer size! */
	orb_set_interval(_gyro_sub, 4);
#else
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	/* XXX remove this!, BUT increase the data buffer size! */
	orb_set_interval(_sensor_combined_sub, 4);
#endif

	parameters_update();

	/* set initial filter state */
	fuseVelData = false;
	fusePosData = false;
	fuseHgtData = false;
	fuseMagData = false;
	fuseVtasData = false;
	statesInitialised = false;

	/* initialize measurement data */
	VtasMeas = 0.0f;
	Vector3f lastAngRate = {0.0f, 0.0f, 0.0f};
	Vector3f lastAccel = {0.0f, 0.0f, -9.81f};
	dVelIMU.x = 0.0f;
	dVelIMU.y = 0.0f;
	dVelIMU.z = 0.0f;
	dAngIMU.x = 0.0f;
	dAngIMU.y = 0.0f;
	dAngIMU.z = 0.0f;

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
#ifndef SENSOR_COMBINED_SUB
	fds[1].fd = _gyro_sub;
	fds[1].events = POLLIN;
#else
	fds[1].fd = _sensor_combined_sub;
	fds[1].events = POLLIN;
#endif

	hrt_abstime start_time = hrt_absolute_time();

	bool newDataGps = false;
	bool newAdsData = false;
	bool newDataMag = false;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

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
			vehicle_status_poll();

			bool accel_updated;
			bool mag_updated;

			perf_count(_perf_gyro);

			/**
			 *    PART ONE: COLLECT ALL DATA
			 **/

			hrt_abstime last_sensor_timestamp;

			/* load local copies */
#ifndef SENSOR_COMBINED_SUB
			orb_copy(ORB_ID(sensor_gyro), _gyro_sub, &_gyro);


			orb_check(_accel_sub, &accel_updated);

			if (accel_updated) {
				perf_count(_perf_accel);
				orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
			}

			last_sensor_timestamp = _gyro.timestamp;
			IMUmsec = _gyro.timestamp / 1e3f;

			float deltaT = (_gyro.timestamp - last_run) / 1e6f;
			last_run = _gyro.timestamp;

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;


			// Always store data, independent of init status
			/* fill in last data set */
			dtIMU = deltaT;

			angRate.x = _gyro.x;
			angRate.y = _gyro.y;
			angRate.z = _gyro.z;

			accel.x = _accel.x;
			accel.y = _accel.y;
			accel.z = _accel.z;

			dAngIMU = 0.5f * (angRate + lastAngRate) * dtIMU;
			lastAngRate = angRate;
			dVelIMU = 0.5f * (accel + lastAccel) * dtIMU;
			lastAccel = accel;


#else
			orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);

			static hrt_abstime last_accel = 0;
			static hrt_abstime last_mag = 0;

			if (last_accel != _sensor_combined.accelerometer_timestamp) {
				accel_updated = true;
			}

			last_accel = _sensor_combined.accelerometer_timestamp;


			// Copy gyro and accel
			last_sensor_timestamp = _sensor_combined.timestamp;
			IMUmsec = _sensor_combined.timestamp / 1e3f;

			float deltaT = (_sensor_combined.timestamp - last_run) / 1e6f;
			last_run = _sensor_combined.timestamp;

			/* guard against too large deltaT's */
			if (deltaT > 1.0f || deltaT < 0.000001f)
				deltaT = 0.01f;

			// Always store data, independent of init status
			/* fill in last data set */
			dtIMU = deltaT;

			angRate.x = _sensor_combined.gyro_rad_s[0];
			angRate.y = _sensor_combined.gyro_rad_s[1];
			angRate.z = _sensor_combined.gyro_rad_s[2];

			accel.x = _sensor_combined.accelerometer_m_s2[0];
			accel.y = _sensor_combined.accelerometer_m_s2[1];
			accel.z = _sensor_combined.accelerometer_m_s2[2];

			dAngIMU = 0.5f * (angRate + lastAngRate) * dtIMU;
			lastAngRate = angRate;
			dVelIMU = 0.5f * (accel + lastAccel) * dtIMU;
			lastAccel = accel;

			if (last_mag != _sensor_combined.magnetometer_timestamp) {
				mag_updated = true;
				newDataMag = true;

			} else {
				newDataMag = false;
			}

			last_mag = _sensor_combined.magnetometer_timestamp;

#endif

			bool airspeed_updated;
			orb_check(_airspeed_sub, &airspeed_updated);

			if (airspeed_updated) {
				orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
				perf_count(_perf_airspeed);

				VtasMeas = _airspeed.true_airspeed_m_s;
				newAdsData = true;

			} else {
				newAdsData = false;
			}

			bool gps_updated;
			orb_check(_gps_sub, &gps_updated);

			if (gps_updated) {
				orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &_gps);
				perf_count(_perf_gps);

				if (_gps.fix_type < 3) {
					gps_updated = false;
					newDataGps = false;

				} else {
					/* fuse GPS updates */

					//_gps.timestamp / 1e3;
					GPSstatus = _gps.fix_type;
					velNED[0] = _gps.vel_n_m_s;
					velNED[1] = _gps.vel_e_m_s;
					velNED[2] = _gps.vel_d_m_s;

					// warnx("GPS updated: status: %d, vel: %8.4f %8.4f %8.4f", (int)GPSstatus, velNED[0], velNED[1], velNED[2]);

					gpsLat = math::radians(_gps.lat / (double)1e7);
					gpsLon = math::radians(_gps.lon / (double)1e7) - M_PI;
					gpsHgt = _gps.alt / 1e3f;
					newDataGps = true;

				}

			}

			bool baro_updated;
			orb_check(_baro_sub, &baro_updated);

			if (baro_updated) {
				orb_copy(ORB_ID(sensor_baro), _baro_sub, &_baro);

				baroHgt = _baro.altitude;

				// Could use a blend of GPS and baro alt data if desired
				hgtMea = 1.0f * baroHgt + 0.0f * gpsHgt;
			}

#ifndef SENSOR_COMBINED_SUB
			orb_check(_mag_sub, &mag_updated);
#endif

			if (mag_updated) {

				perf_count(_perf_mag);

#ifndef SENSOR_COMBINED_SUB
				orb_copy(ORB_ID(sensor_mag), _mag_sub, &_mag);

				// XXX we compensate the offsets upfront - should be close to zero.
				// 0.001f
				magData.x = _mag.x;
				magBias.x = 0.000001f; // _mag_offsets.x_offset

				magData.y = _mag.y;
				magBias.y = 0.000001f; // _mag_offsets.y_offset

				magData.z = _mag.z;
				magBias.z = 0.000001f; // _mag_offsets.y_offset

#else

				// XXX we compensate the offsets upfront - should be close to zero.
				// 0.001f
				magData.x = _sensor_combined.magnetometer_ga[0];
				magBias.x = 0.000001f; // _mag_offsets.x_offset

				magData.y = _sensor_combined.magnetometer_ga[1];
				magBias.y = 0.000001f; // _mag_offsets.y_offset

				magData.z = _sensor_combined.magnetometer_ga[2];
				magBias.z = 0.000001f; // _mag_offsets.y_offset

#endif

				newDataMag = true;

			} else {
				newDataMag = false;
			}


			/**
			 *    PART TWO: EXECUTE THE FILTER
			 **/

			// Wait long enough to ensure all sensors updated once
			// XXX we rather want to check all updated


			if (hrt_elapsed_time(&start_time) > 100000) {

				if (!_gps_initialized && (GPSstatus == 3)) {
					velNED[0] = _gps.vel_n_m_s;
					velNED[1] = _gps.vel_e_m_s;
					velNED[2] = _gps.vel_d_m_s;
					InitialiseFilter(velNED);

					_gps_initialized = true;

				} else if (!statesInitialised) {
					velNED[0] = _gps.vel_n_m_s;
					velNED[1] = _gps.vel_e_m_s;
					velNED[2] = _gps.vel_d_m_s;
					InitialiseFilter(velNED);
				}
			}

			// If valid IMU data and states initialised, predict states and covariances
			if (statesInitialised) {
				// Run the strapdown INS equations every IMU update
				UpdateStrapdownEquationsNED();
#if 0
				// debug code - could be tunred into a filter mnitoring/watchdog function
				float tempQuat[4];

				for (uint8_t j = 0; j <= 3; j++) tempQuat[j] = states[j];

				quat2eul(eulerEst, tempQuat);

				for (uint8_t j = 0; j <= 2; j++) eulerDif[j] = eulerEst[j] - ahrsEul[j];

				if (eulerDif[2] > pi) eulerDif[2] -= 2 * pi;

				if (eulerDif[2] < -pi) eulerDif[2] += 2 * pi;

#endif
				// store the predicted states for subsequent use by measurement fusion
				StoreStates(IMUmsec);
				// Check if on ground - status is used by covariance prediction
				OnGroundCheck();
				onGround = false;
				// sum delta angles and time used by covariance prediction
				summedDelAng = summedDelAng + correctedDelAng;
				summedDelVel = summedDelVel + dVelIMU;
				dt += dtIMU;

				// perform a covariance prediction if the total delta angle has exceeded the limit
				// or the time limit will be exceeded at the next IMU update
				if ((dt >= (covTimeStepMax - dtIMU)) || (summedDelAng.length() > covDelAngMax)) {
					CovariancePrediction(dt);
					summedDelAng = summedDelAng.zero();
					summedDelVel = summedDelVel.zero();
					dt = 0.0f;
				}

				_initialized = true;
			}

			// Fuse GPS Measurements
			if (newDataGps && _gps_initialized) {
				// Convert GPS measurements to Pos NE, hgt and Vel NED
				velNED[0] = _gps.vel_n_m_s;
				velNED[1] = _gps.vel_e_m_s;
				velNED[2] = _gps.vel_d_m_s;
				calcposNED(posNED, gpsLat, gpsLon, gpsHgt, latRef, lonRef, hgtRef);

				posNE[0] = posNED[0];
				posNE[1] = posNED[1];
				// set fusion flags
				fuseVelData = true;
				fusePosData = true;
				// recall states stored at time of measurement after adjusting for delays
				RecallStates(statesAtVelTime, (IMUmsec - msecVelDelay));
				RecallStates(statesAtPosTime, (IMUmsec - msecPosDelay));
				// run the fusion step
				FuseVelposNED();

			} else if (statesInitialised) {
				// Convert GPS measurements to Pos NE, hgt and Vel NED
				velNED[0] = 0.0f;
				velNED[1] = 0.0f;
				velNED[2] = 0.0f;
				posNED[0] = 0.0f;
				posNED[1] = 0.0f;
				posNED[2] = 0.0f;

				posNE[0] = posNED[0];
				posNE[1] = posNED[1];
				// set fusion flags
				fuseVelData = true;
				fusePosData = true;
				// recall states stored at time of measurement after adjusting for delays
				RecallStates(statesAtVelTime, (IMUmsec - msecVelDelay));
				RecallStates(statesAtPosTime, (IMUmsec - msecPosDelay));
				// run the fusion step
				FuseVelposNED();
			} else {
				fuseVelData = false;
				fusePosData = false;
			}

			if (newAdsData && statesInitialised) {
				// Could use a blend of GPS and baro alt data if desired
				hgtMea = 1.0f * baroHgt + 0.0f * gpsHgt;
				fuseHgtData = true;
				// recall states stored at time of measurement after adjusting for delays
				RecallStates(statesAtHgtTime, (IMUmsec - msecHgtDelay));
				// run the fusion step
				FuseVelposNED();

			} else {
				fuseHgtData = false;
			}

			// Fuse Magnetometer Measurements
			if (newDataMag && statesInitialised) {
				fuseMagData = true;
				RecallStates(statesAtMagMeasTime, (IMUmsec - msecMagDelay)); // Assume 50 msec avg delay for magnetometer data

			} else {
				fuseMagData = false;
			}

			if (statesInitialised) FuseMagnetometer();

			// Fuse Airspeed Measurements
			if (newAdsData && statesInitialised && VtasMeas > 8.0f) {
				fuseVtasData = true;
				RecallStates(statesAtVtasMeasTime, (IMUmsec - msecTasDelay)); // assume 100 msec avg delay for airspeed data
				FuseAirspeed();

			} else {
				fuseVtasData = false;
			}

			// if (hrt_elapsed_time(&start_time) > 100000 && !_initialized && (GPSstatus == 3)) {
			// 	InitialiseFilter(velNED);
			// 	_initialized = true;

			// 	warnx("init done.");
			// }

			// if (_initialized) {

			// 	/* predict states and covariances */

			// 	/* run the strapdown INS every sensor update */
			// 	UpdateStrapdownEquationsNED();

			// 	/* store the predictions */
			// 	StoreStates(IMUmsec);

			// 	/* evaluate if on ground */
			// 	// OnGroundCheck();
			// 	onGround = false;

			// 	/* prepare the delta angles and time used by the covariance prediction */
			// 	summedDelAng = summedDelAng + correctedDelAng;
			// 	summedDelVel = summedDelVel + correctedDelVel;

			// 	dt += dtIMU;

			// 	/* predict the covairance if the total delta angle has exceeded the threshold
			// 	 * or the time limit will be exceeded on the next measurement update
			// 	 */
			// 	if ((dt >= (covTimeStepMax - dtIMU)) || (summedDelAng.length() > covDelAngMax)) {
			// 		CovariancePrediction(dt);
			// 		summedDelAng = summedDelAng.zero();
			// 		summedDelVel = summedDelVel.zero();
			// 		dt = 0.0f;
			// 	}

			// }


			// if (gps_updated && _initialized) {

			// 	/* convert GPS measurements to horizontal NE, altitude and 3D velocity NED */
			// 	calcposNED(posNED, gpsLat, gpsLon, gpsHgt, latRef, lonRef, hgtRef);

			// 	posNE[0] = posNED[0];
			// 	posNE[1] = posNED[1];

			// 	// set flags for further processing
			// 	fuseVelData = true;
			// 	fusePosData = true;

			// 	/* recall states after adjusting for delays */
			// 	RecallStates(statesAtVelTime, (IMUmsec - _parameters.vel_delay_ms));
			// 	RecallStates(statesAtPosTime, (IMUmsec - _parameters.pos_delay_ms));

			// 	/* run the actual fusion */
			// 	FuseVelposNED();
			// } else {
			// 	fuseVelData = false;
			// 	fusePosData = false;
			// }

			// if (baro_updated && _initialized) {

			// 	fuseHgtData = true;
			// 	// recall states stored at time of measurement after adjusting for delays
			// 	RecallStates(statesAtHgtTime, (IMUmsec - _parameters.height_delay_ms));
			// 	// run the fusion step
			// 	FuseVelposNED();
			// } else {
			// 	fuseHgtData = false;
			// }

			// if (mag_updated && _initialized) {
			// 	fuseMagData = true;
			// 	RecallStates(statesAtMagMeasTime, (IMUmsec - _parameters.mag_delay_ms));

			// } else {
			// 	fuseMagData = false;
			// }

			// if (_initialized) {
			// 	FuseMagnetometer();
			// }

			// if (airspeed_updated && _initialized
			// 	&& _airspeed.true_airspeed_m_s > 6.0f /* XXX magic number */) {

			// 	fuseVtasData = true;
			// 	RecallStates(statesAtVtasMeasTime, (IMUmsec - _parameters.tas_delay_ms)); // assume 100 msec avg delay for airspeed data
			// 	FuseAirspeed();
			// } else {
			// 	fuseVtasData = false;
			// }

			// Publish results
			if (_initialized) {



				// State vector:
				// 0-3: quaternions (q0, q1, q2, q3)
				// 4-6: Velocity - m/sec (North, East, Down)
				// 7-9: Position - m (North, East, Down)
				// 10-12: Delta Angle bias - rad (X,Y,Z)
				// 13-14: Wind Vector  - m/sec (North,East)
				// 15-17: Earth Magnetic Field Vector - milligauss (North, East, Down)
				// 18-20: Body Magnetic Field Vector - milligauss (X,Y,Z)

				math::Quaternion q(states[0], states[1], states[2], states[3]);
				math::Matrix<3, 3> R = q.to_dcm();
				math::Vector<3> euler = R.to_euler();

				for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
						_att.R[i][j] = R(i, j);

				_att.timestamp = last_sensor_timestamp;
				_att.q[0] = states[0];
				_att.q[1] = states[1];
				_att.q[2] = states[2];
				_att.q[3] = states[3];
				_att.q_valid = true;
				_att.R_valid = true;

				_att.timestamp = last_sensor_timestamp;
				_att.roll = euler(0);
				_att.pitch = euler(1);
				_att.yaw = euler(2);

				_att.rollspeed = angRate.x - states[10];
				_att.pitchspeed = angRate.y - states[11];
				_att.yawspeed = angRate.z - states[12];
				// gyro offsets
				_att.rate_offsets[0] = states[10];
				_att.rate_offsets[1] = states[11];
				_att.rate_offsets[2] = states[12];

				/* lazily publish the attitude only once available */
				if (_att_pub > 0) {
					/* publish the attitude setpoint */
					orb_publish(ORB_ID(vehicle_attitude), _att_pub, &_att);

				} else {
					/* advertise and publish */
					_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &_att);
				}

				_local_pos.timestamp = last_sensor_timestamp;
				_local_pos.x = states[7];
				_local_pos.y = states[8];
				_local_pos.z = states[9];

				_local_pos.vx = states[4];
				_local_pos.vy = states[5];
				_local_pos.vz = states[6];

				_local_pos.xy_valid = true;
				_local_pos.z_valid = true;
				_local_pos.v_xy_valid = true;
				_local_pos.v_z_valid = true;

				/* lazily publish the local position only once available */
				if (_local_pos_pub > 0) {
					/* publish the attitude setpoint */
					orb_publish(ORB_ID(vehicle_local_position), _local_pos_pub, &_local_pos);

				} else {
					/* advertise and publish */
					_local_pos_pub = orb_advertise(ORB_ID(vehicle_local_position), &_local_pos);
				}

				_global_pos.timestamp = _gyro.timestamp;

				// /* lazily publish the global position only once available */
				// if (_global_pos_pub > 0) {
				// 	/* publish the attitude setpoint */
				// 	orb_publish(ORB_ID(vehicle_global_position), _global_pos_pub, &_global_pos);

				// } else {
				// 	/* advertise and publish */
				// 	_global_pos_pub = orb_advertise(ORB_ID(vehicle_global_position), &_global_pos);
				// }
			}

		}

		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_estimator_task = -1;
	_exit(0);
}

int
FixedwingEstimator::start()
{
	ASSERT(_estimator_task == -1);

	/* start the task */
	_estimator_task = task_spawn_cmd("fw_att_pos_estimator",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 40,
					 6000,
					 (main_t)&FixedwingEstimator::task_main_trampoline,
					 nullptr);

	if (_estimator_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void print_status()
{
	math::Quaternion q(states[0], states[1], states[2], states[3]);
	math::Matrix<3, 3> R = q.to_dcm();
	math::Vector<3> euler = R.to_euler();

	printf("attitude: roll: %8.4f, pitch %8.4f, yaw: %8.4f degrees\n",
	       (double)math::degrees(euler(0)), (double)math::degrees(euler(1)), (double)math::degrees(euler(2)));

	// State vector:
	// 0-3: quaternions (q0, q1, q2, q3)
	// 4-6: Velocity - m/sec (North, East, Down)
	// 7-9: Position - m (North, East, Down)
	// 10-12: Delta Angle bias - rad (X,Y,Z)
	// 13-14: Wind Vector  - m/sec (North,East)
	// 15-17: Earth Magnetic Field Vector - milligauss (North, East, Down)
	// 18-20: Body Magnetic Field Vector - milligauss (X,Y,Z)

	printf("dtIMU: %8.6f dt: %8.6f IMUmsec: %d\n", dtIMU, dt, (int)IMUmsec);
	printf("dvel: %8.6f %8.6f %8.6f accel: %8.6f %8.6f %8.6f\n", (double)dVelIMU.x, (double)dVelIMU.y, (double)dVelIMU.z, (double)accel.x, (double)accel.y, (double)accel.z);
	printf("dang: %8.4f %8.4f %8.4f dang corr: %8.4f %8.4f %8.4f\n" , (double)dAngIMU.x, (double)dAngIMU.y, (double)dAngIMU.z, (double)correctedDelAng.x, (double)correctedDelAng.y, (double)correctedDelAng.z);
	printf("states (quat)        [1-4]: %8.4f, %8.4f, %8.4f, %8.4f\n", (double)states[0], (double)states[1], (double)states[2], (double)states[3]);
	printf("states (vel m/s)     [5-7]: %8.4f, %8.4f, %8.4f\n", (double)states[4], (double)states[5], (double)states[6]);
	printf("states (pos m)      [8-10]: %8.4f, %8.4f, %8.4f\n", (double)states[7], (double)states[8], (double)states[9]);
	printf("states (delta ang) [11-13]: %8.4f, %8.4f, %8.4f\n", (double)states[10], (double)states[11], (double)states[12]);
	printf("states (wind)      [14-15]: %8.4f, %8.4f\n", (double)states[13], (double)states[14]);
	printf("states (earth mag) [16-18]: %8.4f, %8.4f, %8.4f\n", (double)states[15], (double)states[16], (double)states[17]);
	printf("states (body mag)  [19-21]: %8.4f, %8.4f, %8.4f\n", (double)states[18], (double)states[19], (double)states[20]);
	printf("states: %s %s %s %s %s %s %s %s %s\n",
	       (statesInitialised) ? "INITIALIZED" : "NON_INIT",
	       (onGround) ? "ON_GROUND" : "AIRBORNE",
	       (fuseVelData) ? "FUSE_VEL" : "INH_VEL",
	       (fusePosData) ? "FUSE_POS" : "INH_POS",
	       (fuseHgtData) ? "FUSE_HGT" : "INH_HGT",
	       (fuseMagData) ? "FUSE_MAG" : "INH_MAG",
	       (fuseVtasData) ? "FUSE_VTAS" : "INH_VTAS",
	       (useAirspeed) ? "USE_AIRSPD" : "IGN_AIRSPD",
	       (useCompass) ? "USE_COMPASS" : "IGN_COMPASS");
}

int fw_att_pos_estimator_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: fw_att_pos_estimator {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (estimator::g_estimator != nullptr)
			errx(1, "already running");

		estimator::g_estimator = new FixedwingEstimator;

		if (estimator::g_estimator == nullptr)
			errx(1, "alloc failed");

		if (OK != estimator::g_estimator->start()) {
			delete estimator::g_estimator;
			estimator::g_estimator = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (estimator::g_estimator == nullptr)
			errx(1, "not running");

		delete estimator::g_estimator;
		estimator::g_estimator = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (estimator::g_estimator) {
			warnx("running");

			print_status();

			exit(0);

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
