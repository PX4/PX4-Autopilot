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
#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/mission.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

/**
 * estimator app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_att_pos_estimator_main(int argc, char *argv[]);

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

	int		_global_pos_sub;
	int		_gyro_sub;			/**< gyro sensor subscription */
	int		_accel_sub;			/**< accel sensor subscription */
	int		_mag_sub;			/**< mag sensor subscription */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_vstatus_sub;			/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;		/**< notification of manual control updates */
	int		_mission_sub;

	orb_advert_t	_att_pub;			/**< position setpoint */

	struct vehicle_attitude_s			_att;			/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct gyro_report				_gyro;
	struct accel_report				_accel;
	struct mag_report				_mag;
	struct airspeed_s				_airspeed;		/**< airspeed */
	struct vehicle_status_s				_vstatus;		/**< vehicle status */
	struct vehicle_global_position_s		_global_pos;		/**< global vehicle position */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	unsigned	_mission_items_maxcount;				/**< maximum number of mission items supported */
	struct	mission_item_s 				* _mission_items;	/**< storage for mission items */
	bool		_mission_valid;						/**< flag if mission is valid */

	/** manual control states */
	float		_seatbelt_hold_heading;		/**< heading the system should hold in seatbelt mode */
	float		_loiter_hold_lat;
	float		_loiter_hold_lon;
	float		_loiter_hold_alt;
	bool		_loiter_hold;

	struct {
		float throttle_cruise;
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
	_global_pos_sub(-1),
	_gyro_sub(-1),
	_accel_sub(-1),
	_mag_sub(-1),
	_airspeed_sub(-1),
	_vstatus_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),

/* publications */
	_att_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw_att_pos_estimator")),
/* states */
	_mission_items_maxcount(20),
	_mission_valid(false),
	_loiter_hold(false)
{

	_parameter_handles.throttle_cruise = param_find("NAV_DUMMY");

	/* fetch initial parameter values */
	parameters_update();
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

	//param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));

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

void
FixedwingEstimator::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vstatus_sub, 200);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	parameters_update();

	/* wakeup source(s) */
	struct pollfd fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _gyro_sub;
	fds[1].events = POLLIN;

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

		/* check vehicle status for changes to publication state */
		vehicle_status_poll();

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

			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(sensor_gyro), _gyro_sub, &_gyro);

			bool updated;

			orb_check(_accel_sub, &updated);
			if (updated)
				orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);

			orb_check(_mag_sub, &updated);
			if (updated)
				orb_copy(ORB_ID(sensor_mag), _mag_sub, &_mag);

			orb_check(_airspeed_sub, &updated);
			if (updated)
				orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);

			/* lazily publish the attitude only once available */
			if (_att_pub > 0) {
				/* publish the attitude setpoint */
				orb_publish(ORB_ID(vehicle_attitude), _att_pub, &_att);

			} else {
				/* advertise and publish */
				_att_pub = orb_advertise(ORB_ID(vehicle_attitude), &_att);
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
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&FixedwingEstimator::task_main_trampoline,
				       nullptr);

	if (_estimator_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
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
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
