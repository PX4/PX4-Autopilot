/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file sensors.cpp
 * Sensor readout process.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>

#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>

#include <nuttx/analog/adc.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_airspeed.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <conversion/rotation.h>

#include <systemlib/airspeed.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>

#define GYRO_HEALTH_COUNTER_LIMIT_ERROR 20   /* 40 ms downtime at 500 Hz update rate   */
#define ACC_HEALTH_COUNTER_LIMIT_ERROR  20   /* 40 ms downtime at 500 Hz update rate   */
#define MAGN_HEALTH_COUNTER_LIMIT_ERROR 100  /* 1000 ms downtime at 100 Hz update rate  */
#define BARO_HEALTH_COUNTER_LIMIT_ERROR 50   /* 500 ms downtime at 100 Hz update rate  */
#define ADC_HEALTH_COUNTER_LIMIT_ERROR  10   /* 100 ms downtime at 100 Hz update rate  */

#define GYRO_HEALTH_COUNTER_LIMIT_OK 5
#define ACC_HEALTH_COUNTER_LIMIT_OK  5
#define MAGN_HEALTH_COUNTER_LIMIT_OK 5
#define BARO_HEALTH_COUNTER_LIMIT_OK 5
#define ADC_HEALTH_COUNTER_LIMIT_OK  5

/**
 * Analog layout:
 * FMU:
 * IN2 - battery voltage
 * IN3 - battery current
 * IN4 - 5V sense
 * IN10 - spare (we could actually trim these from the set)
 * IN11 - spare (we could actually trim these from the set)
 * IN12 - spare (we could actually trim these from the set)
 * IN13 - aux1
 * IN14 - aux2
 * IN15 - pressure sensor
 *
 * IO:
 * IN4 - servo supply rail
 * IN5 - analog RSSI
 */

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
#define ADC_BATTERY_VOLTAGE_CHANNEL	10
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	11
#endif

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
#define ADC_BATTERY_VOLTAGE_CHANNEL	2
#define ADC_BATTERY_CURRENT_CHANNEL	3
#define ADC_5V_RAIL_SENSE		4
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	15
#endif

#define BAT_VOL_INITIAL 0.f
#define BAT_VOL_LOWPASS_1 0.99f
#define BAT_VOL_LOWPASS_2 0.01f
#define VOLTAGE_BATTERY_IGNORE_THRESHOLD_VOLTS 3.5f

/**
 * HACK - true temperature is much less than indicated temperature in baro,
 * subtract 5 degrees in an attempt to account for the electrical upheating of the PCB
 */
#define PCB_TEMP_ESTIMATE_DEG 5.0f

#define limit_minus_one_to_one(arg) (arg < -1.0f) ? -1.0f : ((arg > 1.0f) ? 1.0f : arg)

/**
 * Sensor app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int sensors_main(int argc, char *argv[]);

class Sensors
{
public:
	/**
	 * Constructor
	 */
	Sensors();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~Sensors();

	/**
	 * Start the sensors task.
	 *
	 * @return		OK on success.
	 */
	int		start();

private:
	static const unsigned _rc_max_chan_count = RC_INPUT_MAX_CHANNELS;	/**< maximum number of r/c channels we handle */

	hrt_abstime	_rc_last_valid;		/**< last time we got a valid RC signal */

	/**
	 * Gather and publish RC input data.
	 */
	void		rc_poll();

	/* XXX should not be here - should be own driver */
	int 		_fd_adc;			/**< ADC driver handle */
	hrt_abstime	_last_adc;			/**< last time we took input from the ADC */

	bool 		_task_should_exit;		/**< if true, sensor task should exit */
	int 		_sensors_task;			/**< task handle for sensor task */

	bool		_hil_enabled;			/**< if true, HIL is active */
	bool		_publishing;			/**< if true, we are publishing sensor data */

	int		_gyro_sub;			/**< raw gyro data subscription */
	int		_accel_sub;			/**< raw accel data subscription */
	int		_mag_sub;			/**< raw mag data subscription */
	int 		_rc_sub;			/**< raw rc channels data subscription */
	int		_baro_sub;			/**< raw baro data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_diff_pres_sub;			/**< raw differential pressure subscription */
	int		_vcontrol_mode_sub;			/**< vehicle control mode subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_control_sub;			/**< notification of manual control updates */

	orb_advert_t	_sensor_pub;			/**< combined sensor data topic */
	orb_advert_t	_manual_control_pub;		/**< manual control signal topic */
	orb_advert_t	_actuator_group_3_pub;		/**< manual control as actuator topic */
	orb_advert_t	_rc_pub;			/**< raw r/c control topic */
	orb_advert_t	_battery_pub;			/**< battery status */
	orb_advert_t	_airspeed_pub;			/**< airspeed */
	orb_advert_t	_diff_pres_pub;			/**< differential_pressure */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	struct rc_channels_s _rc;			/**< r/c channel data */
	struct battery_status_s _battery_status;	/**< battery status */
	struct baro_report _barometer;			/**< barometer data */
	struct differential_pressure_s _diff_pres;
	struct airspeed_s _airspeed;

	math::Matrix	_board_rotation;		/**< rotation matrix for the orientation that the board is mounted */
	math::Matrix	_external_mag_rotation;		/**< rotation matrix for the orientation that an external mag is mounted */
	bool		_mag_is_external;		/**< true if the active mag is on an external board */

	struct {
		float min[_rc_max_chan_count];
		float trim[_rc_max_chan_count];
		float max[_rc_max_chan_count];
		float rev[_rc_max_chan_count];
		float dz[_rc_max_chan_count];
		float scaling_factor[_rc_max_chan_count];

		float gyro_offset[3];
		float gyro_scale[3];
		float mag_offset[3];
		float mag_scale[3];
		float accel_offset[3];
		float accel_scale[3];
		float diff_pres_offset_pa;
		float diff_pres_analog_enabled;

		int board_rotation;
		int external_mag_rotation;

		int rc_map_roll;
		int rc_map_pitch;
		int rc_map_yaw;
		int rc_map_throttle;

		int rc_map_mode_sw;
		int rc_map_return_sw;
		int rc_map_assisted_sw;
		int rc_map_mission_sw;

//		int rc_map_offboard_ctrl_mode_sw;

		int rc_map_flaps;

		int rc_map_aux1;
		int rc_map_aux2;
		int rc_map_aux3;
		int rc_map_aux4;
		int rc_map_aux5;

		float rc_scale_roll;
		float rc_scale_pitch;
		float rc_scale_yaw;
		float rc_scale_flaps;

		int rc_fs_ch;
		int rc_fs_mode;
		float rc_fs_thr;

		float battery_voltage_scaling;

	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t min[_rc_max_chan_count];
		param_t trim[_rc_max_chan_count];
		param_t max[_rc_max_chan_count];
		param_t rev[_rc_max_chan_count];
		param_t dz[_rc_max_chan_count];

		param_t gyro_offset[3];
		param_t gyro_scale[3];
		param_t accel_offset[3];
		param_t accel_scale[3];
		param_t mag_offset[3];
		param_t mag_scale[3];
		param_t diff_pres_offset_pa;
		param_t diff_pres_analog_enabled;

		param_t rc_map_roll;
		param_t rc_map_pitch;
		param_t rc_map_yaw;
		param_t rc_map_throttle;

		param_t rc_map_mode_sw;
		param_t rc_map_return_sw;
		param_t rc_map_assisted_sw;
		param_t rc_map_mission_sw;

//		param_t rc_map_offboard_ctrl_mode_sw;

		param_t rc_map_flaps;

		param_t rc_map_aux1;
		param_t rc_map_aux2;
		param_t rc_map_aux3;
		param_t rc_map_aux4;
		param_t rc_map_aux5;

		param_t rc_scale_roll;
		param_t rc_scale_pitch;
		param_t rc_scale_yaw;
		param_t rc_scale_flaps;

		param_t rc_fs_ch;
		param_t rc_fs_mode;
		param_t rc_fs_thr;

		param_t battery_voltage_scaling;

		param_t board_rotation;
		param_t external_mag_rotation;

	}		_parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Do accel-related initialisation.
	 */
	void		accel_init();

	/**
	 * Do gyro-related initialisation.
	 */
	void		gyro_init();

	/**
	 * Do mag-related initialisation.
	 */
	void		mag_init();

	/**
	 * Do baro-related initialisation.
	 */
	void		baro_init();

	/**
	 * Do adc-related initialisation.
	 */
	void		adc_init();

	/**
	 * Poll the accelerometer for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		accel_poll(struct sensor_combined_s &raw);

	/**
	 * Poll the gyro for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		gyro_poll(struct sensor_combined_s &raw);

	/**
	 * Poll the magnetometer for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		mag_poll(struct sensor_combined_s &raw);

	/**
	 * Poll the barometer for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		baro_poll(struct sensor_combined_s &raw);

	/**
	 * Poll the differential pressure sensor for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		diff_pres_poll(struct sensor_combined_s &raw);

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in parameters.
	 */
	void 		parameter_update_poll(bool forced = false);

	/**
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll(struct sensor_combined_s &raw);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main() __attribute__((noreturn));
};

namespace sensors
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

Sensors	*g_sensors = nullptr;
}

Sensors::Sensors() :
	_rc_last_valid(0),

	_fd_adc(-1),
	_last_adc(0),

	_task_should_exit(false),
	_sensors_task(-1),
	_hil_enabled(false),
	_publishing(true),

/* subscriptions */
	_gyro_sub(-1),
	_accel_sub(-1),
	_mag_sub(-1),
	_rc_sub(-1),
	_baro_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),

/* publications */
	_sensor_pub(-1),
	_manual_control_pub(-1),
	_actuator_group_3_pub(-1),
	_rc_pub(-1),
	_battery_pub(-1),
	_airspeed_pub(-1),
	_diff_pres_pub(-1),

/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "sensor task update")),

	_board_rotation(3, 3),
	_external_mag_rotation(3, 3),
	_mag_is_external(false)
{

	/* basic r/c parameters */
	for (unsigned i = 0; i < _rc_max_chan_count; i++) {
		char nbuf[16];

		/* min values */
		sprintf(nbuf, "RC%d_MIN", i + 1);
		_parameter_handles.min[i] = param_find(nbuf);

		/* trim values */
		sprintf(nbuf, "RC%d_TRIM", i + 1);
		_parameter_handles.trim[i] = param_find(nbuf);

		/* max values */
		sprintf(nbuf, "RC%d_MAX", i + 1);
		_parameter_handles.max[i] = param_find(nbuf);

		/* channel reverse */
		sprintf(nbuf, "RC%d_REV", i + 1);
		_parameter_handles.rev[i] = param_find(nbuf);

		/* channel deadzone */
		sprintf(nbuf, "RC%d_DZ", i + 1);
		_parameter_handles.dz[i] = param_find(nbuf);

	}

	/* mandatory input switched, mapped to channels 1-4 per default */
	_parameter_handles.rc_map_roll 	= param_find("RC_MAP_ROLL");
	_parameter_handles.rc_map_pitch = param_find("RC_MAP_PITCH");
	_parameter_handles.rc_map_yaw 	= param_find("RC_MAP_YAW");
	_parameter_handles.rc_map_throttle = param_find("RC_MAP_THROTTLE");

	/* mandatory mode switches, mapped to channel 5 and 6 per default */
	_parameter_handles.rc_map_mode_sw = param_find("RC_MAP_MODE_SW");
	_parameter_handles.rc_map_return_sw = param_find("RC_MAP_RETURN_SW");

	_parameter_handles.rc_map_flaps = param_find("RC_MAP_FLAPS");

	/* optional mode switches, not mapped per default */
	_parameter_handles.rc_map_assisted_sw = param_find("RC_MAP_ASSIST_SW");
	_parameter_handles.rc_map_mission_sw = param_find("RC_MAP_MISSIO_SW");

//	_parameter_handles.rc_map_offboard_ctrl_mode_sw = param_find("RC_MAP_OFFB_SW");

	_parameter_handles.rc_map_aux1 = param_find("RC_MAP_AUX1");
	_parameter_handles.rc_map_aux2 = param_find("RC_MAP_AUX2");
	_parameter_handles.rc_map_aux3 = param_find("RC_MAP_AUX3");
	_parameter_handles.rc_map_aux4 = param_find("RC_MAP_AUX4");
	_parameter_handles.rc_map_aux5 = param_find("RC_MAP_AUX5");

	_parameter_handles.rc_scale_roll = param_find("RC_SCALE_ROLL");
	_parameter_handles.rc_scale_pitch = param_find("RC_SCALE_PITCH");
	_parameter_handles.rc_scale_yaw = param_find("RC_SCALE_YAW");
	_parameter_handles.rc_scale_flaps = param_find("RC_SCALE_FLAPS");

	/* RC failsafe */
	_parameter_handles.rc_fs_ch = param_find("RC_FS_CH");
	_parameter_handles.rc_fs_mode = param_find("RC_FS_MODE");
	_parameter_handles.rc_fs_thr = param_find("RC_FS_THR");

	/* gyro offsets */
	_parameter_handles.gyro_offset[0] = param_find("SENS_GYRO_XOFF");
	_parameter_handles.gyro_offset[1] = param_find("SENS_GYRO_YOFF");
	_parameter_handles.gyro_offset[2] = param_find("SENS_GYRO_ZOFF");
	_parameter_handles.gyro_scale[0] = param_find("SENS_GYRO_XSCALE");
	_parameter_handles.gyro_scale[1] = param_find("SENS_GYRO_YSCALE");
	_parameter_handles.gyro_scale[2] = param_find("SENS_GYRO_ZSCALE");

	/* accel offsets */
	_parameter_handles.accel_offset[0] = param_find("SENS_ACC_XOFF");
	_parameter_handles.accel_offset[1] = param_find("SENS_ACC_YOFF");
	_parameter_handles.accel_offset[2] = param_find("SENS_ACC_ZOFF");
	_parameter_handles.accel_scale[0] = param_find("SENS_ACC_XSCALE");
	_parameter_handles.accel_scale[1] = param_find("SENS_ACC_YSCALE");
	_parameter_handles.accel_scale[2] = param_find("SENS_ACC_ZSCALE");

	/* mag offsets */
	_parameter_handles.mag_offset[0] = param_find("SENS_MAG_XOFF");
	_parameter_handles.mag_offset[1] = param_find("SENS_MAG_YOFF");
	_parameter_handles.mag_offset[2] = param_find("SENS_MAG_ZOFF");

	_parameter_handles.mag_scale[0] = param_find("SENS_MAG_XSCALE");
	_parameter_handles.mag_scale[1] = param_find("SENS_MAG_YSCALE");
	_parameter_handles.mag_scale[2] = param_find("SENS_MAG_ZSCALE");

	/* Differential pressure offset */
	_parameter_handles.diff_pres_offset_pa = param_find("SENS_DPRES_OFF");
	_parameter_handles.diff_pres_analog_enabled = param_find("SENS_DPRES_ANA");

	_parameter_handles.battery_voltage_scaling = param_find("BAT_V_SCALING");

	/* rotations */
	_parameter_handles.board_rotation = param_find("SENS_BOARD_ROT");
	_parameter_handles.external_mag_rotation = param_find("SENS_EXT_MAG_ROT");

	/* fetch initial parameter values */
	parameters_update();
}

Sensors::~Sensors()
{
	if (_sensors_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_sensors_task);
				break;
			}
		} while (_sensors_task != -1);
	}

	sensors::g_sensors = nullptr;
}

int
Sensors::parameters_update()
{
	bool rc_valid = true;
	float tmpScaleFactor = 0.0f;
	float tmpRevFactor = 0.0f;

	/* rc values */
	for (unsigned int i = 0; i < _rc_max_chan_count; i++) {

		param_get(_parameter_handles.min[i], &(_parameters.min[i]));
		param_get(_parameter_handles.trim[i], &(_parameters.trim[i]));
		param_get(_parameter_handles.max[i], &(_parameters.max[i]));
		param_get(_parameter_handles.rev[i], &(_parameters.rev[i]));
		param_get(_parameter_handles.dz[i], &(_parameters.dz[i]));

		tmpScaleFactor = (1.0f / ((_parameters.max[i] - _parameters.min[i]) / 2.0f) * _parameters.rev[i]);
		tmpRevFactor = tmpScaleFactor * _parameters.rev[i];

		/* handle blowup in the scaling factor calculation */
		if (!isfinite(tmpScaleFactor) ||
		    (tmpRevFactor < 0.000001f) ||
		    (tmpRevFactor > 0.2f)) {
			warnx("RC chan %u not sane, scaling: %8.6f, rev: %d", i, tmpScaleFactor, (int)(_parameters.rev[i]));
			/* scaling factors do not make sense, lock them down */
			_parameters.scaling_factor[i] = 0.0f;
			rc_valid = false;

		} else {
			_parameters.scaling_factor[i] = tmpScaleFactor;
		}
	}

	/* handle wrong values */
	if (!rc_valid)
		warnx("WARNING     WARNING     WARNING\n\nRC CALIBRATION NOT SANE!\n\n");

	/* channel mapping */
	if (param_get(_parameter_handles.rc_map_roll, &(_parameters.rc_map_roll)) != OK) {
		warnx("Failed getting roll chan index");
	}

	if (param_get(_parameter_handles.rc_map_pitch, &(_parameters.rc_map_pitch)) != OK) {
		warnx("Failed getting pitch chan index");
	}

	if (param_get(_parameter_handles.rc_map_yaw, &(_parameters.rc_map_yaw)) != OK) {
		warnx("Failed getting yaw chan index");
	}

	if (param_get(_parameter_handles.rc_map_throttle, &(_parameters.rc_map_throttle)) != OK) {
		warnx("Failed getting throttle chan index");
	}

	if (param_get(_parameter_handles.rc_map_mode_sw, &(_parameters.rc_map_mode_sw)) != OK) {
		warnx("Failed getting mode sw chan index");
	}

	if (param_get(_parameter_handles.rc_map_return_sw, &(_parameters.rc_map_return_sw)) != OK) {
		warnx("Failed getting return sw chan index");
	}

	if (param_get(_parameter_handles.rc_map_assisted_sw, &(_parameters.rc_map_assisted_sw)) != OK) {
		warnx("Failed getting assisted sw chan index");
	}

	if (param_get(_parameter_handles.rc_map_mission_sw, &(_parameters.rc_map_mission_sw)) != OK) {
		warnx("Failed getting mission sw chan index");
	}

	if (param_get(_parameter_handles.rc_map_flaps, &(_parameters.rc_map_flaps)) != OK) {
		warnx("Failed getting flaps chan index");
	}

//	if (param_get(_parameter_handles.rc_map_offboard_ctrl_mode_sw, &(_parameters.rc_map_offboard_ctrl_mode_sw)) != OK) {
//		warnx("Failed getting offboard control mode sw chan index");
//	}

	param_get(_parameter_handles.rc_map_aux1, &(_parameters.rc_map_aux1));
	param_get(_parameter_handles.rc_map_aux2, &(_parameters.rc_map_aux2));
	param_get(_parameter_handles.rc_map_aux3, &(_parameters.rc_map_aux3));
	param_get(_parameter_handles.rc_map_aux4, &(_parameters.rc_map_aux4));
	param_get(_parameter_handles.rc_map_aux5, &(_parameters.rc_map_aux5));
	param_get(_parameter_handles.rc_scale_roll, &(_parameters.rc_scale_roll));
	param_get(_parameter_handles.rc_scale_pitch, &(_parameters.rc_scale_pitch));
	param_get(_parameter_handles.rc_scale_yaw, &(_parameters.rc_scale_yaw));
	param_get(_parameter_handles.rc_scale_flaps, &(_parameters.rc_scale_flaps));
	param_get(_parameter_handles.rc_fs_ch, &(_parameters.rc_fs_ch));
	param_get(_parameter_handles.rc_fs_mode, &(_parameters.rc_fs_mode));
	param_get(_parameter_handles.rc_fs_thr, &(_parameters.rc_fs_thr));

	/* update RC function mappings */
	_rc.function[THROTTLE] = _parameters.rc_map_throttle - 1;
	_rc.function[ROLL] = _parameters.rc_map_roll - 1;
	_rc.function[PITCH] = _parameters.rc_map_pitch - 1;
	_rc.function[YAW] = _parameters.rc_map_yaw - 1;

	_rc.function[MODE] = _parameters.rc_map_mode_sw - 1;
	_rc.function[RETURN] = _parameters.rc_map_return_sw - 1;
	_rc.function[ASSISTED] = _parameters.rc_map_assisted_sw - 1;
	_rc.function[MISSION] = _parameters.rc_map_mission_sw - 1;

	_rc.function[FLAPS] = _parameters.rc_map_flaps - 1;

//	_rc.function[OFFBOARD_MODE] = _parameters.rc_map_offboard_ctrl_mode_sw - 1;

	_rc.function[AUX_1] = _parameters.rc_map_aux1 - 1;
	_rc.function[AUX_2] = _parameters.rc_map_aux2 - 1;
	_rc.function[AUX_3] = _parameters.rc_map_aux3 - 1;
	_rc.function[AUX_4] = _parameters.rc_map_aux4 - 1;
	_rc.function[AUX_5] = _parameters.rc_map_aux5 - 1;

	/* gyro offsets */
	param_get(_parameter_handles.gyro_offset[0], &(_parameters.gyro_offset[0]));
	param_get(_parameter_handles.gyro_offset[1], &(_parameters.gyro_offset[1]));
	param_get(_parameter_handles.gyro_offset[2], &(_parameters.gyro_offset[2]));
	param_get(_parameter_handles.gyro_scale[0], &(_parameters.gyro_scale[0]));
	param_get(_parameter_handles.gyro_scale[1], &(_parameters.gyro_scale[1]));
	param_get(_parameter_handles.gyro_scale[2], &(_parameters.gyro_scale[2]));

	/* accel offsets */
	param_get(_parameter_handles.accel_offset[0], &(_parameters.accel_offset[0]));
	param_get(_parameter_handles.accel_offset[1], &(_parameters.accel_offset[1]));
	param_get(_parameter_handles.accel_offset[2], &(_parameters.accel_offset[2]));
	param_get(_parameter_handles.accel_scale[0], &(_parameters.accel_scale[0]));
	param_get(_parameter_handles.accel_scale[1], &(_parameters.accel_scale[1]));
	param_get(_parameter_handles.accel_scale[2], &(_parameters.accel_scale[2]));

	/* mag offsets */
	param_get(_parameter_handles.mag_offset[0], &(_parameters.mag_offset[0]));
	param_get(_parameter_handles.mag_offset[1], &(_parameters.mag_offset[1]));
	param_get(_parameter_handles.mag_offset[2], &(_parameters.mag_offset[2]));
	/* mag scaling */
	param_get(_parameter_handles.mag_scale[0], &(_parameters.mag_scale[0]));
	param_get(_parameter_handles.mag_scale[1], &(_parameters.mag_scale[1]));
	param_get(_parameter_handles.mag_scale[2], &(_parameters.mag_scale[2]));

	/* Airspeed offset */
	param_get(_parameter_handles.diff_pres_offset_pa, &(_parameters.diff_pres_offset_pa));
	param_get(_parameter_handles.diff_pres_analog_enabled, &(_parameters.diff_pres_analog_enabled));

	/* scaling of ADC ticks to battery voltage */
	if (param_get(_parameter_handles.battery_voltage_scaling, &(_parameters.battery_voltage_scaling)) != OK) {
		warnx("Failed updating voltage scaling param");
	}

	param_get(_parameter_handles.board_rotation, &(_parameters.board_rotation));
	param_get(_parameter_handles.external_mag_rotation, &(_parameters.external_mag_rotation));

	get_rot_matrix((enum Rotation)_parameters.board_rotation, &_board_rotation);
	get_rot_matrix((enum Rotation)_parameters.external_mag_rotation, &_external_mag_rotation);

	return OK;
}

void
Sensors::accel_init()
{
	int	fd;

	fd = open(ACCEL_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("%s", ACCEL_DEVICE_PATH);
		errx(1, "FATAL: no accelerometer found");

	} else {

		// XXX do the check more elegantly

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1

		/* set the accel internal sampling rate up to at leat 1000Hz */
		ioctl(fd, ACCELIOCSSAMPLERATE, 1000);

		/* set the driver to poll at 1000Hz */
		ioctl(fd, SENSORIOCSPOLLRATE, 1000);

#elif CONFIG_ARCH_BOARD_PX4FMU_V2

		/* set the accel internal sampling rate up to at leat 800Hz */
		ioctl(fd, ACCELIOCSSAMPLERATE, 800);

		/* set the driver to poll at 800Hz */
		ioctl(fd, SENSORIOCSPOLLRATE, 800);

#else
#error Need a board configuration, either CONFIG_ARCH_BOARD_PX4FMU_V1 or CONFIG_ARCH_BOARD_PX4FMU_V2

#endif

		warnx("using system accel");
		close(fd);
	}
}

void
Sensors::gyro_init()
{
	int	fd;

	fd = open(GYRO_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("%s", GYRO_DEVICE_PATH);
		errx(1, "FATAL: no gyro found");

	} else {

		// XXX do the check more elegantly

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1

		/* set the gyro internal sampling rate up to at least 1000Hz */
		if (ioctl(fd, GYROIOCSSAMPLERATE, 1000) != OK)
			ioctl(fd, GYROIOCSSAMPLERATE, 800);

		/* set the driver to poll at 1000Hz */
		if (ioctl(fd, SENSORIOCSPOLLRATE, 1000) != OK)
			ioctl(fd, SENSORIOCSPOLLRATE, 800);

#else

		/* set the gyro internal sampling rate up to at least 760Hz */
		ioctl(fd, GYROIOCSSAMPLERATE, 760);

		/* set the driver to poll at 760Hz */
		ioctl(fd, SENSORIOCSPOLLRATE, 760);

#endif

		warnx("using system gyro");
		close(fd);
	}
}

void
Sensors::mag_init()
{
	int	fd;
	int	ret;

	fd = open(MAG_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("%s", MAG_DEVICE_PATH);
		errx(1, "FATAL: no magnetometer found");
	}

	/* try different mag sampling rates */


	ret = ioctl(fd, MAGIOCSSAMPLERATE, 150);

	if (ret == OK) {
		/* set the pollrate accordingly */
		ioctl(fd, SENSORIOCSPOLLRATE, 150);

	} else {
		ret = ioctl(fd, MAGIOCSSAMPLERATE, 100);

		/* if the slower sampling rate still fails, something is wrong */
		if (ret == OK) {
			/* set the driver to poll also at the slower rate */
			ioctl(fd, SENSORIOCSPOLLRATE, 100);

		} else {
			errx(1, "FATAL: mag sampling rate could not be set");
		}
	}



	ret = ioctl(fd, MAGIOCGEXTERNAL, 0);

	if (ret < 0)
		errx(1, "FATAL: unknown if magnetometer is external or onboard");
	else if (ret == 1)
		_mag_is_external = true;
	else
		_mag_is_external = false;

	close(fd);
}

void
Sensors::baro_init()
{
	int	fd;

	fd = open(BARO_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("%s", BARO_DEVICE_PATH);
		errx(1, "FATAL: No barometer found");
	}

	/* set the driver to poll at 150Hz */
	ioctl(fd, SENSORIOCSPOLLRATE, 150);

	close(fd);
}

void
Sensors::adc_init()
{

	_fd_adc = open(ADC_DEVICE_PATH, O_RDONLY | O_NONBLOCK);

	if (_fd_adc < 0) {
		warn(ADC_DEVICE_PATH);
		warnx("FATAL: no ADC found");
	}
}

void
Sensors::accel_poll(struct sensor_combined_s &raw)
{
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		struct accel_report	accel_report;

		orb_copy(ORB_ID(sensor_accel), _accel_sub, &accel_report);

		math::Vector3 vect = {accel_report.x, accel_report.y, accel_report.z};
		vect = _board_rotation * vect;

		raw.accelerometer_m_s2[0] = vect(0);
		raw.accelerometer_m_s2[1] = vect(1);
		raw.accelerometer_m_s2[2] = vect(2);

		raw.accelerometer_raw[0] = accel_report.x_raw;
		raw.accelerometer_raw[1] = accel_report.y_raw;
		raw.accelerometer_raw[2] = accel_report.z_raw;

		raw.accelerometer_counter++;
	}
}

void
Sensors::gyro_poll(struct sensor_combined_s &raw)
{
	bool gyro_updated;
	orb_check(_gyro_sub, &gyro_updated);

	if (gyro_updated) {
		struct gyro_report	gyro_report;

		orb_copy(ORB_ID(sensor_gyro), _gyro_sub, &gyro_report);

		math::Vector3 vect = {gyro_report.x, gyro_report.y, gyro_report.z};
		vect = _board_rotation * vect;

		raw.gyro_rad_s[0] = vect(0);
		raw.gyro_rad_s[1] = vect(1);
		raw.gyro_rad_s[2] = vect(2);

		raw.gyro_raw[0] = gyro_report.x_raw;
		raw.gyro_raw[1] = gyro_report.y_raw;
		raw.gyro_raw[2] = gyro_report.z_raw;

		raw.gyro_counter++;
	}
}

void
Sensors::mag_poll(struct sensor_combined_s &raw)
{
	bool mag_updated;
	orb_check(_mag_sub, &mag_updated);

	if (mag_updated) {
		struct mag_report	mag_report;

		orb_copy(ORB_ID(sensor_mag), _mag_sub, &mag_report);

		math::Vector3 vect = {mag_report.x, mag_report.y, mag_report.z};

		if (_mag_is_external)
			vect = _external_mag_rotation * vect;
		else
			vect = _board_rotation * vect;

		raw.magnetometer_ga[0] = vect(0);
		raw.magnetometer_ga[1] = vect(1);
		raw.magnetometer_ga[2] = vect(2);

		raw.magnetometer_raw[0] = mag_report.x_raw;
		raw.magnetometer_raw[1] = mag_report.y_raw;
		raw.magnetometer_raw[2] = mag_report.z_raw;

		raw.magnetometer_counter++;
	}
}

void
Sensors::baro_poll(struct sensor_combined_s &raw)
{
	bool baro_updated;
	orb_check(_baro_sub, &baro_updated);

	if (baro_updated) {

		orb_copy(ORB_ID(sensor_baro), _baro_sub, &_barometer);

		raw.baro_pres_mbar = _barometer.pressure; // Pressure in mbar
		raw.baro_alt_meter = _barometer.altitude; // Altitude in meters
		raw.baro_temp_celcius = _barometer.temperature; // Temperature in degrees celcius

		raw.baro_counter++;
	}
}

void
Sensors::diff_pres_poll(struct sensor_combined_s &raw)
{
	bool updated;
	orb_check(_diff_pres_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(differential_pressure), _diff_pres_sub, &_diff_pres);

		raw.differential_pressure_pa = _diff_pres.differential_pressure_pa;
		raw.differential_pressure_counter++;

		_airspeed.indicated_airspeed_m_s = calc_indicated_airspeed(_diff_pres.differential_pressure_pa);
		_airspeed.true_airspeed_m_s = calc_true_airspeed(_diff_pres.differential_pressure_pa + raw.baro_pres_mbar * 1e2f,
					      raw.baro_pres_mbar * 1e2f, raw.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG);

		/* announce the airspeed if needed, just publish else */
		if (_airspeed_pub > 0) {
			orb_publish(ORB_ID(airspeed), _airspeed_pub, &_airspeed);

		} else {
			_airspeed_pub = orb_advertise(ORB_ID(airspeed), &_airspeed);
		}
	}
}

void
Sensors::vehicle_control_mode_poll()
{
	struct vehicle_control_mode_s vcontrol_mode;
	bool vcontrol_mode_updated;

	/* Check HIL state if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &vcontrol_mode);

		/* switching from non-HIL to HIL mode */
		//printf("[sensors] Vehicle mode: %i \t AND: %i, HIL: %i\n", vstatus.mode, vstatus.mode & VEHICLE_MODE_FLAG_HIL_ENABLED, hil_enabled);
		if (vcontrol_mode.flag_system_hil_enabled && !_hil_enabled) {
			_hil_enabled = true;
			_publishing = false;

			/* switching from HIL to non-HIL mode */

		} else if (!_publishing && !_hil_enabled) {
			_hil_enabled = false;
			_publishing = true;
		}
	}
}

void
Sensors::parameter_update_poll(bool forced)
{
	bool param_updated;

	/* Check if any parameter has changed */
	orb_check(_params_sub, &param_updated);

	if (param_updated || forced) {
		/* read from param to clear updated flag */
		struct parameter_update_s update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &update);

		/* update parameters */
		parameters_update();

		/* update sensor offsets */
		int fd = open(GYRO_DEVICE_PATH, 0);
		struct gyro_scale gscale = {
			_parameters.gyro_offset[0],
			_parameters.gyro_scale[0],
			_parameters.gyro_offset[1],
			_parameters.gyro_scale[1],
			_parameters.gyro_offset[2],
			_parameters.gyro_scale[2],
		};

		if (OK != ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gscale))
			warn("WARNING: failed to set scale / offsets for gyro");

		close(fd);

		fd = open(ACCEL_DEVICE_PATH, 0);
		struct accel_scale ascale = {
			_parameters.accel_offset[0],
			_parameters.accel_scale[0],
			_parameters.accel_offset[1],
			_parameters.accel_scale[1],
			_parameters.accel_offset[2],
			_parameters.accel_scale[2],
		};

		if (OK != ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&ascale))
			warn("WARNING: failed to set scale / offsets for accel");

		close(fd);

		fd = open(MAG_DEVICE_PATH, 0);
		struct mag_scale mscale = {
			_parameters.mag_offset[0],
			_parameters.mag_scale[0],
			_parameters.mag_offset[1],
			_parameters.mag_scale[1],
			_parameters.mag_offset[2],
			_parameters.mag_scale[2],
		};

		if (OK != ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale))
			warn("WARNING: failed to set scale / offsets for mag");

		close(fd);

		fd = open(AIRSPEED_DEVICE_PATH, 0);

		/* this sensor is optional, abort without error */

		if (fd > 0) {
			struct airspeed_scale airscale = {
				_parameters.diff_pres_offset_pa,
				1.0f,
			};

			if (OK != ioctl(fd, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale))
				warn("WARNING: failed to set scale / offsets for airspeed sensor");
			close(fd);
		}

#if 0
		printf("CH0: RAW MAX: %d MIN %d S: %d MID: %d FUNC: %d\n", (int)_parameters.max[0], (int)_parameters.min[0], (int)(_rc.chan[0].scaling_factor * 10000), (int)(_rc.chan[0].mid), (int)_rc.function[0]);
		printf("CH1: RAW MAX: %d MIN %d S: %d MID: %d FUNC: %d\n", (int)_parameters.max[1], (int)_parameters.min[1], (int)(_rc.chan[1].scaling_factor * 10000), (int)(_rc.chan[1].mid), (int)_rc.function[1]);
		printf("MAN: %d %d\n", (int)(_rc.chan[0].scaled * 100), (int)(_rc.chan[1].scaled * 100));
		fflush(stdout);
		usleep(5000);
#endif
	}
}

void
Sensors::adc_poll(struct sensor_combined_s &raw)
{
	/* only read if publishing */
	if (!_publishing)
		return;

	/* rate limit to 100 Hz */
	if (hrt_absolute_time() - _last_adc >= 10000) {
		/* make space for a maximum of eight channels */
		struct adc_msg_s buf_adc[8];
		/* read all channels available */
		int ret = read(_fd_adc, &buf_adc, sizeof(buf_adc));

		for (unsigned i = 0; i < sizeof(buf_adc) / sizeof(buf_adc[0]); i++) {

			if (ret >= (int)sizeof(buf_adc[0])) {

				/* Save raw voltage values */
				if (i < (sizeof(raw.adc_voltage_v)) / sizeof(raw.adc_voltage_v[0])) {
					raw.adc_voltage_v[i] = buf_adc[i].am_data / (4096.0f / 3.3f);
				}

				/* look for specific channels and process the raw voltage to measurement data */
				if (ADC_BATTERY_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {
					/* Voltage in volts */
					float voltage = (buf_adc[i].am_data * _parameters.battery_voltage_scaling);

					if (voltage > VOLTAGE_BATTERY_IGNORE_THRESHOLD_VOLTS) {

						/* one-time initialization of low-pass value to avoid long init delays */
						if (_battery_status.voltage_v < 3.0f) {
							_battery_status.voltage_v = voltage;
						}

						_battery_status.timestamp = hrt_absolute_time();
						_battery_status.voltage_v = (BAT_VOL_LOWPASS_1 * (_battery_status.voltage_v + BAT_VOL_LOWPASS_2 * voltage));;
						/* current and discharge are unknown */
						_battery_status.current_a = -1.0f;
						_battery_status.discharged_mah = -1.0f;

						/* announce the battery voltage if needed, just publish else */
						if (_battery_pub > 0) {
							orb_publish(ORB_ID(battery_status), _battery_pub, &_battery_status);

						} else {
							_battery_pub = orb_advertise(ORB_ID(battery_status), &_battery_status);
						}
					}

				} else if (ADC_AIRSPEED_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {

					/* calculate airspeed, raw is the difference from */
					float voltage = (float)(buf_adc[i].am_data) * 3.3f / 4096.0f * 2.0f;  //V_ref/4096 * (voltage divider factor)

					/**
					 * The voltage divider pulls the signal down, only act on
					 * a valid voltage from a connected sensor. Also assume a non-
					 * zero offset from the sensor if its connected.
					 */
					if (voltage > 0.4f && _parameters.diff_pres_analog_enabled) {

						float diff_pres_pa = voltage * 1000.0f - _parameters.diff_pres_offset_pa; //for MPXV7002DP sensor

						_diff_pres.timestamp = hrt_absolute_time();
						_diff_pres.differential_pressure_pa = diff_pres_pa;
						_diff_pres.voltage = voltage;

						/* announce the airspeed if needed, just publish else */
						if (_diff_pres_pub > 0) {
							orb_publish(ORB_ID(differential_pressure), _diff_pres_pub, &_diff_pres);

						} else {
							_diff_pres_pub = orb_advertise(ORB_ID(differential_pressure), &_diff_pres);
						}
					}
				}

				_last_adc = hrt_absolute_time();
			}
		}
	}
}

void
Sensors::rc_poll()
{
	bool rc_updated;
	orb_check(_rc_sub, &rc_updated);

	if (rc_updated) {
		/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
		struct rc_input_values	rc_input;

		orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);

		struct manual_control_setpoint_s manual_control;
		struct actuator_controls_s actuator_group_3;

		/* initialize to default values */
		manual_control.roll = NAN;
		manual_control.pitch = NAN;
		manual_control.yaw = NAN;
		manual_control.throttle = NAN;

		manual_control.mode_switch = NAN;
		manual_control.return_switch = NAN;
		manual_control.assisted_switch = NAN;
		manual_control.mission_switch = NAN;
//		manual_control.auto_offboard_input_switch = NAN;

		manual_control.flaps = NAN;
		manual_control.aux1 = NAN;
		manual_control.aux2 = NAN;
		manual_control.aux3 = NAN;
		manual_control.aux4 = NAN;
		manual_control.aux5 = NAN;

		/* require at least four channels to consider the signal valid */
		if (rc_input.channel_count < 4)
			return;

		/* failsafe check */
		if (_parameters.rc_fs_ch != 0) {
			if (_parameters.rc_fs_mode == 0) {
				if (rc_input.values[_parameters.rc_fs_ch - 1] < _parameters.rc_fs_thr)
					return;

			} else if (_parameters.rc_fs_mode == 1) {
				if (rc_input.values[_parameters.rc_fs_ch - 1] > _parameters.rc_fs_thr)
					return;
			}
		}

		unsigned channel_limit = rc_input.channel_count;

		if (channel_limit > _rc_max_chan_count)
			channel_limit = _rc_max_chan_count;

		/* we are accepting this message */
		_rc_last_valid = rc_input.timestamp;

		/* Read out values from raw message */
		for (unsigned int i = 0; i < channel_limit; i++) {

			/*
			 * 1) Constrain to min/max values, as later processing depends on bounds.
			 */
			if (rc_input.values[i] < _parameters.min[i])
				rc_input.values[i] = _parameters.min[i];

			if (rc_input.values[i] > _parameters.max[i])
				rc_input.values[i] = _parameters.max[i];

			/*
			 * 2) Scale around the mid point differently for lower and upper range.
			 *
			 * This is necessary as they don't share the same endpoints and slope.
			 *
			 * First normalize to 0..1 range with correct sign (below or above center),
			 * the total range is 2 (-1..1).
			 * If center (trim) == min, scale to 0..1, if center (trim) == max,
			 * scale to -1..0.
			 *
			 * As the min and max bounds were enforced in step 1), division by zero
			 * cannot occur, as for the case of center == min or center == max the if
			 * statement is mutually exclusive with the arithmetic NaN case.
			 *
			 * DO NOT REMOVE OR ALTER STEP 1!
			 */
			if (rc_input.values[i] > (_parameters.trim[i] + _parameters.dz[i])) {
				_rc.chan[i].scaled = (rc_input.values[i] - _parameters.trim[i] - _parameters.dz[i]) / (float)(_parameters.max[i] - _parameters.trim[i] - _parameters.dz[i]);

			} else if (rc_input.values[i] < (_parameters.trim[i] - _parameters.dz[i])) {
				_rc.chan[i].scaled = (rc_input.values[i] - _parameters.trim[i] + _parameters.dz[i]) / (float)(_parameters.trim[i] - _parameters.min[i] - _parameters.dz[i]);

			} else {
				/* in the configured dead zone, output zero */
				_rc.chan[i].scaled = 0.0f;
			}

			_rc.chan[i].scaled *= _parameters.rev[i];

			/* handle any parameter-induced blowups */
			if (!isfinite(_rc.chan[i].scaled))
				_rc.chan[i].scaled = 0.0f;
		}

		_rc.chan_count = rc_input.channel_count;
		_rc.timestamp = rc_input.timestamp;

		manual_control.timestamp = rc_input.timestamp;

		/* roll input - rolling right is stick-wise and rotation-wise positive */
		manual_control.roll = limit_minus_one_to_one(_rc.chan[_rc.function[ROLL]].scaled);
		/*
		 * pitch input - stick down is negative, but stick down is pitching up (pos) in NED,
		 * so reverse sign.
		 */
		manual_control.pitch = limit_minus_one_to_one(-1.0f * _rc.chan[_rc.function[PITCH]].scaled);
		/* yaw input - stick right is positive and positive rotation */
		manual_control.yaw = limit_minus_one_to_one(_rc.chan[_rc.function[YAW]].scaled);
		/* throttle input */
		manual_control.throttle = _rc.chan[_rc.function[THROTTLE]].scaled;

		if (manual_control.throttle < 0.0f) manual_control.throttle = 0.0f;

		if (manual_control.throttle > 1.0f) manual_control.throttle = 1.0f;

		/* scale output */
		if (isfinite(_parameters.rc_scale_roll) && _parameters.rc_scale_roll > 0.0f) {
			manual_control.roll *= _parameters.rc_scale_roll;
		}

		if (isfinite(_parameters.rc_scale_pitch) && _parameters.rc_scale_pitch > 0.0f) {
			manual_control.pitch *= _parameters.rc_scale_pitch;
		}

		if (isfinite(_parameters.rc_scale_yaw) && _parameters.rc_scale_yaw > 0.0f) {
			manual_control.yaw *= _parameters.rc_scale_yaw;
		}

		/* flaps */
		if (_rc.function[FLAPS] >= 0) {

			manual_control.flaps = limit_minus_one_to_one(_rc.chan[_rc.function[FLAPS]].scaled);

			if (isfinite(_parameters.rc_scale_flaps) && _parameters.rc_scale_flaps > 0.0f) {
				manual_control.flaps *= _parameters.rc_scale_flaps;
			}
		}

		/* mode switch input */
		if (_rc.function[MODE] >= 0) {
			manual_control.mode_switch = limit_minus_one_to_one(_rc.chan[_rc.function[MODE]].scaled);
		}

		/* assisted switch input */
		if (_rc.function[ASSISTED] >= 0) {
			manual_control.assisted_switch = limit_minus_one_to_one(_rc.chan[_rc.function[ASSISTED]].scaled);
		}

		/* mission switch input */
		if (_rc.function[MISSION] >= 0) {
			manual_control.mission_switch = limit_minus_one_to_one(_rc.chan[_rc.function[MISSION]].scaled);
		}

		/* return switch input */
		if (_rc.function[RETURN] >= 0) {
			manual_control.return_switch = limit_minus_one_to_one(_rc.chan[_rc.function[RETURN]].scaled);
		}

//		if (_rc.function[OFFBOARD_MODE] >= 0) {
//			manual_control.auto_offboard_input_switch = limit_minus_one_to_one(_rc.chan[_rc.function[OFFBOARD_MODE]].scaled);
//		}

		/* aux functions, only assign if valid mapping is present */
		if (_rc.function[AUX_1] >= 0) {
			manual_control.aux1 = limit_minus_one_to_one(_rc.chan[_rc.function[AUX_1]].scaled);
		}

		if (_rc.function[AUX_2] >= 0) {
			manual_control.aux2 = limit_minus_one_to_one(_rc.chan[_rc.function[AUX_2]].scaled);
		}

		if (_rc.function[AUX_3] >= 0) {
			manual_control.aux3 = limit_minus_one_to_one(_rc.chan[_rc.function[AUX_3]].scaled);
		}

		if (_rc.function[AUX_4] >= 0) {
			manual_control.aux4 = limit_minus_one_to_one(_rc.chan[_rc.function[AUX_4]].scaled);
		}

		if (_rc.function[AUX_5] >= 0) {
			manual_control.aux5 = limit_minus_one_to_one(_rc.chan[_rc.function[AUX_5]].scaled);
		}

		/* copy from mapped manual control to control group 3 */
		actuator_group_3.control[0] = manual_control.roll;
		actuator_group_3.control[1] = manual_control.pitch;
		actuator_group_3.control[2] = manual_control.yaw;
		actuator_group_3.control[3] = manual_control.throttle;
		actuator_group_3.control[4] = manual_control.flaps;
		actuator_group_3.control[5] = manual_control.aux1;
		actuator_group_3.control[6] = manual_control.aux2;
		actuator_group_3.control[7] = manual_control.aux3;

		/* check if ready for publishing */
		if (_rc_pub > 0) {
			orb_publish(ORB_ID(rc_channels), _rc_pub, &_rc);

		} else {
			/* advertise the rc topic */
			_rc_pub = orb_advertise(ORB_ID(rc_channels), &_rc);
		}

		/* check if ready for publishing */
		if (_manual_control_pub > 0) {
			orb_publish(ORB_ID(manual_control_setpoint), _manual_control_pub, &manual_control);

		} else {
			_manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual_control);
		}

		/* check if ready for publishing */
		if (_actuator_group_3_pub > 0) {
			orb_publish(ORB_ID(actuator_controls_3), _actuator_group_3_pub, &actuator_group_3);

		} else {
			_actuator_group_3_pub = orb_advertise(ORB_ID(actuator_controls_3), &actuator_group_3);
		}
	}

}

void
Sensors::task_main_trampoline(int argc, char *argv[])
{
	sensors::g_sensors->task_main();
}

void
Sensors::task_main()
{

	/* inform about start */
	warnx("Initializing..");

	/* start individual sensors */
	accel_init();
	gyro_init();
	mag_init();
	baro_init();
	adc_init();

	/*
	 * do subscriptions
	 */
	_gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
	_accel_sub = orb_subscribe(ORB_ID(sensor_accel));
	_mag_sub = orb_subscribe(ORB_ID(sensor_mag));
	_rc_sub = orb_subscribe(ORB_ID(input_rc));
	_baro_sub = orb_subscribe(ORB_ID(sensor_baro));
	_diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vcontrol_mode_sub, 200);

	/* rate limit gyro to 250 Hz (the gyro signal is lowpassed accordingly earlier) */
	orb_set_interval(_gyro_sub, 4);

	/*
	 * do advertisements
	 */
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	raw.timestamp = hrt_absolute_time();
	raw.adc_voltage_v[0] = 0.0f;
	raw.adc_voltage_v[1] = 0.0f;
	raw.adc_voltage_v[2] = 0.0f;
	raw.adc_voltage_v[3] = 0.0f;

	memset(&_battery_status, 0, sizeof(_battery_status));
	_battery_status.voltage_v = BAT_VOL_INITIAL;

	/* get a set of initial values */
	accel_poll(raw);
	gyro_poll(raw);
	mag_poll(raw);
	baro_poll(raw);
	diff_pres_poll(raw);

	parameter_update_poll(true /* forced */);

	/* advertise the sensor_combined topic and make the initial publication */
	_sensor_pub = orb_advertise(ORB_ID(sensor_combined), &raw);

	/* wakeup source(s) */
	struct pollfd fds[1];

	/* use the gyro to pace output - XXX BROKEN if we are using the L3GD20 */
	fds[0].fd = _gyro_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
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
		vehicle_control_mode_poll();

		/* check parameters for updates */
		parameter_update_poll();

		/* store the time closest to all measurements (this is bogus, sensor timestamps should be propagated...) */
		raw.timestamp = hrt_absolute_time();

		/* copy most recent sensor data */
		gyro_poll(raw);
		accel_poll(raw);
		mag_poll(raw);
		baro_poll(raw);

		/* check battery voltage */
		adc_poll(raw);

		diff_pres_poll(raw);

		/* Inform other processes that new data is available to copy */
		if (_publishing)
			orb_publish(ORB_ID(sensor_combined), _sensor_pub, &raw);

		/* Look for new r/c input data */
		rc_poll();

		perf_end(_loop_perf);
	}

	printf("[sensors] exiting.\n");

	_sensors_task = -1;
	_exit(0);
}

int
Sensors::start()
{
	ASSERT(_sensors_task == -1);

	/* start the task */
	_sensors_task = task_spawn_cmd("sensors_task",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2048,
				       (main_t)&Sensors::task_main_trampoline,
				       nullptr);

	if (_sensors_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int sensors_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: sensors {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (sensors::g_sensors != nullptr)
			errx(0, "sensors task already running");

		sensors::g_sensors = new Sensors;

		if (sensors::g_sensors == nullptr)
			errx(1, "sensors task alloc failed");

		if (OK != sensors::g_sensors->start()) {
			delete sensors::g_sensors;
			sensors::g_sensors = nullptr;
			err(1, "sensors task start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (sensors::g_sensors == nullptr)
			errx(1, "sensors task not running");

		delete sensors::g_sensors;
		sensors::g_sensors = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (sensors::g_sensors) {
			errx(0, "task is running");

		} else {
			errx(1, "task is not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}

