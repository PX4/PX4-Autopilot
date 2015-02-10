/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
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
#include <drivers/drv_px4flow.h>

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
#include <uORB/topics/rc_parameter_map.h>

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
#define ADC_BATTERY_CURRENT_CHANNEL	-1
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	11
#endif

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
#define ADC_BATTERY_VOLTAGE_CHANNEL	2
#define ADC_BATTERY_CURRENT_CHANNEL	3
#define ADC_5V_RAIL_SENSE		4
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	15
#endif

#ifdef CONFIG_ARCH_BOARD_AEROCORE
#define ADC_BATTERY_VOLTAGE_CHANNEL	10
#define ADC_BATTERY_CURRENT_CHANNEL	-1
#define ADC_AIRSPEED_VOLTAGE_CHANNEL	-1
#endif

#define BATT_V_LOWPASS 0.001f
#define BATT_V_IGNORE_THRESHOLD 4.8f

/**
 * HACK - true temperature is much less than indicated temperature in baro,
 * subtract 5 degrees in an attempt to account for the electrical upheating of the PCB
 */
#define PCB_TEMP_ESTIMATE_DEG 5.0f

#define STICK_ON_OFF_LIMIT 0.75f

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define CAL_FAILED_APPLY_CAL_MSG "FAILED APPLYING SENSOR CAL"

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

	/**
	 * Get and limit value for specified RC function. Returns NAN if not mapped.
	 */
	float		get_rc_value(uint8_t func, float min_value, float max_value);

	/**
	 * Get switch position for specified function.
	 */
	switch_pos_t	get_rc_sw3pos_position(uint8_t func, float on_th, bool on_inv, float mid_th, bool mid_inv);
	switch_pos_t	get_rc_sw2pos_position(uint8_t func, float on_th, bool on_inv);

	/**
	 * Update paramters from RC channels if the functionality is activated and the
	 * input has changed since the last update
	 *
	 * @param
	 */
	void set_params_from_rc();

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

	int		_gyro_sub;			/**< raw gyro0 data subscription */
	int		_accel_sub;			/**< raw accel0 data subscription */
	int		_mag_sub;			/**< raw mag0 data subscription */
	int		_gyro1_sub;			/**< raw gyro1 data subscription */
	int		_accel1_sub;			/**< raw accel1 data subscription */
	int		_mag1_sub;			/**< raw mag1 data subscription */
	int		_gyro2_sub;			/**< raw gyro2 data subscription */
	int		_accel2_sub;			/**< raw accel2 data subscription */
	int		_mag2_sub;			/**< raw mag2 data subscription */
	int 		_rc_sub;			/**< raw rc channels data subscription */
	int		_baro_sub;			/**< raw baro data subscription */
	int		_baro1_sub;			/**< raw baro data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_diff_pres_sub;			/**< raw differential pressure subscription */
	int		_vcontrol_mode_sub;			/**< vehicle control mode subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int		_rc_parameter_map_sub;			/**< rc parameter map subscription */
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
	struct rc_parameter_map_s _rc_parameter_map;

	math::Matrix<3, 3>	_board_rotation;		/**< rotation matrix for the orientation that the board is mounted */
	math::Matrix<3, 3>	_external_mag_rotation;		/**< rotation matrix for the orientation that an external mag is mounted */
	bool		_mag_is_external;		/**< true if the active mag is on an external board */

	uint64_t _battery_discharged;			/**< battery discharged current in mA*ms */
	hrt_abstime _battery_current_timestamp;	/**< timestamp of last battery current reading */

	struct {
		float min[_rc_max_chan_count];
		float trim[_rc_max_chan_count];
		float max[_rc_max_chan_count];
		float rev[_rc_max_chan_count];
		float dz[_rc_max_chan_count];
		float scaling_factor[_rc_max_chan_count];

		float diff_pres_offset_pa;
		float diff_pres_analog_scale;

		int board_rotation;
		int flow_rotation;
		int external_mag_rotation;

		float board_offset[3];

		int rc_map_roll;
		int rc_map_pitch;
		int rc_map_yaw;
		int rc_map_throttle;
		int rc_map_failsafe;

		int rc_map_mode_sw;
		int rc_map_return_sw;
		int rc_map_posctl_sw;
		int rc_map_loiter_sw;
		int rc_map_acro_sw;
		int rc_map_offboard_sw;

		int rc_map_flaps;

		int rc_map_aux1;
		int rc_map_aux2;
		int rc_map_aux3;
		int rc_map_aux4;
		int rc_map_aux5;

		int rc_map_param[RC_PARAM_MAP_NCHAN];

		int32_t rc_fails_thr;
		float rc_assist_th;
		float rc_auto_th;
		float rc_posctl_th;
		float rc_return_th;
		float rc_loiter_th;
		float rc_acro_th;
		float rc_offboard_th;
		bool rc_assist_inv;
		bool rc_auto_inv;
		bool rc_posctl_inv;
		bool rc_return_inv;
		bool rc_loiter_inv;
		bool rc_acro_inv;
		bool rc_offboard_inv;

		float battery_voltage_scaling;
		float battery_current_scaling;

		float baro_qnh;

	}		_parameters;			/**< local copies of interesting parameters */

	struct {
		param_t min[_rc_max_chan_count];
		param_t trim[_rc_max_chan_count];
		param_t max[_rc_max_chan_count];
		param_t rev[_rc_max_chan_count];
		param_t dz[_rc_max_chan_count];

		param_t diff_pres_offset_pa;
		param_t diff_pres_analog_scale;

		param_t rc_map_roll;
		param_t rc_map_pitch;
		param_t rc_map_yaw;
		param_t rc_map_throttle;
		param_t rc_map_failsafe;

		param_t rc_map_mode_sw;
		param_t rc_map_return_sw;
		param_t rc_map_posctl_sw;
		param_t rc_map_loiter_sw;
		param_t rc_map_acro_sw;
		param_t rc_map_offboard_sw;

		param_t rc_map_flaps;

		param_t rc_map_aux1;
		param_t rc_map_aux2;
		param_t rc_map_aux3;
		param_t rc_map_aux4;
		param_t rc_map_aux5;

		param_t rc_map_param[RC_PARAM_MAP_NCHAN];
		param_t rc_param[RC_PARAM_MAP_NCHAN];	/**< param handles for the paramters which are bound
							  to a RC channel, equivalent float values in the
							  _parameters struct are not existing
							  because these parameters are never read. */

		param_t rc_fails_thr;
		param_t rc_assist_th;
		param_t rc_auto_th;
		param_t rc_posctl_th;
		param_t rc_return_th;
		param_t rc_loiter_th;
		param_t rc_acro_th;
		param_t rc_offboard_th;

		param_t battery_voltage_scaling;
		param_t battery_current_scaling;

		param_t board_rotation;
		param_t flow_rotation;
		param_t external_mag_rotation;

		param_t board_offset[3];

		param_t baro_qnh;

	}		_parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Do accel-related initialisation.
	 */
	int		accel_init();

	/**
	 * Do gyro-related initialisation.
	 */
	int		gyro_init();

	/**
	 * Do mag-related initialisation.
	 */
	int		mag_init();

	/**
	 * Do baro-related initialisation.
	 */
	int		baro_init();

	/**
	 * Do adc-related initialisation.
	 */
	int		adc_init();

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
	 * Check for changes in rc_parameter_map
	 */
	void 		rc_parameter_map_poll(bool forced = false);

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
	void		task_main();
};

namespace sensors
{

Sensors	*g_sensors = nullptr;
}

Sensors::Sensors() :
	_fd_adc(-1),
	_last_adc(0),

	_task_should_exit(true),
	_sensors_task(-1),
	_hil_enabled(false),
	_publishing(true),

	/* subscriptions */
	_gyro_sub(-1),
	_accel_sub(-1),
	_mag_sub(-1),
	_gyro1_sub(-1),
	_accel1_sub(-1),
	_mag1_sub(-1),
	_gyro2_sub(-1),
	_accel2_sub(-1),
	_mag2_sub(-1),
	_rc_sub(-1),
	_baro_sub(-1),
	_baro1_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_rc_parameter_map_sub(-1),
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

	_mag_is_external(false),
	_battery_discharged(0),
	_battery_current_timestamp(0)
{
	memset(&_rc, 0, sizeof(_rc));
	memset(&_diff_pres, 0, sizeof(_diff_pres));
	memset(&_rc_parameter_map, 0, sizeof(_rc_parameter_map));

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
	_parameter_handles.rc_map_failsafe = param_find("RC_MAP_FAILSAFE");

	/* mandatory mode switches, mapped to channel 5 and 6 per default */
	_parameter_handles.rc_map_mode_sw = param_find("RC_MAP_MODE_SW");
	_parameter_handles.rc_map_return_sw = param_find("RC_MAP_RETURN_SW");

	_parameter_handles.rc_map_flaps = param_find("RC_MAP_FLAPS");

	/* optional mode switches, not mapped per default */
	_parameter_handles.rc_map_posctl_sw = param_find("RC_MAP_POSCTL_SW");
	_parameter_handles.rc_map_loiter_sw = param_find("RC_MAP_LOITER_SW");
	_parameter_handles.rc_map_acro_sw = param_find("RC_MAP_ACRO_SW");
	_parameter_handles.rc_map_offboard_sw = param_find("RC_MAP_OFFB_SW");

	_parameter_handles.rc_map_aux1 = param_find("RC_MAP_AUX1");
	_parameter_handles.rc_map_aux2 = param_find("RC_MAP_AUX2");
	_parameter_handles.rc_map_aux3 = param_find("RC_MAP_AUX3");
	_parameter_handles.rc_map_aux4 = param_find("RC_MAP_AUX4");
	_parameter_handles.rc_map_aux5 = param_find("RC_MAP_AUX5");

	/* RC to parameter mapping for changing parameters with RC */
	for (int i = 0; i < RC_PARAM_MAP_NCHAN; i++) {
		char name[PARAM_ID_LEN];
		snprintf(name, PARAM_ID_LEN, "RC_MAP_PARAM%d", i + 1); // shifted by 1 because param name starts at 1
		_parameter_handles.rc_map_param[i] = param_find(name);
	}

	/* RC thresholds */
	_parameter_handles.rc_fails_thr = param_find("RC_FAILS_THR");
	_parameter_handles.rc_assist_th = param_find("RC_ASSIST_TH");
	_parameter_handles.rc_auto_th = param_find("RC_AUTO_TH");
	_parameter_handles.rc_posctl_th = param_find("RC_POSCTL_TH");
	_parameter_handles.rc_return_th = param_find("RC_RETURN_TH");
	_parameter_handles.rc_loiter_th = param_find("RC_LOITER_TH");
	_parameter_handles.rc_acro_th = param_find("RC_ACRO_TH");
	_parameter_handles.rc_offboard_th = param_find("RC_OFFB_TH");

	/* Differential pressure offset */
	_parameter_handles.diff_pres_offset_pa = param_find("SENS_DPRES_OFF");
	_parameter_handles.diff_pres_analog_scale = param_find("SENS_DPRES_ANSC");

	_parameter_handles.battery_voltage_scaling = param_find("BAT_V_SCALING");
	_parameter_handles.battery_current_scaling = param_find("BAT_C_SCALING");

	/* rotations */
	_parameter_handles.board_rotation = param_find("SENS_BOARD_ROT");
	_parameter_handles.flow_rotation = param_find("SENS_FLOW_ROT");
	_parameter_handles.external_mag_rotation = param_find("SENS_EXT_MAG_ROT");

	/* rotation offsets */
	_parameter_handles.board_offset[0] = param_find("SENS_BOARD_X_OFF");
	_parameter_handles.board_offset[1] = param_find("SENS_BOARD_Y_OFF");
	_parameter_handles.board_offset[2] = param_find("SENS_BOARD_Z_OFF");

	/* Barometer QNH */
	_parameter_handles.baro_qnh = param_find("SENS_BARO_QNH");

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
			warnx("RC chan %u not sane, scaling: %8.6f, rev: %d", i, (double)tmpScaleFactor, (int)(_parameters.rev[i]));
			/* scaling factors do not make sense, lock them down */
			_parameters.scaling_factor[i] = 0.0f;
			rc_valid = false;

		} else {
			_parameters.scaling_factor[i] = tmpScaleFactor;
		}
	}

	/* handle wrong values */
	if (!rc_valid) {
		warnx("WARNING     WARNING     WARNING\n\nRC CALIBRATION NOT SANE!\n\n");
	}

	const char *paramerr = "FAIL PARM LOAD";

	/* channel mapping */
	if (param_get(_parameter_handles.rc_map_roll, &(_parameters.rc_map_roll)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_pitch, &(_parameters.rc_map_pitch)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_yaw, &(_parameters.rc_map_yaw)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_throttle, &(_parameters.rc_map_throttle)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_failsafe, &(_parameters.rc_map_failsafe)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_mode_sw, &(_parameters.rc_map_mode_sw)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_return_sw, &(_parameters.rc_map_return_sw)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_posctl_sw, &(_parameters.rc_map_posctl_sw)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_loiter_sw, &(_parameters.rc_map_loiter_sw)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_acro_sw, &(_parameters.rc_map_acro_sw)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_offboard_sw, &(_parameters.rc_map_offboard_sw)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_flaps, &(_parameters.rc_map_flaps)) != OK) {
		warnx("%s", paramerr);
	}

	param_get(_parameter_handles.rc_map_aux1, &(_parameters.rc_map_aux1));
	param_get(_parameter_handles.rc_map_aux2, &(_parameters.rc_map_aux2));
	param_get(_parameter_handles.rc_map_aux3, &(_parameters.rc_map_aux3));
	param_get(_parameter_handles.rc_map_aux4, &(_parameters.rc_map_aux4));
	param_get(_parameter_handles.rc_map_aux5, &(_parameters.rc_map_aux5));

	for (int i = 0; i < RC_PARAM_MAP_NCHAN; i++) {
		param_get(_parameter_handles.rc_map_param[i], &(_parameters.rc_map_param[i]));
	}

	param_get(_parameter_handles.rc_fails_thr, &(_parameters.rc_fails_thr));
	param_get(_parameter_handles.rc_assist_th, &(_parameters.rc_assist_th));
	_parameters.rc_assist_inv = (_parameters.rc_assist_th < 0);
	_parameters.rc_assist_th = fabs(_parameters.rc_assist_th);
	param_get(_parameter_handles.rc_auto_th, &(_parameters.rc_auto_th));
	_parameters.rc_auto_inv = (_parameters.rc_auto_th < 0);
	_parameters.rc_auto_th = fabs(_parameters.rc_auto_th);
	param_get(_parameter_handles.rc_posctl_th, &(_parameters.rc_posctl_th));
	_parameters.rc_posctl_inv = (_parameters.rc_posctl_th < 0);
	_parameters.rc_posctl_th = fabs(_parameters.rc_posctl_th);
	param_get(_parameter_handles.rc_return_th, &(_parameters.rc_return_th));
	_parameters.rc_return_inv = (_parameters.rc_return_th < 0);
	_parameters.rc_return_th = fabs(_parameters.rc_return_th);
	param_get(_parameter_handles.rc_loiter_th, &(_parameters.rc_loiter_th));
	_parameters.rc_loiter_inv = (_parameters.rc_loiter_th < 0);
	_parameters.rc_loiter_th = fabs(_parameters.rc_loiter_th);
	param_get(_parameter_handles.rc_acro_th, &(_parameters.rc_acro_th));
	_parameters.rc_acro_inv = (_parameters.rc_acro_th < 0);
	_parameters.rc_acro_th = fabs(_parameters.rc_acro_th);
	param_get(_parameter_handles.rc_offboard_th, &(_parameters.rc_offboard_th));
	_parameters.rc_offboard_inv = (_parameters.rc_offboard_th < 0);
	_parameters.rc_offboard_th = fabs(_parameters.rc_offboard_th);

	/* update RC function mappings */
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE] = _parameters.rc_map_throttle - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ROLL] = _parameters.rc_map_roll - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PITCH] = _parameters.rc_map_pitch - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_YAW] = _parameters.rc_map_yaw - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_MODE] = _parameters.rc_map_mode_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RETURN] = _parameters.rc_map_return_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL] = _parameters.rc_map_posctl_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_LOITER] = _parameters.rc_map_loiter_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ACRO] = _parameters.rc_map_acro_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD] = _parameters.rc_map_offboard_sw - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS] = _parameters.rc_map_flaps - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1] = _parameters.rc_map_aux1 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2] = _parameters.rc_map_aux2 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3] = _parameters.rc_map_aux3 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4] = _parameters.rc_map_aux4 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5] = _parameters.rc_map_aux5 - 1;

	for (int i = 0; i < RC_PARAM_MAP_NCHAN; i++) {
		_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] = _parameters.rc_map_param[i] - 1;
	}

	/* Airspeed offset */
	param_get(_parameter_handles.diff_pres_offset_pa, &(_parameters.diff_pres_offset_pa));
	param_get(_parameter_handles.diff_pres_analog_scale, &(_parameters.diff_pres_analog_scale));

	/* scaling of ADC ticks to battery voltage */
	if (param_get(_parameter_handles.battery_voltage_scaling, &(_parameters.battery_voltage_scaling)) != OK) {
		warnx("%s", paramerr);
	}

	/* scaling of ADC ticks to battery current */
	if (param_get(_parameter_handles.battery_current_scaling, &(_parameters.battery_current_scaling)) != OK) {
		warnx("%s", paramerr);
	}

	param_get(_parameter_handles.board_rotation, &(_parameters.board_rotation));
	param_get(_parameter_handles.flow_rotation, &(_parameters.flow_rotation));
	param_get(_parameter_handles.external_mag_rotation, &(_parameters.external_mag_rotation));

	/* set px4flow rotation */
	int	flowfd;
	flowfd = open(PX4FLOW0_DEVICE_PATH, 0);

	if (flowfd >= 0) {
		int flowret = ioctl(flowfd, SENSORIOCSROTATION, _parameters.flow_rotation);

		if (flowret) {
			warnx("flow rotation could not be set");
			close(flowfd);
			return ERROR;
		}

		close(flowfd);
	}

	get_rot_matrix((enum Rotation)_parameters.board_rotation, &_board_rotation);
	get_rot_matrix((enum Rotation)_parameters.external_mag_rotation, &_external_mag_rotation);

	param_get(_parameter_handles.board_offset[0], &(_parameters.board_offset[0]));
	param_get(_parameter_handles.board_offset[1], &(_parameters.board_offset[1]));
	param_get(_parameter_handles.board_offset[2], &(_parameters.board_offset[2]));

	/** fine tune board offset on parameter update **/
	math::Matrix<3, 3> board_rotation_offset;
	board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _parameters.board_offset[0],
					 M_DEG_TO_RAD_F * _parameters.board_offset[1],
					 M_DEG_TO_RAD_F * _parameters.board_offset[2]);

	_board_rotation = _board_rotation * board_rotation_offset;

	/* update barometer qnh setting */
	param_get(_parameter_handles.baro_qnh, &(_parameters.baro_qnh));
	int	barofd;
	barofd = open(BARO0_DEVICE_PATH, 0);

	if (barofd < 0) {
		warnx("ERROR: no barometer foundon %s", BARO0_DEVICE_PATH);
		return ERROR;

	} else {
		int baroret = ioctl(barofd, BAROIOCSMSLPRESSURE, (unsigned long)(_parameters.baro_qnh * 100));

		if (baroret) {
			warnx("qnh could not be set");
			close(barofd);
			return ERROR;
		}

		close(barofd);
	}

	return OK;
}

int
Sensors::accel_init()
{
	int	fd;

	fd = open(ACCEL0_DEVICE_PATH, 0);

	if (fd < 0) {
		warnx("FATAL: no accelerometer found: %s", ACCEL0_DEVICE_PATH);
		return ERROR;

	} else {

		/* set the accel internal sampling rate to default rate */
		ioctl(fd, ACCELIOCSSAMPLERATE, ACCEL_SAMPLERATE_DEFAULT);

		/* set the driver to poll at default rate */
		ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);

		close(fd);
	}

	return OK;
}

int
Sensors::gyro_init()
{
	int	fd;

	fd = open(GYRO0_DEVICE_PATH, 0);

	if (fd < 0) {
		warnx("FATAL: no gyro found: %s", GYRO0_DEVICE_PATH);
		return ERROR;

	} else {

		/* set the gyro internal sampling rate to default rate */
		ioctl(fd, GYROIOCSSAMPLERATE, GYRO_SAMPLERATE_DEFAULT);

		/* set the driver to poll at default rate */
		ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);

	}

	return OK;
}

int
Sensors::mag_init()
{
	int	fd;
	int	ret;

	fd = open(MAG0_DEVICE_PATH, 0);

	if (fd < 0) {
		warnx("FATAL: no magnetometer found: %s", MAG0_DEVICE_PATH);
		return ERROR;
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
			warnx("FATAL: mag sampling rate could not be set");
			return ERROR;
		}
	}



	ret = ioctl(fd, MAGIOCGEXTERNAL, 0);

	if (ret < 0) {
		warnx("FATAL: unknown if magnetometer is external or onboard");
		return ERROR;

	} else if (ret == 1) {
		_mag_is_external = true;

	} else {
		_mag_is_external = false;
	}

	close(fd);

	return OK;
}

int
Sensors::baro_init()
{
	int	fd;

	fd = open(BARO0_DEVICE_PATH, 0);

	if (fd < 0) {
		warnx("FATAL: No barometer found: %s", BARO0_DEVICE_PATH);
		return ERROR;
	}

	/* set the driver to poll at 150Hz */
	ioctl(fd, SENSORIOCSPOLLRATE, 150);

	close(fd);

	return OK;
}

int
Sensors::adc_init()
{

	_fd_adc = open(ADC0_DEVICE_PATH, O_RDONLY | O_NONBLOCK);

	if (_fd_adc < 0) {
		warnx("FATAL: no ADC found: %s", ADC0_DEVICE_PATH);
		return ERROR;
	}

	return OK;
}

void
Sensors::accel_poll(struct sensor_combined_s &raw)
{
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		struct accel_report	accel_report;

		orb_copy(ORB_ID(sensor_accel), _accel_sub, &accel_report);

		math::Vector<3> vect(accel_report.x, accel_report.y, accel_report.z);
		vect = _board_rotation * vect;

		raw.accelerometer_m_s2[0] = vect(0);
		raw.accelerometer_m_s2[1] = vect(1);
		raw.accelerometer_m_s2[2] = vect(2);

		raw.accelerometer_raw[0] = accel_report.x_raw;
		raw.accelerometer_raw[1] = accel_report.y_raw;
		raw.accelerometer_raw[2] = accel_report.z_raw;

		raw.accelerometer_timestamp = accel_report.timestamp;
		raw.accelerometer_errcount = accel_report.error_count;
	}

	orb_check(_accel1_sub, &accel_updated);

	if (accel_updated) {
		struct accel_report	accel_report;

		orb_copy(ORB_ID(sensor_accel), _accel1_sub, &accel_report);

		math::Vector<3> vect(accel_report.x, accel_report.y, accel_report.z);
		vect = _board_rotation * vect;

		raw.accelerometer1_m_s2[0] = vect(0);
		raw.accelerometer1_m_s2[1] = vect(1);
		raw.accelerometer1_m_s2[2] = vect(2);

		raw.accelerometer1_raw[0] = accel_report.x_raw;
		raw.accelerometer1_raw[1] = accel_report.y_raw;
		raw.accelerometer1_raw[2] = accel_report.z_raw;

		raw.accelerometer1_timestamp = accel_report.timestamp;
		raw.accelerometer1_errcount = accel_report.error_count;
	}

	orb_check(_accel2_sub, &accel_updated);

	if (accel_updated) {
		struct accel_report	accel_report;

		orb_copy(ORB_ID(sensor_accel), _accel2_sub, &accel_report);

		math::Vector<3> vect(accel_report.x, accel_report.y, accel_report.z);
		vect = _board_rotation * vect;

		raw.accelerometer2_m_s2[0] = vect(0);
		raw.accelerometer2_m_s2[1] = vect(1);
		raw.accelerometer2_m_s2[2] = vect(2);

		raw.accelerometer2_raw[0] = accel_report.x_raw;
		raw.accelerometer2_raw[1] = accel_report.y_raw;
		raw.accelerometer2_raw[2] = accel_report.z_raw;

		raw.accelerometer2_timestamp = accel_report.timestamp;
		raw.accelerometer2_errcount = accel_report.error_count;
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

		math::Vector<3> vect(gyro_report.x, gyro_report.y, gyro_report.z);
		vect = _board_rotation * vect;

		raw.gyro_rad_s[0] = vect(0);
		raw.gyro_rad_s[1] = vect(1);
		raw.gyro_rad_s[2] = vect(2);

		raw.gyro_raw[0] = gyro_report.x_raw;
		raw.gyro_raw[1] = gyro_report.y_raw;
		raw.gyro_raw[2] = gyro_report.z_raw;

		raw.timestamp = gyro_report.timestamp;
		raw.gyro_errcount = gyro_report.error_count;
	}

	orb_check(_gyro1_sub, &gyro_updated);

	if (gyro_updated) {
		struct gyro_report	gyro_report;

		orb_copy(ORB_ID(sensor_gyro), _gyro1_sub, &gyro_report);

		math::Vector<3> vect(gyro_report.x, gyro_report.y, gyro_report.z);
		vect = _board_rotation * vect;

		raw.gyro1_rad_s[0] = vect(0);
		raw.gyro1_rad_s[1] = vect(1);
		raw.gyro1_rad_s[2] = vect(2);

		raw.gyro1_raw[0] = gyro_report.x_raw;
		raw.gyro1_raw[1] = gyro_report.y_raw;
		raw.gyro1_raw[2] = gyro_report.z_raw;

		raw.gyro1_timestamp = gyro_report.timestamp;
		raw.gyro1_errcount = gyro_report.error_count;
	}

	orb_check(_gyro2_sub, &gyro_updated);

	if (gyro_updated) {
		struct gyro_report	gyro_report;

		orb_copy(ORB_ID(sensor_gyro), _gyro2_sub, &gyro_report);

		math::Vector<3> vect(gyro_report.x, gyro_report.y, gyro_report.z);
		vect = _board_rotation * vect;

		raw.gyro2_rad_s[0] = vect(0);
		raw.gyro2_rad_s[1] = vect(1);
		raw.gyro2_rad_s[2] = vect(2);

		raw.gyro2_raw[0] = gyro_report.x_raw;
		raw.gyro2_raw[1] = gyro_report.y_raw;
		raw.gyro2_raw[2] = gyro_report.z_raw;

		raw.gyro2_timestamp = gyro_report.timestamp;
		raw.gyro2_errcount = gyro_report.error_count;
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

		math::Vector<3> vect(mag_report.x, mag_report.y, mag_report.z);

		// XXX we need device-id based handling here

		if (_mag_is_external) {
			vect = _external_mag_rotation * vect;

		} else {
			vect = _board_rotation * vect;
		}

		raw.magnetometer_ga[0] = vect(0);
		raw.magnetometer_ga[1] = vect(1);
		raw.magnetometer_ga[2] = vect(2);

		raw.magnetometer_raw[0] = mag_report.x_raw;
		raw.magnetometer_raw[1] = mag_report.y_raw;
		raw.magnetometer_raw[2] = mag_report.z_raw;

		raw.magnetometer_timestamp = mag_report.timestamp;
		raw.magnetometer_errcount = mag_report.error_count;
	}

	orb_check(_mag1_sub, &mag_updated);

	if (mag_updated) {
		struct mag_report	mag_report;

		orb_copy(ORB_ID(sensor_mag), _mag1_sub, &mag_report);

		math::Vector<3> vect(mag_report.x, mag_report.y, mag_report.z);

		// XXX presume internal mag

		vect = _board_rotation * vect;

		raw.magnetometer1_ga[0] = vect(0);
		raw.magnetometer1_ga[1] = vect(1);
		raw.magnetometer1_ga[2] = vect(2);

		raw.magnetometer1_raw[0] = mag_report.x_raw;
		raw.magnetometer1_raw[1] = mag_report.y_raw;
		raw.magnetometer1_raw[2] = mag_report.z_raw;

		raw.magnetometer1_timestamp = mag_report.timestamp;
		raw.magnetometer1_errcount = mag_report.error_count;
	}

	orb_check(_mag2_sub, &mag_updated);

	if (mag_updated) {
		struct mag_report	mag_report;

		orb_copy(ORB_ID(sensor_mag), _mag2_sub, &mag_report);

		math::Vector<3> vect(mag_report.x, mag_report.y, mag_report.z);

		// XXX presume internal mag

		vect = _board_rotation * vect;

		raw.magnetometer2_ga[0] = vect(0);
		raw.magnetometer2_ga[1] = vect(1);
		raw.magnetometer2_ga[2] = vect(2);

		raw.magnetometer2_raw[0] = mag_report.x_raw;
		raw.magnetometer2_raw[1] = mag_report.y_raw;
		raw.magnetometer2_raw[2] = mag_report.z_raw;

		raw.magnetometer2_timestamp = mag_report.timestamp;
		raw.magnetometer2_errcount = mag_report.error_count;
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

		raw.baro_timestamp = _barometer.timestamp;
	}

	orb_check(_baro1_sub, &baro_updated);

	if (baro_updated) {

		struct baro_report	baro_report;

		orb_copy(ORB_ID(sensor_baro), _baro1_sub, &baro_report);

		raw.baro1_pres_mbar = baro_report.pressure; // Pressure in mbar
		raw.baro1_alt_meter = baro_report.altitude; // Altitude in meters
		raw.baro1_temp_celcius = baro_report.temperature; // Temperature in degrees celcius

		raw.baro1_timestamp = baro_report.timestamp;
	}
}

void
Sensors::diff_pres_poll(struct sensor_combined_s &raw)
{
	bool updated;
	orb_check(_diff_pres_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(differential_pressure), _diff_pres_sub, &_diff_pres);

		raw.differential_pressure_pa = _diff_pres.differential_pressure_raw_pa;
		raw.differential_pressure_timestamp = _diff_pres.timestamp;
		raw.differential_pressure_filtered_pa = _diff_pres.differential_pressure_filtered_pa;

		float air_temperature_celsius = (_diff_pres.temperature > -300.0f) ? _diff_pres.temperature :
						(raw.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG);

		_airspeed.timestamp = _diff_pres.timestamp;

		/* don't risk to feed negative airspeed into the system */
		_airspeed.indicated_airspeed_m_s = math::max(0.0f,
						   calc_indicated_airspeed(_diff_pres.differential_pressure_filtered_pa));
		_airspeed.true_airspeed_m_s = math::max(0.0f,
							calc_true_airspeed(_diff_pres.differential_pressure_filtered_pa + raw.baro_pres_mbar * 1e2f,
									raw.baro_pres_mbar * 1e2f, air_temperature_celsius));
		_airspeed.air_temperature_celsius = air_temperature_celsius;

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

		/* set offset parameters to new values */
		bool failed;
		int res;
		char str[30];
		unsigned mag_count = 0;
		unsigned gyro_count = 0;
		unsigned accel_count = 0;

		/* run through all gyro sensors */
		for (unsigned s = 0; s < 3; s++) {

			res = ERROR;
			(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);

			int fd = open(str, 0);

			if (fd < 0) {
				continue;
			}

			bool config_ok = false;

			/* run through all stored calibrations */ 
			for (unsigned i = 0; i < 3; i++) {
				/* initially status is ok per config */
				failed = false;

				(void)sprintf(str, "CAL_GYRO%u_ID", i);
				int device_id;
				failed |= (OK != param_get(param_find(str), &device_id));

				if (failed) {
					close(fd);
					continue;
				}

				/* if the calibration is for this device, apply it */
				if (device_id == ioctl(fd, DEVIOCGDEVICEID, 0)) {
					struct gyro_scale gscale = {};
					(void)sprintf(str, "CAL_GYRO%u_XOFF", i);
					failed |= (OK != param_get(param_find(str), &gscale.x_offset));
					(void)sprintf(str, "CAL_GYRO%u_YOFF", i);
					failed |= (OK != param_get(param_find(str), &gscale.y_offset));
					(void)sprintf(str, "CAL_GYRO%u_ZOFF", i);
					failed |= (OK != param_get(param_find(str), &gscale.z_offset));
					(void)sprintf(str, "CAL_GYRO%u_XSCALE", i);
					failed |= (OK != param_get(param_find(str), &gscale.x_scale));
					(void)sprintf(str, "CAL_GYRO%u_YSCALE", i);
					failed |= (OK != param_get(param_find(str), &gscale.y_scale));
					(void)sprintf(str, "CAL_GYRO%u_ZSCALE", i);
					failed |= (OK != param_get(param_find(str), &gscale.z_scale));

					if (failed) {
						warnx("%s: gyro #%u", CAL_FAILED_APPLY_CAL_MSG, gyro_count);
					} else {
						/* apply new scaling and offsets */
						res = ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gscale);
						if (res) {
							warnx(CAL_FAILED_APPLY_CAL_MSG);
						} else {
							gyro_count++;
							config_ok = true;
						}
					}
					break;
				}
			}

			close(fd);

			if (!config_ok) {
				warnx("NO CONFIG FOR GYRO #%u", s);
			}
		}

		/* run through all accel sensors */
		for (unsigned s = 0; s < 3; s++) {

			res = ERROR;
			(void)sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, s);

			int fd = open(str, 0);

			if (fd < 0) {
				continue;
			}

			bool config_ok = false;

			/* run through all stored calibrations */ 
			for (unsigned i = 0; i < 3; i++) {
				/* initially status is ok per config */
				failed = false;

				(void)sprintf(str, "CAL_ACC%u_ID", i);
				int device_id;
				failed |= (OK != param_get(param_find(str), &device_id));

				if (failed) {
					close(fd);
					continue;
				}

				/* if the calibration is for this device, apply it */
				if (device_id == ioctl(fd, DEVIOCGDEVICEID, 0)) {
					struct accel_scale gscale = {};
					(void)sprintf(str, "CAL_ACC%u_XOFF", i);
					failed |= (OK != param_get(param_find(str), &gscale.x_offset));
					(void)sprintf(str, "CAL_ACC%u_YOFF", i);
					failed |= (OK != param_get(param_find(str), &gscale.y_offset));
					(void)sprintf(str, "CAL_ACC%u_ZOFF", i);
					failed |= (OK != param_get(param_find(str), &gscale.z_offset));
					(void)sprintf(str, "CAL_ACC%u_XSCALE", i);
					failed |= (OK != param_get(param_find(str), &gscale.x_scale));
					(void)sprintf(str, "CAL_ACC%u_YSCALE", i);
					failed |= (OK != param_get(param_find(str), &gscale.y_scale));
					(void)sprintf(str, "CAL_ACC%u_ZSCALE", i);
					failed |= (OK != param_get(param_find(str), &gscale.z_scale));

					if (failed) {
						warnx("%s: acc #%u", CAL_FAILED_APPLY_CAL_MSG, accel_count);
					} else {
						/* apply new scaling and offsets */
						res = ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&gscale);
						if (res) {
							warnx(CAL_FAILED_APPLY_CAL_MSG);
						} else {
							accel_count++;
							config_ok = true;
						}
					}
					break;
				}
			}

			close(fd);

			if (!config_ok) {
				warnx("NO CONFIG FOR ACCEL #%u", s);
			}
		}

		/* run through all mag sensors */
		for (unsigned s = 0; s < 3; s++) {

			res = ERROR;
			(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, s);

			int fd = open(str, 0);

			if (fd < 0) {
				continue;
			}

			bool config_ok = false;

			/* run through all stored calibrations */ 
			for (unsigned i = 0; i < 3; i++) {
				/* initially status is ok per config */
				failed = false;

				(void)sprintf(str, "CAL_MAG%u_ID", i);
				int device_id;
				failed |= (OK != param_get(param_find(str), &device_id));

				if (failed) {
					close(fd);
					continue;
				}

				/* if the calibration is for this device, apply it */
				if (device_id == ioctl(fd, DEVIOCGDEVICEID, 0)) {
					struct mag_scale gscale = {};
					(void)sprintf(str, "CAL_MAG%u_XOFF", i);
					failed |= (OK != param_get(param_find(str), &gscale.x_offset));
					(void)sprintf(str, "CAL_MAG%u_YOFF", i);
					failed |= (OK != param_get(param_find(str), &gscale.y_offset));
					(void)sprintf(str, "CAL_MAG%u_ZOFF", i);
					failed |= (OK != param_get(param_find(str), &gscale.z_offset));
					(void)sprintf(str, "CAL_MAG%u_XSCALE", i);
					failed |= (OK != param_get(param_find(str), &gscale.x_scale));
					(void)sprintf(str, "CAL_MAG%u_YSCALE", i);
					failed |= (OK != param_get(param_find(str), &gscale.y_scale));
					(void)sprintf(str, "CAL_MAG%u_ZSCALE", i);
					failed |= (OK != param_get(param_find(str), &gscale.z_scale));

					if (failed) {
						warnx("%s: mag #%u", CAL_FAILED_APPLY_CAL_MSG, mag_count);
					} else {
						/* apply new scaling and offsets */
						res = ioctl(fd, MAGIOCSSCALE, (long unsigned int)&gscale);
						if (res) {
							warnx(CAL_FAILED_APPLY_CAL_MSG);
						} else {
							mag_count++;
							config_ok = true;
						}
					}
					break;
				}
			}

			close(fd);

			if (!config_ok) {
				warnx("NO CONFIG FOR MAG #%u", s);
			}
		}

		int fd = open(AIRSPEED0_DEVICE_PATH, 0);

		/* this sensor is optional, abort without error */

		if (fd >= 0) {
			struct airspeed_scale airscale = {
				_parameters.diff_pres_offset_pa,
				1.0f,
			};

			if (OK != ioctl(fd, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale)) {
				warn("WARNING: failed to set scale / offsets for airspeed sensor");
			}

			close(fd);
		}

		warnx("config: %u gyros, %u mags, %u accels", gyro_count, mag_count, accel_count);
	}
}

void
Sensors::rc_parameter_map_poll(bool forced)
{
	bool map_updated;
	orb_check(_rc_parameter_map_sub, &map_updated);

	if (map_updated) {
		orb_copy(ORB_ID(rc_parameter_map), _rc_parameter_map_sub, &_rc_parameter_map);

		/* update paramter handles to which the RC channels are mapped */
		for (int i = 0; i < RC_PARAM_MAP_NCHAN; i++) {
			if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
				/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
				 * or no request to map this channel to a param has been sent via mavlink
				 */
				continue;
			}

			/* Set the handle by index if the index is set, otherwise use the id */
			if (_rc_parameter_map.param_index[i] >= 0) {
				_parameter_handles.rc_param[i] = param_for_index((unsigned)_rc_parameter_map.param_index[i]);

			} else {
				_parameter_handles.rc_param[i] = param_find(_rc_parameter_map.param_id[i]);
			}

		}

		warnx("rc to parameter map updated");

		for (int i = 0; i < RC_PARAM_MAP_NCHAN; i++) {
			warnx("\ti %d param_id %s scale %.3f value0 %.3f, min %.3f, max %.3f",
			      i,
			      _rc_parameter_map.param_id[i],
			      (double)_rc_parameter_map.scale[i],
			      (double)_rc_parameter_map.value0[i],
			      (double)_rc_parameter_map.value_min[i],
			      (double)_rc_parameter_map.value_max[i]
			     );
		}
	}
}

void
Sensors::adc_poll(struct sensor_combined_s &raw)
{
	/* only read if publishing */
	if (!_publishing) {
		return;
	}

	hrt_abstime t = hrt_absolute_time();

	/* rate limit to 100 Hz */
	if (t - _last_adc >= 10000) {
		/* make space for a maximum of twelve channels (to ensure reading all channels at once) */
		struct adc_msg_s buf_adc[12];
		/* read all channels available */
		int ret = read(_fd_adc, &buf_adc, sizeof(buf_adc));

		if (ret >= (int)sizeof(buf_adc[0])) {

			/* Read add channels we got */
			for (unsigned i = 0; i < ret / sizeof(buf_adc[0]); i++) {
				/* Save raw voltage values */
				if (i < (sizeof(raw.adc_voltage_v) / sizeof(raw.adc_voltage_v[0]))) {
					raw.adc_voltage_v[i] = buf_adc[i].am_data / (4096.0f / 3.3f);
					raw.adc_mapping[i] = buf_adc[i].am_channel;
				}

				/* look for specific channels and process the raw voltage to measurement data */
				if (ADC_BATTERY_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {
					/* Voltage in volts */
					float voltage = (buf_adc[i].am_data * _parameters.battery_voltage_scaling);

					if (voltage > BATT_V_IGNORE_THRESHOLD) {
						_battery_status.voltage_v = voltage;

						/* one-time initialization of low-pass value to avoid long init delays */
						if (_battery_status.voltage_filtered_v < BATT_V_IGNORE_THRESHOLD) {
							_battery_status.voltage_filtered_v = voltage;
						}

						_battery_status.timestamp = t;
						_battery_status.voltage_filtered_v += (voltage - _battery_status.voltage_filtered_v) * BATT_V_LOWPASS;

					} else {
						/* mark status as invalid */
						_battery_status.voltage_v = -1.0f;
						_battery_status.voltage_filtered_v = -1.0f;
					}

				} else if (ADC_BATTERY_CURRENT_CHANNEL == buf_adc[i].am_channel) {
					/* handle current only if voltage is valid */
					if (_battery_status.voltage_v > 0.0f) {
						float current = (buf_adc[i].am_data * _parameters.battery_current_scaling);

						/* check measured current value */
						if (current >= 0.0f) {
							_battery_status.timestamp = t;
							_battery_status.current_a = current;

							if (_battery_current_timestamp != 0) {
								/* initialize discharged value */
								if (_battery_status.discharged_mah < 0.0f) {
									_battery_status.discharged_mah = 0.0f;
								}

								_battery_discharged += current * (t - _battery_current_timestamp);
								_battery_status.discharged_mah = ((float) _battery_discharged) / 3600000.0f;
							}
						}
					}

					_battery_current_timestamp = t;

				} else if (ADC_AIRSPEED_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {

					/* calculate airspeed, raw is the difference from */
					float voltage = (float)(buf_adc[i].am_data) * 3.3f / 4096.0f * 2.0f;  // V_ref/4096 * (voltage divider factor)

					/**
					 * The voltage divider pulls the signal down, only act on
					 * a valid voltage from a connected sensor. Also assume a non-
					 * zero offset from the sensor if its connected.
					 */
					if (voltage > 0.4f && (_parameters.diff_pres_analog_scale > 0.0f)) {

						float diff_pres_pa_raw = voltage * _parameters.diff_pres_analog_scale - _parameters.diff_pres_offset_pa;

						_diff_pres.timestamp = t;
						_diff_pres.differential_pressure_raw_pa = diff_pres_pa_raw;
						_diff_pres.differential_pressure_filtered_pa = (_diff_pres.differential_pressure_filtered_pa * 0.9f) +
								(diff_pres_pa_raw * 0.1f);
						_diff_pres.temperature = -1000.0f;

						/* announce the airspeed if needed, just publish else */
						if (_diff_pres_pub > 0) {
							orb_publish(ORB_ID(differential_pressure), _diff_pres_pub, &_diff_pres);

						} else {
							_diff_pres_pub = orb_advertise(ORB_ID(differential_pressure), &_diff_pres);
						}
					}
				}
			}

			_last_adc = t;

			if (_battery_status.voltage_filtered_v > BATT_V_IGNORE_THRESHOLD) {
				/* announce the battery status if needed, just publish else */
				if (_battery_pub > 0) {
					orb_publish(ORB_ID(battery_status), _battery_pub, &_battery_status);

				} else {
					_battery_pub = orb_advertise(ORB_ID(battery_status), &_battery_status);
				}
			}
		}
	}
}

float
Sensors::get_rc_value(uint8_t func, float min_value, float max_value)
{
	if (_rc.function[func] >= 0) {
		float value = _rc.channels[_rc.function[func]];

		if (value < min_value) {
			return min_value;

		} else if (value > max_value) {
			return max_value;

		} else {
			return value;
		}

	} else {
		return 0.0f;
	}
}

switch_pos_t
Sensors::get_rc_sw3pos_position(uint8_t func, float on_th, bool on_inv, float mid_th, bool mid_inv)
{
	if (_rc.function[func] >= 0) {
		float value = 0.5f * _rc.channels[_rc.function[func]] + 0.5f;

		if (on_inv ? value < on_th : value > on_th) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else if (mid_inv ? value < mid_th : value > mid_th) {
			return manual_control_setpoint_s::SWITCH_POS_MIDDLE;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}

	} else {
		return manual_control_setpoint_s::SWITCH_POS_NONE;
	}
}

switch_pos_t
Sensors::get_rc_sw2pos_position(uint8_t func, float on_th, bool on_inv)
{
	if (_rc.function[func] >= 0) {
		float value = 0.5f * _rc.channels[_rc.function[func]] + 0.5f;

		if (on_inv ? value < on_th : value > on_th) {
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else {
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}

	} else {
		return manual_control_setpoint_s::SWITCH_POS_NONE;
	}
}

void
Sensors::set_params_from_rc()
{
	static float param_rc_values[RC_PARAM_MAP_NCHAN] = {};

	for (int i = 0; i < RC_PARAM_MAP_NCHAN; i++) {
		if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
			/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
			 * or no request to map this channel to a param has been sent via mavlink
			 */
			continue;
		}

		float rc_val = get_rc_value((rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i), -1.0, 1.0);
		/* Check if the value has changed,
		 * maybe we need to introduce a more aggressive limit here */
		if (rc_val > param_rc_values[i] + FLT_EPSILON || rc_val < param_rc_values[i] - FLT_EPSILON) {
			param_rc_values[i] = rc_val;
			float param_val = math::constrain(
						  _rc_parameter_map.value0[i] + _rc_parameter_map.scale[i] * rc_val,
						  _rc_parameter_map.value_min[i], _rc_parameter_map.value_max[i]);
			param_set(_parameter_handles.rc_param[i], &param_val);
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
		struct rc_input_values rc_input;

		orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);

		/* detect RC signal loss */
		bool signal_lost;

		/* check flags and require at least four channels to consider the signal valid */
		if (rc_input.rc_lost || rc_input.rc_failsafe || rc_input.channel_count < 4) {
			/* signal is lost or no enough channels */
			signal_lost = true;

		} else {
			/* signal looks good */
			signal_lost = false;

			/* check failsafe */
			int8_t fs_ch = _rc.function[_parameters.rc_map_failsafe]; // get channel mapped to throttle

			if (_parameters.rc_map_failsafe > 0) { // if not 0, use channel number instead of rc.function mapping
				fs_ch = _parameters.rc_map_failsafe - 1;
			}

			if (_parameters.rc_fails_thr > 0 && fs_ch >= 0) {
				/* failsafe configured */
				if ((_parameters.rc_fails_thr < _parameters.min[fs_ch] && rc_input.values[fs_ch] < _parameters.rc_fails_thr) ||
				    (_parameters.rc_fails_thr > _parameters.max[fs_ch] && rc_input.values[fs_ch] > _parameters.rc_fails_thr)) {
					/* failsafe triggered, signal is lost by receiver */
					signal_lost = true;
				}
			}
		}

		unsigned channel_limit = rc_input.channel_count;

		if (channel_limit > _rc_max_chan_count) {
			channel_limit = _rc_max_chan_count;
		}

		/* read out and scale values from raw message even if signal is invalid */
		for (unsigned int i = 0; i < channel_limit; i++) {

			/*
			 * 1) Constrain to min/max values, as later processing depends on bounds.
			 */
			if (rc_input.values[i] < _parameters.min[i]) {
				rc_input.values[i] = _parameters.min[i];
			}

			if (rc_input.values[i] > _parameters.max[i]) {
				rc_input.values[i] = _parameters.max[i];
			}

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
				_rc.channels[i] = (rc_input.values[i] - _parameters.trim[i] - _parameters.dz[i]) / (float)(
							  _parameters.max[i] - _parameters.trim[i] - _parameters.dz[i]);

			} else if (rc_input.values[i] < (_parameters.trim[i] - _parameters.dz[i])) {
				_rc.channels[i] = (rc_input.values[i] - _parameters.trim[i] + _parameters.dz[i]) / (float)(
							  _parameters.trim[i] - _parameters.min[i] - _parameters.dz[i]);

			} else {
				/* in the configured dead zone, output zero */
				_rc.channels[i] = 0.0f;
			}

			_rc.channels[i] *= _parameters.rev[i];

			/* handle any parameter-induced blowups */
			if (!isfinite(_rc.channels[i])) {
				_rc.channels[i] = 0.0f;
			}
		}

		_rc.channel_count = rc_input.channel_count;
		_rc.rssi = rc_input.rssi;
		_rc.signal_lost = signal_lost;
		_rc.timestamp = rc_input.timestamp_last_signal;

		/* publish rc_channels topic even if signal is invalid, for debug */
		if (_rc_pub > 0) {
			orb_publish(ORB_ID(rc_channels), _rc_pub, &_rc);

		} else {
			_rc_pub = orb_advertise(ORB_ID(rc_channels), &_rc);
		}

		if (!signal_lost) {
			struct manual_control_setpoint_s manual;
			memset(&manual, 0 , sizeof(manual));

			/* fill values in manual_control_setpoint topic only if signal is valid */
			manual.timestamp = rc_input.timestamp_last_signal;

			/* limit controls */
			manual.y = get_rc_value (rc_channels_s::RC_CHANNELS_FUNCTION_ROLL, -1.0, 1.0);
			manual.x = get_rc_value (rc_channels_s::RC_CHANNELS_FUNCTION_PITCH, -1.0, 1.0);
			manual.r = get_rc_value (rc_channels_s::RC_CHANNELS_FUNCTION_YAW, -1.0, 1.0);
			manual.z = get_rc_value (rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE, 0.0, 1.0);
			manual.flaps = get_rc_value (rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS, -1.0, 1.0);
			manual.aux1 = get_rc_value (rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1, -1.0, 1.0);
			manual.aux2 = get_rc_value (rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2, -1.0, 1.0);
			manual.aux3 = get_rc_value (rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3, -1.0, 1.0);
			manual.aux4 = get_rc_value (rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4, -1.0, 1.0);
			manual.aux5 = get_rc_value (rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5, -1.0, 1.0);

			/* mode switches */
			manual.mode_switch = get_rc_sw3pos_position (rc_channels_s::RC_CHANNELS_FUNCTION_MODE, _parameters.rc_auto_th, _parameters.rc_auto_inv, _parameters.rc_assist_th, _parameters.rc_assist_inv);
			manual.posctl_switch = get_rc_sw2pos_position (rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL, _parameters.rc_posctl_th, _parameters.rc_posctl_inv);
			manual.return_switch = get_rc_sw2pos_position (rc_channels_s::RC_CHANNELS_FUNCTION_RETURN, _parameters.rc_return_th, _parameters.rc_return_inv);
			manual.loiter_switch = get_rc_sw2pos_position (rc_channels_s::RC_CHANNELS_FUNCTION_LOITER, _parameters.rc_loiter_th, _parameters.rc_loiter_inv);
			manual.acro_switch = get_rc_sw2pos_position (rc_channels_s::RC_CHANNELS_FUNCTION_ACRO, _parameters.rc_acro_th, _parameters.rc_acro_inv);
			manual.offboard_switch = get_rc_sw2pos_position (rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD, _parameters.rc_offboard_th, _parameters.rc_offboard_inv);

			/* publish manual_control_setpoint topic */
			if (_manual_control_pub > 0) {
				orb_publish(ORB_ID(manual_control_setpoint), _manual_control_pub, &manual);

			} else {
				_manual_control_pub = orb_advertise(ORB_ID(manual_control_setpoint), &manual);
			}

			/* copy from mapped manual control to control group 3 */
			struct actuator_controls_s actuator_group_3;
			memset(&actuator_group_3, 0 , sizeof(actuator_group_3));

			actuator_group_3.timestamp = rc_input.timestamp_last_signal;

			actuator_group_3.control[0] = manual.y;
			actuator_group_3.control[1] = manual.x;
			actuator_group_3.control[2] = manual.r;
			actuator_group_3.control[3] = manual.z;
			actuator_group_3.control[4] = manual.flaps;
			actuator_group_3.control[5] = manual.aux1;
			actuator_group_3.control[6] = manual.aux2;
			actuator_group_3.control[7] = manual.aux3;

			/* publish actuator_controls_3 topic */
			if (_actuator_group_3_pub > 0) {
				orb_publish(ORB_ID(actuator_controls_3), _actuator_group_3_pub, &actuator_group_3);

			} else {
				_actuator_group_3_pub = orb_advertise(ORB_ID(actuator_controls_3), &actuator_group_3);
			}

			/* Update parameters from RC Channels (tuning with RC) if activated */
			static hrt_abstime last_rc_to_param_map_time = 0;

			if (hrt_elapsed_time(&last_rc_to_param_map_time) > 1e6) {
				set_params_from_rc();
				last_rc_to_param_map_time = hrt_absolute_time();
			}
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

	/* start individual sensors */
	int ret;
	ret = accel_init();

	if (ret) {
		goto exit_immediate;
	}

	ret = gyro_init();

	if (ret) {
		goto exit_immediate;
	}

	ret = mag_init();

	if (ret) {
		goto exit_immediate;
	}

	ret = baro_init();

	if (ret) {
		goto exit_immediate;
	}

	ret = adc_init();

	if (ret) {
		goto exit_immediate;
	}

	/*
	 * do subscriptions
	 */
	_gyro_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 0);
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_mag_sub = orb_subscribe_multi(ORB_ID(sensor_mag), 0);
	_gyro1_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 1);
	_accel1_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 1);
	_mag1_sub = orb_subscribe_multi(ORB_ID(sensor_mag), 1);
	_gyro2_sub = orb_subscribe_multi(ORB_ID(sensor_gyro), 2);
	_accel2_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 2);
	_mag2_sub = orb_subscribe_multi(ORB_ID(sensor_mag), 2);
	_rc_sub = orb_subscribe(ORB_ID(input_rc));
	_baro_sub = orb_subscribe_multi(ORB_ID(sensor_baro), 0);
	_baro1_sub = orb_subscribe_multi(ORB_ID(sensor_baro), 1);
	_diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_rc_parameter_map_sub = orb_subscribe(ORB_ID(rc_parameter_map));
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

	/* set high initial error counts to deselect gyros */
	raw.gyro_errcount = 100000;
	raw.gyro1_errcount = 100000;
	raw.gyro2_errcount = 100000;

	/* set high initial error counts to deselect accels */
	raw.accelerometer_errcount = 100000;
	raw.accelerometer1_errcount = 100000;
	raw.accelerometer2_errcount = 100000;

	/* set high initial error counts to deselect mags */
	raw.magnetometer_errcount = 100000;
	raw.magnetometer1_errcount = 100000;
	raw.magnetometer2_errcount = 100000;

	memset(&_battery_status, 0, sizeof(_battery_status));
	_battery_status.voltage_v = -1.0f;
	_battery_status.voltage_filtered_v = -1.0f;
	_battery_status.current_a = -1.0f;
	_battery_status.discharged_mah = -1.0f;

	/* get a set of initial values */
	accel_poll(raw);
	gyro_poll(raw);
	mag_poll(raw);
	baro_poll(raw);
	diff_pres_poll(raw);

	parameter_update_poll(true /* forced */);
	rc_parameter_map_poll(true /* forced */);

	/* advertise the sensor_combined topic and make the initial publication */
	_sensor_pub = orb_advertise(ORB_ID(sensor_combined), &raw);

	/* wakeup source(s) */
	struct pollfd fds[1];

	/* use the gyro to pace output - XXX BROKEN if we are using the L3GD20 */
	fds[0].fd = _gyro_sub;
	fds[0].events = POLLIN;

	_task_should_exit = false;

	while (!_task_should_exit) {

		/* wait for up to 50ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		/* if pret == 0 it timed out - periodic check for _task_should_exit, etc. */

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

		/* check rc parameter map for updates */
		rc_parameter_map_poll();

		/* the timestamp of the raw struct is updated by the gyro_poll() method */

		/* copy most recent sensor data */
		gyro_poll(raw);
		accel_poll(raw);
		mag_poll(raw);
		baro_poll(raw);

		/* work out if main gyro timed out and fail over to alternate gyro */
		if (hrt_elapsed_time(&raw.timestamp) > 20 * 1000) {

			/* if the secondary failed as well, go to the tertiary */
			if (hrt_elapsed_time(&raw.gyro1_timestamp) > 20 * 1000) {
				fds[0].fd = _gyro2_sub;
			} else {
				fds[0].fd = _gyro1_sub;
			}
		}

		/* check battery voltage */
		adc_poll(raw);

		diff_pres_poll(raw);

		/* Inform other processes that new data is available to copy */
		if (_publishing) {
			orb_publish(ORB_ID(sensor_combined), _sensor_pub, &raw);
		}

		/* Look for new r/c input data */
		rc_poll();

		perf_end(_loop_perf);
	}

	warnx("exiting.");

exit_immediate:
	_sensors_task = -1;
	_exit(ret);
}

int
Sensors::start()
{
	ASSERT(_sensors_task == -1);

	/* start the task */
	_sensors_task = task_spawn_cmd("sensors_task",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       2000,
				       (main_t)&Sensors::task_main_trampoline,
				       nullptr);

	/* wait until the task is up and running or has failed */
	while (_sensors_task > 0 && _task_should_exit) {
		usleep(100);
	}

	if (_sensors_task < 0) {
		return -ERROR;
	}

	return OK;
}

int sensors_main(int argc, char *argv[])
{
	if (argc < 1) {
		errx(1, "usage: sensors {start|stop|status}");
	}

	if (!strcmp(argv[1], "start")) {

		if (sensors::g_sensors != nullptr) {
			errx(0, "already running");
		}

		sensors::g_sensors = new Sensors;

		if (sensors::g_sensors == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != sensors::g_sensors->start()) {
			delete sensors::g_sensors;
			sensors::g_sensors = nullptr;
			err(1, "start failed");
		}

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (sensors::g_sensors == nullptr) {
			errx(1, "not running");
		}

		delete sensors::g_sensors;
		sensors::g_sensors = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (sensors::g_sensors) {
			errx(0, "is running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
