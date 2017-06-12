/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 *
 * PX4 Flight Core transitional mapping layer.
 *
 * This app / class mapps the PX4 middleware layer / drivers to the application
 * layer of the PX4 Flight Core. Individual sensors can be accessed directly as
 * well instead of relying on the sensor_combined topic.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

// TODO-JYW: TESTING-TESTING
#define DEBUG_BUILD 1

#include <board_config.h>

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <px4_time.h>

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

#include <px4_adc.h>

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

#include <lib/ecl/validation/data_validator.h>

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

#include <DevMgr.hpp>

#include "sensors_init.h"

using namespace DriverFramework;

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
 *
 * The channel definitions (e.g., ADC_BATTERY_VOLTAGE_CHANNEL, ADC_BATTERY_CURRENT_CHANNEL, and ADC_AIRSPEED_VOLTAGE_CHANNEL) are defined in board_config.h
 */


#define BATT_V_LOWPASS			0.001f
#define BATT_V_IGNORE_THRESHOLD		2.1f

/**
 * HACK - true temperature is much less than indicated temperature in baro,
 * subtract 5 degrees in an attempt to account for the electrical upheating of the PCB
 */
#define PCB_TEMP_ESTIMATE_DEG		5.0f
#define STICK_ON_OFF_LIMIT		0.75f
#define MAG_ROT_VAL_INTERNAL		-1

#define SENSOR_COUNT_MAX		3

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define CAL_ERROR_APPLY_CAL_MSG "FAILED APPLYING %s CAL #%u"

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
	static const unsigned _rc_max_chan_count =
		input_rc_s::RC_INPUT_MAX_CHANNELS;	/**< maximum number of r/c channels we handle */

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
	 * Update parameters from RC channels if the functionality is activated and the
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
	DevHandle 	_h_adc;				/**< ADC driver handle */
	hrt_abstime	_last_adc;			/**< last time we took input from the ADC */

	bool 		_task_should_exit;		/**< if true, sensor task should exit */
	int 		_sensors_task;			/**< task handle for sensor task */

	bool		_hil_enabled;			/**< if true, HIL is active */
	bool		_publishing;			/**< if true, we are publishing sensor data */
	bool		_armed;				/**< arming status of the vehicle */

	int		_gyro_sub[SENSOR_COUNT_MAX];	/**< raw gyro data subscription */
	int		_accel_sub[SENSOR_COUNT_MAX];	/**< raw accel data subscription */
	int		_mag_sub[SENSOR_COUNT_MAX];	/**< raw mag data subscription */
	int		_baro_sub[SENSOR_COUNT_MAX];	/**< raw baro data subscription */
	unsigned	_gyro_count;			/**< raw gyro data count */
	unsigned	_accel_count;			/**< raw accel data count */
	unsigned	_mag_count;			/**< raw mag data count */
	unsigned	_baro_count;			/**< raw baro data count */

	int 		_rc_sub;			/**< raw rc channels data subscription */
	int		_diff_pres_sub;			/**< raw differential pressure subscription */
	int		_vcontrol_mode_sub;		/**< vehicle control mode subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int		_rc_parameter_map_sub;		/**< rc parameter map subscription */
	int 		_manual_control_sub;		/**< notification of manual control updates */

	orb_advert_t	_sensor_pub;			/**< combined sensor data topic */
	orb_advert_t	_manual_control_pub;		/**< manual control signal topic */
	orb_advert_t	_actuator_group_3_pub;		/**< manual control as actuator topic */
	orb_advert_t	_rc_pub;			/**< raw r/c control topic */
	orb_advert_t	_battery_pub;			/**< battery status */
	orb_advert_t	_airspeed_pub;			/**< airspeed */
	orb_advert_t	_diff_pres_pub;			/**< differential_pressure */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	DataValidator	_airspeed_validator;		/**< data validator to monitor airspeed */

	struct rc_channels_s _rc;			/**< r/c channel data */
	struct battery_status_s _battery_status;	/**< battery status */
	struct baro_report _barometer;			/**< barometer data */
	struct differential_pressure_s _diff_pres;
	struct airspeed_s _airspeed;
	struct rc_parameter_map_s _rc_parameter_map;
	float _param_rc_values[rc_parameter_map_s::RC_PARAM_MAP_NCHAN];	/**< parameter values for RC control */

	math::Matrix<3, 3>	_board_rotation;	/**< rotation matrix for the orientation that the board is mounted */
	math::Matrix<3, 3>	_mag_rotation[3];	/**< rotation matrix for the orientation that the external mag0 is mounted */

	uint64_t _battery_discharged;			/**< battery discharged current in mA*ms */
	hrt_abstime _battery_current_timestamp;		/**< timestamp of last battery current reading */

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

		float board_offset[3];

		int rc_map_roll;
		int rc_map_pitch;
		int rc_map_yaw;
		int rc_map_throttle;
		int rc_map_failsafe;

		int rc_map_mode_sw;
		int rc_map_return_sw;
		int rc_map_rattitude_sw;
		int rc_map_posctl_sw;
		int rc_map_loiter_sw;
		int rc_map_acro_sw;
		int rc_map_offboard_sw;
		int rc_map_kill_sw;

		int rc_map_flaps;

		int rc_map_aux1;
		int rc_map_aux2;
		int rc_map_aux3;
		int rc_map_aux4;
		int rc_map_aux5;

		int rc_map_param[rc_parameter_map_s::RC_PARAM_MAP_NCHAN];

		int rc_map_flightmode;

		int32_t rc_fails_thr;
		float rc_assist_th;
		float rc_auto_th;
		float rc_rattitude_th;
		float rc_posctl_th;
		float rc_return_th;
		float rc_loiter_th;
		float rc_acro_th;
		float rc_offboard_th;
		float rc_killswitch_th;
		bool rc_assist_inv;
		bool rc_auto_inv;
		bool rc_rattitude_inv;
		bool rc_posctl_inv;
		bool rc_return_inv;
		bool rc_loiter_inv;
		bool rc_acro_inv;
		bool rc_offboard_inv;
		bool rc_killswitch_inv;

		float battery_voltage_scaling;
		float battery_current_scaling;
		float battery_current_offset;

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
		param_t rc_map_rattitude_sw;
		param_t rc_map_posctl_sw;
		param_t rc_map_loiter_sw;
		param_t rc_map_acro_sw;
		param_t rc_map_offboard_sw;
		param_t rc_map_kill_sw;

		param_t rc_map_flaps;

		param_t rc_map_aux1;
		param_t rc_map_aux2;
		param_t rc_map_aux3;
		param_t rc_map_aux4;
		param_t rc_map_aux5;

		param_t rc_map_param[rc_parameter_map_s::RC_PARAM_MAP_NCHAN];
		param_t rc_param[rc_parameter_map_s::RC_PARAM_MAP_NCHAN];	/**< param handles for the parameters which are bound
							  to a RC channel, equivalent float values in the
							  _parameters struct are not existing
							  because these parameters are never read. */

		param_t rc_map_flightmode;

		param_t rc_fails_thr;
		param_t rc_assist_th;
		param_t rc_auto_th;
		param_t rc_rattitude_th;
		param_t rc_posctl_th;
		param_t rc_return_th;
		param_t rc_loiter_th;
		param_t rc_acro_th;
		param_t rc_offboard_th;
		param_t rc_killswitch_th;

		param_t battery_voltage_scaling;
		param_t battery_current_scaling;
		param_t battery_current_offset;

		param_t board_rotation;

		param_t board_offset[3];

		param_t baro_qnh;

	}		_parameter_handles;		/**< handles for interesting parameters */


	int		init_sensor_class(const struct orb_metadata *meta, int *subs,
					  uint32_t *priorities, uint32_t *errcount);

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

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
	 * Apply a gyro calibration.
	 *
	 * @param h: reference to the DevHandle in use
	 * @param gscale: the calibration data.
	 * @param device: the device id of the sensor.
	 * @return: true if config is ok
	 */
	bool	apply_gyro_calibration(DevHandle &h, const struct gyro_calibration_s *gcal, const int device_id);

	/**
	 * Apply a accel calibration.
	 *
	 * @param h: reference to the DevHandle in use
	 * @param ascale: the calibration data.
	 * @param device: the device id of the sensor.
	 * @return: true if config is ok
	 */
	bool	apply_accel_calibration(DevHandle &h, const struct accel_calibration_s *acal, const int device_id);

	/**
	 * Apply a mag calibration.
	 *
	 * @param h: reference to the DevHandle in use
	 * @param gscale: the calibration data.
	 * @param device: the device id of the sensor.
	 * @return: true if config is ok
	 */
	bool	apply_mag_calibration(DevHandle &h, const struct mag_calibration_s *mcal, const int device_id);

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
	_h_adc(),
	_last_adc(0),

	_task_should_exit(true),
	_sensors_task(-1),
	_hil_enabled(false),
	_publishing(true),
	_armed(false),

	/* subscriptions */
	_gyro_sub{ -1, -1, -1},
	_accel_sub{ -1, -1, -1},
	_mag_sub{ -1, -1, -1},
	_baro_sub{ -1, -1, -1},
	_gyro_count(0),
	_accel_count(0),
	_mag_count(0),
	_baro_count(0),
	_rc_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_rc_parameter_map_sub(-1),
	_manual_control_sub(-1),

	/* publications */
	_sensor_pub(nullptr),
	_manual_control_pub(nullptr),
	_actuator_group_3_pub(nullptr),
	_rc_pub(nullptr),
	_battery_pub(nullptr),
	_airspeed_pub(nullptr),
	_diff_pres_pub(nullptr),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "sensor task update")),
	_airspeed_validator(),

	_param_rc_values{},
	_board_rotation{},
	_mag_rotation{},

	_battery_discharged(0),
	_battery_current_timestamp(0)
{
	/* initialize subscriptions */
	for (unsigned i = 0; i < SENSOR_COUNT_MAX; i++) {
		_gyro_sub[i] = -1;
		_accel_sub[i] = -1;
		_mag_sub[i] = -1;
		_baro_sub[i] = -1;
	}

	memset(&_rc, 0, sizeof(_rc));
	memset(&_diff_pres, 0, sizeof(_diff_pres));
	memset(&_parameters, 0, sizeof(_parameters));
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
	_parameter_handles.rc_map_rattitude_sw = param_find("RC_MAP_RATT_SW");
	_parameter_handles.rc_map_posctl_sw = param_find("RC_MAP_POSCTL_SW");
	_parameter_handles.rc_map_loiter_sw = param_find("RC_MAP_LOITER_SW");
	_parameter_handles.rc_map_acro_sw = param_find("RC_MAP_ACRO_SW");
	_parameter_handles.rc_map_offboard_sw = param_find("RC_MAP_OFFB_SW");
	_parameter_handles.rc_map_kill_sw = param_find("RC_MAP_KILL_SW");

	_parameter_handles.rc_map_aux1 = param_find("RC_MAP_AUX1");
	_parameter_handles.rc_map_aux2 = param_find("RC_MAP_AUX2");
	_parameter_handles.rc_map_aux3 = param_find("RC_MAP_AUX3");
	_parameter_handles.rc_map_aux4 = param_find("RC_MAP_AUX4");
	_parameter_handles.rc_map_aux5 = param_find("RC_MAP_AUX5");

	/* RC to parameter mapping for changing parameters with RC */
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		char name[rc_parameter_map_s::PARAM_ID_LEN];
		snprintf(name, rc_parameter_map_s::PARAM_ID_LEN, "RC_MAP_PARAM%d",
			 i + 1); // shifted by 1 because param name starts at 1
		_parameter_handles.rc_map_param[i] = param_find(name);
	}

	_parameter_handles.rc_map_flightmode = param_find("RC_MAP_FLTMODE");

	/* RC thresholds */
	_parameter_handles.rc_fails_thr = param_find("RC_FAILS_THR");
	_parameter_handles.rc_assist_th = param_find("RC_ASSIST_TH");
	_parameter_handles.rc_auto_th = param_find("RC_AUTO_TH");
	_parameter_handles.rc_rattitude_th = param_find("RC_RATT_TH");
	_parameter_handles.rc_posctl_th = param_find("RC_POSCTL_TH");
	_parameter_handles.rc_return_th = param_find("RC_RETURN_TH");
	_parameter_handles.rc_loiter_th = param_find("RC_LOITER_TH");
	_parameter_handles.rc_acro_th = param_find("RC_ACRO_TH");
	_parameter_handles.rc_offboard_th = param_find("RC_OFFB_TH");
	_parameter_handles.rc_killswitch_th = param_find("RC_KILLSWITCH_TH");

	/* Differential pressure offset */
	_parameter_handles.diff_pres_offset_pa = param_find("SENS_DPRES_OFF");
	_parameter_handles.diff_pres_analog_scale = param_find("SENS_DPRES_ANSC");

	_parameter_handles.battery_voltage_scaling = param_find("BAT_V_SCALING");
	_parameter_handles.battery_current_scaling = param_find("BAT_C_SCALING");
	_parameter_handles.battery_current_offset = param_find("BAT_C_OFFSET");

	/* rotations */
	_parameter_handles.board_rotation = param_find("SENS_BOARD_ROT");

	/* rotation offsets */
	_parameter_handles.board_offset[0] = param_find("SENS_BOARD_X_OFF");
	_parameter_handles.board_offset[1] = param_find("SENS_BOARD_Y_OFF");
	_parameter_handles.board_offset[2] = param_find("SENS_BOARD_Z_OFF");

	/* Barometer QNH */
	_parameter_handles.baro_qnh = param_find("SENS_BARO_QNH");

	// These are parameters for which QGroundControl always expects to be returned in a list request.
	// We do a param_find here to force them into the list.
	(void)param_find("RC_CHAN_CNT");
	(void)param_find("RC_TH_USER");
	(void)param_find("CAL_MAG0_ID");
	(void)param_find("CAL_MAG1_ID");
	(void)param_find("CAL_MAG2_ID");
	(void)param_find("CAL_MAG0_ROT");
	(void)param_find("CAL_MAG1_ROT");
	(void)param_find("CAL_MAG2_ROT");
	(void)param_find("SYS_PARAM_VER");
	(void)param_find("SYS_AUTOSTART");
	(void)param_find("SYS_AUTOCONFIG");
	(void)param_find("PWM_MIN");
	(void)param_find("PWM_MAX");
	(void)param_find("PWM_DISARMED");
	(void)param_find("PWM_AUX_MIN");
	(void)param_find("PWM_AUX_MAX");
	(void)param_find("PWM_AUX_DISARMED");
	(void)param_find("TRIG_MODE");
	(void)param_find("UAVCAN_ENABLE");
	(void)param_find("LPE_ENABLED");

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
				px4_task_delete(_sensors_task);
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
		if (!PX4_ISFINITE(tmpScaleFactor) ||
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

	if (param_get(_parameter_handles.rc_map_rattitude_sw, &(_parameters.rc_map_rattitude_sw)) != OK) {
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

	if (param_get(_parameter_handles.rc_map_kill_sw, &(_parameters.rc_map_kill_sw)) != OK) {
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

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		param_get(_parameter_handles.rc_map_param[i], &(_parameters.rc_map_param[i]));
	}

	param_get(_parameter_handles.rc_map_flightmode, &(_parameters.rc_map_flightmode));

	param_get(_parameter_handles.rc_fails_thr, &(_parameters.rc_fails_thr));
	param_get(_parameter_handles.rc_assist_th, &(_parameters.rc_assist_th));
	_parameters.rc_assist_inv = (_parameters.rc_assist_th < 0);
	_parameters.rc_assist_th = fabs(_parameters.rc_assist_th);
	param_get(_parameter_handles.rc_auto_th, &(_parameters.rc_auto_th));
	_parameters.rc_auto_inv = (_parameters.rc_auto_th < 0);
	_parameters.rc_auto_th = fabs(_parameters.rc_auto_th);
	param_get(_parameter_handles.rc_rattitude_th, &(_parameters.rc_rattitude_th));
	_parameters.rc_rattitude_inv = (_parameters.rc_rattitude_th < 0);
	_parameters.rc_rattitude_th = fabs(_parameters.rc_rattitude_th);
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
	param_get(_parameter_handles.rc_killswitch_th, &(_parameters.rc_killswitch_th));
	_parameters.rc_killswitch_inv = (_parameters.rc_killswitch_th < 0);
	_parameters.rc_killswitch_th = fabs(_parameters.rc_killswitch_th);

	/* update RC function mappings */
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE] = _parameters.rc_map_throttle - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ROLL] = _parameters.rc_map_roll - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PITCH] = _parameters.rc_map_pitch - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_YAW] = _parameters.rc_map_yaw - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_MODE] = _parameters.rc_map_mode_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RETURN] = _parameters.rc_map_return_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE] = _parameters.rc_map_rattitude_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL] = _parameters.rc_map_posctl_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_LOITER] = _parameters.rc_map_loiter_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ACRO] = _parameters.rc_map_acro_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD] = _parameters.rc_map_offboard_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH] = _parameters.rc_map_kill_sw - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS] = _parameters.rc_map_flaps - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1] = _parameters.rc_map_aux1 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2] = _parameters.rc_map_aux2 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3] = _parameters.rc_map_aux3 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4] = _parameters.rc_map_aux4 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5] = _parameters.rc_map_aux5 - 1;

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] = _parameters.rc_map_param[i] - 1;
	}

	/* Airspeed offset */
	param_get(_parameter_handles.diff_pres_offset_pa, &(_parameters.diff_pres_offset_pa));
	param_get(_parameter_handles.diff_pres_analog_scale, &(_parameters.diff_pres_analog_scale));

	/* scaling of ADC ticks to battery voltage */
	if (param_get(_parameter_handles.battery_voltage_scaling, &(_parameters.battery_voltage_scaling)) != OK) {
		warnx("%s", paramerr);

	} else if (_parameters.battery_voltage_scaling < 0.0f) {
		/* apply scaling according to defaults if set to default */
#if defined (CONFIG_ARCH_BOARD_PX4FMU_V4)
		_parameters.battery_voltage_scaling = 0.011f;
#elif defined (CONFIG_ARCH_BOARD_PX4FMU_V2) || defined ( CONFIG_ARCH_BOARD_MINDPX_V2 ) || defined ( CONFIG_ARCH_BOARD_PX4FMU_V4PRO )
		_parameters.battery_voltage_scaling = 0.0082f;
#elif defined (CONFIG_ARCH_BOARD_AEROCORE)
		_parameters.battery_voltage_scaling = 0.0063f;
#elif defined (CONFIG_ARCH_BOARD_PX4FMU_V1)
		_parameters.battery_voltage_scaling = 0.00459340659f;
#else
		/* ensure a missing default trips a low voltage lockdown */
		_parameters.battery_voltage_scaling = 0.00001f;
#endif
	}

	/* scaling of ADC ticks to battery current */
	if (param_get(_parameter_handles.battery_current_scaling, &(_parameters.battery_current_scaling)) != OK) {
		warnx("%s", paramerr);

	} else if (_parameters.battery_current_scaling < 0.0f) {
		/* apply scaling according to defaults if set to default */
#if defined (CONFIG_ARCH_BOARD_PX4FMU_V4)
		/* current scaling for ACSP4 */
		_parameters.battery_current_scaling = 0.0293f;
#elif defined (CONFIG_ARCH_BOARD_PX4FMU_V2) || defined ( CONFIG_ARCH_BOARD_MINDPX_V2 ) || defined ( CONFIG_ARCH_BOARD_PX4FMU_V4PRO )
		/* current scaling for 3DR power brick */
		_parameters.battery_current_scaling = 0.0124f;
#elif defined (CONFIG_ARCH_BOARD_AEROCORE)
		_parameters.battery_current_scaling = 0.0124f;
#elif defined (CONFIG_ARCH_BOARD_PX4FMU_V1)
		_parameters.battery_current_scaling = 0.0124f;
#else
		/* ensure a missing default leads to an unrealistic current value */
		_parameters.battery_current_scaling = 0.00001f;
#endif
	}

	if (param_get(_parameter_handles.battery_current_offset, &(_parameters.battery_current_offset)) != OK) {
		warnx("%s", paramerr);

	} else if (_parameters.battery_current_offset < 0.0f) {
		_parameters.battery_current_offset = 0.0f;
	}

	param_get(_parameter_handles.board_rotation, &(_parameters.board_rotation));
	get_rot_matrix((enum Rotation)_parameters.board_rotation, &_board_rotation);

	param_get(_parameter_handles.board_offset[0], &(_parameters.board_offset[0]));
	param_get(_parameter_handles.board_offset[1], &(_parameters.board_offset[1]));
	param_get(_parameter_handles.board_offset[2], &(_parameters.board_offset[2]));

	/** fine tune board offset on parameter update **/
	math::Matrix<3, 3> board_rotation_offset;
	board_rotation_offset.from_euler(M_DEG_TO_RAD_F * _parameters.board_offset[0],
					 M_DEG_TO_RAD_F * _parameters.board_offset[1],
					 M_DEG_TO_RAD_F * _parameters.board_offset[2]);

	_board_rotation = board_rotation_offset * _board_rotation;

	/* update barometer qnh setting */
	param_get(_parameter_handles.baro_qnh, &(_parameters.baro_qnh));
	DevHandle h_baro;
	DevMgr::getHandle(BARO0_DEVICE_PATH, h_baro);

#ifndef __PX4_QURT

	// TODO: this needs fixing for QURT
	if (!h_baro.isValid()) {
		warnx("ERROR: no barometer found on %s (%d)", BARO0_DEVICE_PATH, h_baro.getError());
		return ERROR;

	} else {
		int baroret = h_baro.ioctl(BAROIOCSMSLPRESSURE, (unsigned long)(_parameters.baro_qnh * 100));

		if (baroret) {
			warnx("qnh could not be set");
			return ERROR;
		}
	}

#endif

	return OK;
}


int
Sensors::adc_init()
{

	DevMgr::getHandle(ADC0_DEVICE_PATH, _h_adc);

	if (!_h_adc.isValid()) {
		warnx("FATAL: no ADC found: %s (%d)", ADC0_DEVICE_PATH, _h_adc.getError());
		return ERROR;
	}

	return OK;
}

void
Sensors::accel_poll(struct sensor_combined_s &raw)
{
	for (unsigned i = 0; i < _accel_count; i++) {
		bool accel_updated;
		orb_check(_accel_sub[i], &accel_updated);

		if (accel_updated) {
			struct accel_report	accel_report;

			orb_copy(ORB_ID(sensor_accel), _accel_sub[i], &accel_report);

			math::Vector<3> vect(accel_report.x, accel_report.y, accel_report.z);
			vect = _board_rotation * vect;

			raw.accelerometer_m_s2[i * 3 + 0] = vect(0);
			raw.accelerometer_m_s2[i * 3 + 1] = vect(1);
			raw.accelerometer_m_s2[i * 3 + 2] = vect(2);

			math::Vector<3> vect_int(accel_report.x_integral, accel_report.y_integral, accel_report.z_integral);
			vect_int = _board_rotation * vect_int;

			raw.accelerometer_integral_m_s[i * 3 + 0] = vect_int(0);
			raw.accelerometer_integral_m_s[i * 3 + 1] = vect_int(1);
			raw.accelerometer_integral_m_s[i * 3 + 2] = vect_int(2);

			raw.accelerometer_integral_dt[i] = accel_report.integral_dt;

			raw.accelerometer_raw[i * 3 + 0] = accel_report.x_raw;
			raw.accelerometer_raw[i * 3 + 1] = accel_report.y_raw;
			raw.accelerometer_raw[i * 3 + 2] = accel_report.z_raw;

			raw.accelerometer_timestamp[i] = accel_report.timestamp;
			raw.accelerometer_errcount[i] = accel_report.error_count;
			raw.accelerometer_temp[i] = accel_report.temperature;
		}
	}
}

void
Sensors::gyro_poll(struct sensor_combined_s &raw)
{
	for (unsigned i = 0; i < _gyro_count; i++) {
		bool gyro_updated;
		orb_check(_gyro_sub[i], &gyro_updated);

		if (gyro_updated) {
			struct gyro_report	gyro_report;

			orb_copy(ORB_ID(sensor_gyro), _gyro_sub[i], &gyro_report);

			math::Vector<3> vect(gyro_report.x, gyro_report.y, gyro_report.z);
			vect = _board_rotation * vect;

			raw.gyro_rad_s[i * 3 + 0] = vect(0);
			raw.gyro_rad_s[i * 3 + 1] = vect(1);
			raw.gyro_rad_s[i * 3 + 2] = vect(2);

			math::Vector<3> vect_int(gyro_report.x_integral, gyro_report.y_integral, gyro_report.z_integral);
			vect_int = _board_rotation * vect_int;

			raw.gyro_integral_rad[i * 3 + 0] = vect_int(0);
			raw.gyro_integral_rad[i * 3 + 1] = vect_int(1);
			raw.gyro_integral_rad[i * 3 + 2] = vect_int(2);

			raw.gyro_integral_dt[i] = gyro_report.integral_dt;

			raw.gyro_raw[i * 3 + 0] = gyro_report.x_raw;
			raw.gyro_raw[i * 3 + 1] = gyro_report.y_raw;
			raw.gyro_raw[i * 3 + 2] = gyro_report.z_raw;

			raw.gyro_timestamp[i] = gyro_report.timestamp;

			if (i == 0) {
				raw.timestamp = gyro_report.timestamp;
			}

			raw.gyro_errcount[i] = gyro_report.error_count;
			raw.gyro_temp[i] = gyro_report.temperature;
		}
	}
}

void
Sensors::mag_poll(struct sensor_combined_s &raw)
{
	for (unsigned i = 0; i < _mag_count; i++) {
		bool mag_updated;
		orb_check(_mag_sub[i], &mag_updated);

		if (mag_updated) {
			struct mag_report	mag_report;

			orb_copy(ORB_ID(sensor_mag), _mag_sub[i], &mag_report);

			math::Vector<3> vect(mag_report.x, mag_report.y, mag_report.z);

			vect = _mag_rotation[i] * vect;

			raw.magnetometer_ga[i * 3 + 0] = vect(0);
			raw.magnetometer_ga[i * 3 + 1] = vect(1);
			raw.magnetometer_ga[i * 3 + 2] = vect(2);

			raw.magnetometer_raw[i * 3 + 0] = mag_report.x_raw;
			raw.magnetometer_raw[i * 3 + 1] = mag_report.y_raw;
			raw.magnetometer_raw[i * 3 + 2] = mag_report.z_raw;

			raw.magnetometer_timestamp[i] = mag_report.timestamp;
			raw.magnetometer_errcount[i] = mag_report.error_count;
			raw.magnetometer_temp[i] = mag_report.temperature;
		}
	}
}

void
Sensors::baro_poll(struct sensor_combined_s &raw)
{
	for (unsigned i = 0; i < _baro_count; i++) {
		bool baro_updated;
		orb_check(_baro_sub[i], &baro_updated);

		if (baro_updated) {

			orb_copy(ORB_ID(sensor_baro), _baro_sub[i], &_barometer);

			raw.baro_pres_mbar[i] = _barometer.pressure; // Pressure in mbar
			raw.baro_alt_meter[i] = _barometer.altitude; // Altitude in meters
			raw.baro_temp_celcius[i] = _barometer.temperature; // Temperature in degrees celcius

			raw.baro_timestamp[i] = _barometer.timestamp;
		}
	}
}

void
Sensors::diff_pres_poll(struct sensor_combined_s &raw)
{
	bool updated;
	orb_check(_diff_pres_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(differential_pressure), _diff_pres_sub, &_diff_pres);

		raw.differential_pressure_pa[0] = _diff_pres.differential_pressure_raw_pa;
		raw.differential_pressure_timestamp[0] = _diff_pres.timestamp;
		raw.differential_pressure_filtered_pa[0] = _diff_pres.differential_pressure_filtered_pa;

		float air_temperature_celsius = (_diff_pres.temperature > -300.0f) ? _diff_pres.temperature :
						(raw.baro_temp_celcius[0] - PCB_TEMP_ESTIMATE_DEG);

		_airspeed.timestamp = _diff_pres.timestamp;

		/* push data into validator */
		_airspeed_validator.put(_airspeed.timestamp, _diff_pres.differential_pressure_raw_pa, _diff_pres.error_count, 100);

#ifdef __PX4_POSIX
		_airspeed.confidence = 1.0f;
#else
		_airspeed.confidence = _airspeed_validator.confidence(hrt_absolute_time());
#endif

		/* don't risk to feed negative airspeed into the system */
		_airspeed.indicated_airspeed_m_s = math::max(0.0f,
						   calc_indicated_airspeed(_diff_pres.differential_pressure_filtered_pa));

		_airspeed.true_airspeed_m_s = math::max(0.0f,
							calc_true_airspeed(_diff_pres.differential_pressure_filtered_pa + raw.baro_pres_mbar[0] * 1e2f,
									raw.baro_pres_mbar[0] * 1e2f, air_temperature_celsius));
		_airspeed.true_airspeed_unfiltered_m_s = math::max(0.0f,
				calc_true_airspeed(_diff_pres.differential_pressure_raw_pa + raw.baro_pres_mbar[0] * 1e2f,
						   raw.baro_pres_mbar[0] * 1e2f, air_temperature_celsius));

		_airspeed.air_temperature_celsius = air_temperature_celsius;

		/* announce the airspeed if needed, just publish else */
		if (_airspeed_pub != nullptr) {
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
		if (vcontrol_mode.flag_system_hil_enabled && !_hil_enabled) {
			_hil_enabled = true;
			_publishing = false;
			_armed = vcontrol_mode.flag_armed;

			/* switching from HIL to non-HIL mode */

		} else if (!_publishing && !_hil_enabled) {
			_hil_enabled = false;
			_publishing = true;
			_armed = vcontrol_mode.flag_armed;
		}
	}
}

void
Sensors::parameter_update_poll(bool forced)
{
	bool param_updated = false;

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
		char str[30];
		unsigned mag_count = 0;
		unsigned gyro_count = 0;
		unsigned accel_count = 0;

		/* run through all gyro sensors */
		for (unsigned s = 0; s < SENSOR_COUNT_MAX; s++) {

			(void)sprintf(str, "%s%u", GYRO_BASE_DEVICE_PATH, s);

			DevHandle h;
			DevMgr::getHandle(str, h);

			if (!h.isValid()) {
				continue;
			}

			bool config_ok = false;

			/* run through all stored calibrations */
			for (unsigned i = 0; i < SENSOR_COUNT_MAX; i++) {
				/* initially status is ok per config */
				failed = false;

				(void)sprintf(str, "CAL_GYRO%u_ID", i);
				int device_id;
				failed = failed || (OK != param_get(param_find(str), &device_id));

				if (failed) {
					DevMgr::releaseHandle(h);
					continue;
				}

				//int id = h.ioctl(DEVIOCGDEVICEID, 0);
				//PX4_WARN("sensors: device ID: %s: %d, %u", str, id, (unsigned)id);

				/* if the calibration is for this device, apply it */
				if (device_id == h.ioctl(DEVIOCGDEVICEID, 0)) {
					struct gyro_calibration_s gscale = {};
					(void)sprintf(str, "CAL_GYRO%u_XOFF", i);
					failed = failed || (OK != param_get(param_find(str), &gscale.x_offset));
					(void)sprintf(str, "CAL_GYRO%u_YOFF", i);
					failed = failed || (OK != param_get(param_find(str), &gscale.y_offset));
					(void)sprintf(str, "CAL_GYRO%u_ZOFF", i);
					failed = failed || (OK != param_get(param_find(str), &gscale.z_offset));
					(void)sprintf(str, "CAL_GYRO%u_XSCALE", i);
					failed = failed || (OK != param_get(param_find(str), &gscale.x_scale));
					(void)sprintf(str, "CAL_GYRO%u_YSCALE", i);
					failed = failed || (OK != param_get(param_find(str), &gscale.y_scale));
					(void)sprintf(str, "CAL_GYRO%u_ZSCALE", i);
					failed = failed || (OK != param_get(param_find(str), &gscale.z_scale));

					if (failed) {
						warnx(CAL_ERROR_APPLY_CAL_MSG, "gyro", i);

					} else {
						/* apply new scaling and offsets */
						config_ok = apply_gyro_calibration(h, &gscale, device_id);

						if (!config_ok) {
							warnx(CAL_ERROR_APPLY_CAL_MSG, "gyro ", i);
						}
					}

					break;
				}
			}

			if (config_ok) {
				gyro_count++;
			}
		}

		/* run through all accel sensors */
		for (unsigned s = 0; s < SENSOR_COUNT_MAX; s++) {

			(void)sprintf(str, "%s%u", ACCEL_BASE_DEVICE_PATH, s);

			DevHandle h;
			DevMgr::getHandle(str, h);

			if (!h.isValid()) {
				continue;
			}

			bool config_ok = false;

			/* run through all stored calibrations */
			for (unsigned i = 0; i < SENSOR_COUNT_MAX; i++) {
				/* initially status is ok per config */
				failed = false;

				(void)sprintf(str, "CAL_ACC%u_ID", i);
				int device_id;
				failed = failed || (OK != param_get(param_find(str), &device_id));

				if (failed) {
					DevMgr::releaseHandle(h);
					continue;
				}

				// int id = h.ioctl(DEVIOCGDEVICEID, 0);
				// PX4_WARN("sensors: device ID: %s: %d, %u", str, id, (unsigned)id);

				/* if the calibration is for this device, apply it */
				if (device_id == h.ioctl(DEVIOCGDEVICEID, 0)) {
					struct accel_calibration_s ascale = {};
					(void)sprintf(str, "CAL_ACC%u_XOFF", i);
					failed = failed || (OK != param_get(param_find(str), &ascale.x_offset));
					(void)sprintf(str, "CAL_ACC%u_YOFF", i);
					failed = failed || (OK != param_get(param_find(str), &ascale.y_offset));
					(void)sprintf(str, "CAL_ACC%u_ZOFF", i);
					failed = failed || (OK != param_get(param_find(str), &ascale.z_offset));
					(void)sprintf(str, "CAL_ACC%u_XSCALE", i);
					failed = failed || (OK != param_get(param_find(str), &ascale.x_scale));
					(void)sprintf(str, "CAL_ACC%u_YSCALE", i);
					failed = failed || (OK != param_get(param_find(str), &ascale.y_scale));
					(void)sprintf(str, "CAL_ACC%u_ZSCALE", i);
					failed = failed || (OK != param_get(param_find(str), &ascale.z_scale));

					if (failed) {
						warnx(CAL_ERROR_APPLY_CAL_MSG, "accel", i);

					} else {
						/* apply new scaling and offsets */
						config_ok = apply_accel_calibration(h, &ascale, device_id);

						if (!config_ok) {
							warnx(CAL_ERROR_APPLY_CAL_MSG, "accel ", i);
						}
					}

					break;
				}
			}

			if (config_ok) {
				accel_count++;
			}
		}

		/* run through all mag sensors */
		for (unsigned s = 0; s < SENSOR_COUNT_MAX; s++) {

			/* set a valid default rotation (same as board).
			 * if the mag is configured, this might be replaced
			 * in the section below.
			 */
			_mag_rotation[s] = _board_rotation;

			(void)sprintf(str, "%s%u", MAG_BASE_DEVICE_PATH, s);

			DevHandle h;
			DevMgr::getHandle(str, h);

			if (!h.isValid()) {
				/* the driver is not running, abort */
				continue;
			}

			bool config_ok = false;

			/* run through all stored calibrations */
			for (unsigned i = 0; i < SENSOR_COUNT_MAX; i++) {
				/* initially status is ok per config */
				failed = false;

				(void)sprintf(str, "CAL_MAG%u_ID", i);
				int device_id;
				failed = failed || (OK != param_get(param_find(str), &device_id));
				(void)sprintf(str, "CAL_MAG%u_ROT", i);
				(void)param_find(str);

				if (failed) {
					DevMgr::releaseHandle(h);
					continue;
				}

				// int id = h.ioctl(DEVIOCGDEVICEID, 0);
				// PX4_WARN("sensors: device ID: %s: %d, %u", str, id, (unsigned)id);

				/* if the calibration is for this device, apply it */
				if (device_id == h.ioctl(DEVIOCGDEVICEID, 0)) {
					struct mag_calibration_s mscale = {};
					(void)sprintf(str, "CAL_MAG%u_XOFF", i);
					failed = failed || (OK != param_get(param_find(str), &mscale.x_offset));
					(void)sprintf(str, "CAL_MAG%u_YOFF", i);
					failed = failed || (OK != param_get(param_find(str), &mscale.y_offset));
					(void)sprintf(str, "CAL_MAG%u_ZOFF", i);
					failed = failed || (OK != param_get(param_find(str), &mscale.z_offset));
					(void)sprintf(str, "CAL_MAG%u_XSCALE", i);
					failed = failed || (OK != param_get(param_find(str), &mscale.x_scale));
					(void)sprintf(str, "CAL_MAG%u_YSCALE", i);
					failed = failed || (OK != param_get(param_find(str), &mscale.y_scale));
					(void)sprintf(str, "CAL_MAG%u_ZSCALE", i);
					failed = failed || (OK != param_get(param_find(str), &mscale.z_scale));

					(void)sprintf(str, "CAL_MAG%u_ROT", i);

					if (h.ioctl(MAGIOCGEXTERNAL, 0) <= 0) {
						/* mag is internal */
						_mag_rotation[s] = _board_rotation;
						/* reset param to -1 to indicate internal mag */
						int32_t minus_one;
						param_get(param_find(str), &minus_one);

						if (minus_one != MAG_ROT_VAL_INTERNAL) {
							minus_one = MAG_ROT_VAL_INTERNAL;
							param_set_no_notification(param_find(str), &minus_one);
						}

					} else {

						int32_t mag_rot;
						param_get(param_find(str), &mag_rot);

						/* check if this mag is still set as internal */
						if (mag_rot < 0) {
							/* it was marked as internal, change to external with no rotation */
							mag_rot = 0;
							param_set_no_notification(param_find(str), &mag_rot);
						}

						/* handling of old setups, will be removed later (noted Feb 2015) */
						int32_t deprecated_mag_rot = 0;
						param_get(param_find("SENS_EXT_MAG_ROT"), &deprecated_mag_rot);

						/*
						 * If the deprecated parameter is non-default (is != 0),
						 * and the new parameter is default (is == 0), then this board
						 * was configured already and we need to copy the old value
						 * to the new parameter.
						 * The < 0 case is special: It means that this param slot was
						 * used previously by an internal sensor, but the the call above
						 * proved that it is currently occupied by an external sensor.
						 * In that case we consider the orientation to be default as well.
						 */
						if ((deprecated_mag_rot != 0) && (mag_rot <= 0)) {
							mag_rot = deprecated_mag_rot;
							param_set_no_notification(param_find(str), &mag_rot);
							/* clear the old param, not supported in GUI anyway */
							deprecated_mag_rot = 0;
							param_set_no_notification(param_find("SENS_EXT_MAG_ROT"), &deprecated_mag_rot);
						}

						/* handling of transition from internal to external */
						if (mag_rot < 0) {
							mag_rot = 0;
						}

						get_rot_matrix((enum Rotation)mag_rot, &_mag_rotation[s]);
					}

					if (failed) {
						warnx(CAL_ERROR_APPLY_CAL_MSG, "mag", i);

					} else {

						/* apply new scaling and offsets */
						config_ok = apply_mag_calibration(h, &mscale, device_id);

						if (!config_ok) {
							warnx(CAL_ERROR_APPLY_CAL_MSG, "mag ", i);
						}
					}

					break;
				}
			}

			if (config_ok) {
				mag_count++;
			}
		}

		int fd = px4_open(AIRSPEED0_DEVICE_PATH, 0);

		/* this sensor is optional, abort without error */

		if (fd >= 0) {
			struct airspeed_scale airscale = {
				_parameters.diff_pres_offset_pa,
				1.0f,
			};

			if (OK != px4_ioctl(fd, AIRSPEEDIOCSSCALE, (long unsigned int)&airscale)) {
				warn("WARNING: failed to set scale / offsets for airspeed sensor");
			}

			px4_close(fd);
		}

		/* do not output this for now, as its covered in preflight checks */
		// warnx("valid configs: %u gyros, %u mags, %u accels", gyro_count, mag_count, accel_count);
	}
}

bool
Sensors::apply_gyro_calibration(DevHandle &h, const struct gyro_calibration_s *gcal, const int device_id)
{
#ifndef __PX4_QURT

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	const int res = h.ioctl(GYROIOCSSCALE, (long unsigned int)gcal);

	if (res) {
		return false;

	} else {
		return true;
	}

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
Sensors::apply_accel_calibration(DevHandle &h, const struct accel_calibration_s *acal, const int device_id)
{
#ifndef __PX4_QURT

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	const int res = h.ioctl(ACCELIOCSSCALE, (long unsigned int)acal);

	if (res) {
		return false;

	} else {
		return true;
	}

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
Sensors::apply_mag_calibration(DevHandle &h, const struct mag_calibration_s *mcal, const int device_id)
{
#ifndef __PX4_QURT

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	const int res = h.ioctl(MAGIOCSSCALE, (long unsigned int)mcal);

	if (res) {
		return false;

	} else {
		return true;
	}

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

void
Sensors::rc_parameter_map_poll(bool forced)
{
	bool map_updated;
	orb_check(_rc_parameter_map_sub, &map_updated);

	if (map_updated) {
		orb_copy(ORB_ID(rc_parameter_map), _rc_parameter_map_sub, &_rc_parameter_map);

		/* update parameter handles to which the RC channels are mapped */
		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
				/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
				 * or no request to map this channel to a param has been sent via mavlink
				 */
				continue;
			}

			/* Set the handle by index if the index is set, otherwise use the id */
			if (_rc_parameter_map.param_index[i] >= 0) {
				_parameter_handles.rc_param[i] = param_for_used_index((unsigned)_rc_parameter_map.param_index[i]);

			} else {
				_parameter_handles.rc_param[i] = param_find(&_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]);
			}

		}

		warnx("rc to parameter map updated");

		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			warnx("\ti %d param_id %s scale %.3f value0 %.3f, min %.3f, max %.3f",
			      i,
			      &_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)],
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
		int ret = _h_adc.read(&buf_adc, sizeof(buf_adc));

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
						float current = ((buf_adc[i].am_data - _parameters.battery_current_offset) * _parameters.battery_current_scaling);

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

#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL

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
						if (_diff_pres_pub != nullptr) {
							orb_publish(ORB_ID(differential_pressure), _diff_pres_pub, &_diff_pres);

						} else {
							_diff_pres_pub = orb_advertise(ORB_ID(differential_pressure), &_diff_pres);
						}
					}

#endif
				}
			}

			_last_adc = t;

			if (_battery_status.voltage_filtered_v > BATT_V_IGNORE_THRESHOLD) {
				/* announce the battery status if needed, just publish else */
				if (_battery_pub != nullptr) {
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
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
			/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
			 * or no request to map this channel to a param has been sent via mavlink
			 */
			continue;
		}

		float rc_val = get_rc_value((rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i), -1.0, 1.0);

		/* Check if the value has changed,
		 * maybe we need to introduce a more aggressive limit here */
		if (rc_val > _param_rc_values[i] + FLT_EPSILON || rc_val < _param_rc_values[i] - FLT_EPSILON) {
			_param_rc_values[i] = rc_val;
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
			if (!PX4_ISFINITE(_rc.channels[i])) {
				_rc.channels[i] = 0.0f;
			}
		}

		_rc.channel_count = rc_input.channel_count;
		_rc.rssi = rc_input.rssi;
		_rc.signal_lost = signal_lost;
		_rc.timestamp = rc_input.timestamp_last_signal;
		_rc.frame_drop_count = rc_input.rc_lost_frame_count;

		/* publish rc_channels topic even if signal is invalid, for debug */
		if (_rc_pub != nullptr) {
			orb_publish(ORB_ID(rc_channels), _rc_pub, &_rc);

		} else {
			_rc_pub = orb_advertise(ORB_ID(rc_channels), &_rc);
		}

		/* only publish manual control if the signal is still present */
		if (!signal_lost) {

			/* initialize manual setpoint */
			struct manual_control_setpoint_s manual = {};
			/* set mode slot to unassigned */
			manual.mode_slot = manual_control_setpoint_s::MODE_SLOT_NONE;
			/* set the timestamp to the last signal time */
			manual.timestamp = rc_input.timestamp_last_signal;

			/* limit controls */
			manual.y = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_ROLL, -1.0, 1.0);
			manual.x = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_PITCH, -1.0, 1.0);
			manual.r = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_YAW, -1.0, 1.0);
			manual.z = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE, 0.0, 1.0);
			manual.flaps = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS, -1.0, 1.0);
			manual.aux1 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1, -1.0, 1.0);
			manual.aux2 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2, -1.0, 1.0);
			manual.aux3 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3, -1.0, 1.0);
			manual.aux4 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4, -1.0, 1.0);
			manual.aux5 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5, -1.0, 1.0);

			if (_parameters.rc_map_flightmode > 0) {

				/* the number of valid slots equals the index of the max marker minus one */
				const unsigned num_slots = manual_control_setpoint_s::MODE_SLOT_MAX;

				/* the half width of the range of a slot is the total range
				 * divided by the number of slots, again divided by two
				 */
				const float slot_width_half = 2.0f / num_slots / 2.0f;

				/* min is -1, max is +1, range is 2. We offset below min and max */
				const float slot_min = -1.0f - slot_width_half;
				const float slot_max = 1.0f + slot_width_half;

				/* the slot gets mapped by first normalizing into a 0..1 interval using min
				 * and max. Then the right slot is obtained by multiplying with the number of
				 * slots. And finally we add half a slot width to ensure that integer rounding
				 * will take us to the correct final index.
				 */
				manual.mode_slot = (((((_rc.channels[_parameters.rc_map_flightmode - 1] - slot_min) * num_slots) + slot_width_half) /
						     (slot_max - slot_min)) + (1.0f / num_slots));
			}

			/* mode switches */
			manual.mode_switch = get_rc_sw3pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MODE, _parameters.rc_auto_th,
					     _parameters.rc_auto_inv, _parameters.rc_assist_th, _parameters.rc_assist_inv);
			manual.rattitude_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE,
						  _parameters.rc_rattitude_th,
						  _parameters.rc_rattitude_inv);
			manual.posctl_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL, _parameters.rc_posctl_th,
					       _parameters.rc_posctl_inv);
			manual.return_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RETURN, _parameters.rc_return_th,
					       _parameters.rc_return_inv);
			manual.loiter_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_LOITER, _parameters.rc_loiter_th,
					       _parameters.rc_loiter_inv);
			manual.acro_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ACRO, _parameters.rc_acro_th,
					     _parameters.rc_acro_inv);
			manual.offboard_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD,
						 _parameters.rc_offboard_th, _parameters.rc_offboard_inv);
			manual.kill_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH,
					     _parameters.rc_killswitch_th, _parameters.rc_killswitch_inv);

			/* publish manual_control_setpoint topic */
			if (_manual_control_pub != nullptr) {
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
			if (_actuator_group_3_pub != nullptr) {
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

int
Sensors::init_sensor_class(const struct orb_metadata *meta, int *subs,
			   uint32_t *priorities, uint32_t *errcount)
{
	unsigned group_count = orb_group_count(meta);

	if (group_count > SENSOR_COUNT_MAX) {
		group_count = SENSOR_COUNT_MAX;
	}

	for (unsigned i = 0; i < group_count; i++) {
		if (subs[i] < 0) {
			subs[i] = orb_subscribe_multi(meta, i);
			orb_priority(subs[i], (int32_t *)&priorities[i]);
		}
	}

	return group_count;
}

void
Sensors::task_main()
{

	/* start individual sensors */
	int ret = 0;

	/* This calls a sensors_init which can have different implementations on NuttX, POSIX, QURT. */
	ret = sensors_init();

#ifndef __PX4_QURT
	// TODO: move adc_init into the sensors_init call.
	ret = ret || adc_init();
#endif

	if (ret) {
		warnx("sensor initialization failed");
		_sensors_task = -1;

		DevMgr::releaseHandle(_h_adc);

		return;
	}

	struct sensor_combined_s raw = {};

	/* ensure no overflows can occur */
	static_assert((sizeof(raw.gyro_timestamp) / sizeof(raw.gyro_timestamp[0])) >= SENSOR_COUNT_MAX,
		      "SENSOR_COUNT_MAX larger than sensor_combined datastructure fields. Overflow would occur");

	/*
	 * do subscriptions
	 */

	unsigned gcount_prev = _gyro_count;

	unsigned mcount_prev = _mag_count;

	unsigned acount_prev = _accel_count;

	unsigned bcount_prev = _baro_count;

	_gyro_count = init_sensor_class(ORB_ID(sensor_gyro), &_gyro_sub[0],
					&raw.gyro_priority[0], &raw.gyro_errcount[0]);

	_mag_count = init_sensor_class(ORB_ID(sensor_mag), &_mag_sub[0],
				       &raw.magnetometer_priority[0], &raw.magnetometer_errcount[0]);

	_accel_count = init_sensor_class(ORB_ID(sensor_accel), &_accel_sub[0],
					 &raw.accelerometer_priority[0], &raw.accelerometer_errcount[0]);

	_baro_count = init_sensor_class(ORB_ID(sensor_baro), &_baro_sub[0],
					&raw.baro_priority[0], &raw.baro_errcount[0]);

	if (gcount_prev != _gyro_count ||
	    mcount_prev != _mag_count ||
	    acount_prev != _accel_count ||
	    bcount_prev != _baro_count) {

		/* reload calibration params */
		parameter_update_poll(true);
	}

	_rc_sub = orb_subscribe(ORB_ID(input_rc));
	_diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_rc_parameter_map_sub = orb_subscribe(ORB_ID(rc_parameter_map));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/*
	 * do advertisements
	 */
	raw.timestamp = hrt_absolute_time();
	raw.adc_voltage_v[0] = 0.0f;
	raw.adc_voltage_v[1] = 0.0f;
	raw.adc_voltage_v[2] = 0.0f;
	raw.adc_voltage_v[3] = 0.0f;

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
	px4_pollfd_struct_t fds[1] = {};

	/* use the gyro to pace output */
	fds[0].fd = _gyro_sub[0];
	fds[0].events = POLLIN;

	_task_should_exit = false;

	raw.timestamp = 0;

	uint64_t _last_config_update = hrt_absolute_time();

	while (!_task_should_exit) {

		/* wait for up to 50ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		/* if pret == 0 it timed out - periodic check for _task_should_exit, etc. */

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			/* if the polling operation failed because no gyro sensor is available yet,
			 * then attempt to subscribe once again
			 */
			if (_gyro_count == 0) {
				_gyro_count = init_sensor_class(ORB_ID(sensor_gyro), &_gyro_sub[0],
								&raw.gyro_priority[0], &raw.gyro_errcount[0]);
				fds[0].fd = _gyro_sub[0];
			}

			continue;
		}

		perf_begin(_loop_perf);

		/* check vehicle status for changes to publication state */
		vehicle_control_mode_poll();

		/* the timestamp of the raw struct is updated by the gyro_poll() method */
		/* copy most recent sensor data */
		gyro_poll(raw);
		accel_poll(raw);
		mag_poll(raw);
		baro_poll(raw);

		// FIXME TODO: this needs more thinking, otherwise we spam the console and keep switching.
		/* Work out if main gyro timed out and fail over to alternate gyro.
		 * However, don't do this if the secondary is not available. */
		if (hrt_elapsed_time(&raw.gyro_timestamp[0]) > 20 * 1000 && _gyro_sub[1] >= 0) {
			warnx("gyro has timed out");

			/* If the secondary failed as well, go to the tertiary, also only if available. */
			if (hrt_elapsed_time(&raw.gyro_timestamp[1]) > 20 * 1000 && _gyro_sub[2] >= 0) {
				fds[0].fd = _gyro_sub[2];
				warnx("failing over to third gyro");

			} else if (_gyro_sub[1] >= 0) {
				fds[0].fd = _gyro_sub[1];
				warnx("failing over to second gyro");
			}
		}

		/* check battery voltage */
		adc_poll(raw);

		diff_pres_poll(raw);

		/* Inform other processes that new data is available to copy */
		if (_publishing && raw.timestamp > 0) {
			orb_publish(ORB_ID(sensor_combined), _sensor_pub, &raw);
		}

		/* keep adding sensors as long as we are not armed,
		 * when not adding sensors poll for param updates
		 */
		if (!_armed && hrt_elapsed_time(&_last_config_update) > 500 * 1000) {
			_gyro_count = init_sensor_class(ORB_ID(sensor_gyro), &_gyro_sub[0],
							&raw.gyro_priority[0], &raw.gyro_errcount[0]);

			_mag_count = init_sensor_class(ORB_ID(sensor_mag), &_mag_sub[0],
						       &raw.magnetometer_priority[0], &raw.magnetometer_errcount[0]);

			_accel_count = init_sensor_class(ORB_ID(sensor_accel), &_accel_sub[0],
							 &raw.accelerometer_priority[0], &raw.accelerometer_errcount[0]);

			_baro_count = init_sensor_class(ORB_ID(sensor_baro), &_baro_sub[0],
							&raw.baro_priority[0], &raw.baro_errcount[0]);

			_last_config_update = hrt_absolute_time();

		} else {

			/* check parameters for updates */
			parameter_update_poll();

			/* check rc parameter map for updates */
			rc_parameter_map_poll();
		}

		/* Look for new r/c input data */
		rc_poll();

		perf_end(_loop_perf);
	}

	warnx("exiting.");
	_sensors_task = -1;
	px4_task_exit(ret);
}

int
Sensors::start()
{
	ASSERT(_sensors_task == -1);

	/* start the task */
	_sensors_task = px4_task_spawn_cmd("sensors",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   2000,
					   (px4_main_t)&Sensors::task_main_trampoline,
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
	if (argc < 2) {
		warnx("usage: sensors {start|stop|status}");
		return 0;
	}

	if (!strcmp(argv[1], "start")) {

		if (sensors::g_sensors != nullptr) {
			warnx("already running");
			return 0;
		}

		sensors::g_sensors = new Sensors;

		if (sensors::g_sensors == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != sensors::g_sensors->start()) {
			delete sensors::g_sensors;
			sensors::g_sensors = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (sensors::g_sensors == nullptr) {
			warnx("not running");
			return 1;
		}

		delete sensors::g_sensors;
		sensors::g_sensors = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (sensors::g_sensors) {
			warnx("is running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
