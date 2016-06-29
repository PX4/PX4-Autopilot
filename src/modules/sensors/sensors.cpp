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

#include <board_config.h>

#include <px4_adc.h>
#include <px4_config.h>
#include <px4_posix.h>
#include <px4_tasks.h>
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

#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_airspeed.h>
#include <drivers/drv_px4flow.h>

#include <systemlib/airspeed.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/battery.h>

#include <conversion/rotation.h>

#include <lib/ecl/validation/data_validator.h>
#include <lib/ecl/validation/data_validator_group.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/rc_parameter_map.h>
#include <uORB/topics/sensor_preflight.h>

#include <DevMgr.hpp>

#include "sensors_init.h"
#include "parameters.h"
#include "rc_update.h"

using namespace DriverFramework;
using namespace sensors;

/**
 * Analog layout:
 * FMU:
 * IN2 - battery voltage
 * IN3 - battery current
 * IN4 - 5V sense
 * IN10 - spare (we could actually trim these from the set)
 * IN11 - spare on FMUv2 & v3, RC RSSI on FMUv4
 * IN12 - spare (we could actually trim these from the set)
 * IN13 - aux1 on FMUv2, unavaible on v3 & v4
 * IN14 - aux2 on FMUv2, unavaible on v3 & v4
 * IN15 - pressure sensor on FMUv2, unavaible on v3 & v4
 *
 * IO:
 * IN4 - servo supply rail
 * IN5 - analog RSSI on FMUv2 & v3
 *
 * The channel definitions (e.g., ADC_BATTERY_VOLTAGE_CHANNEL, ADC_BATTERY_CURRENT_CHANNEL, and ADC_AIRSPEED_VOLTAGE_CHANNEL) are defined in board_config.h
 */


/**
 * HACK - true temperature is much less than indicated temperature in baro,
 * subtract 5 degrees in an attempt to account for the electrical upheating of the PCB
 */
#define PCB_TEMP_ESTIMATE_DEG		5.0f
#define STICK_ON_OFF_LIMIT		0.75f
#define MAG_ROT_VAL_INTERNAL		-1

#define SENSOR_COUNT_MAX		3

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


	void	print_status();

private:
	/* XXX should not be here - should be own driver */
	DevHandle 	_h_adc;				/**< ADC driver handle */
	hrt_abstime	_last_adc;			/**< last time we took input from the ADC */

	bool 		_task_should_exit;		/**< if true, sensor task should exit */
	int 		_sensors_task;			/**< task handle for sensor task */

	bool		_hil_enabled;			/**< if true, HIL is active */
	bool		_publishing;			/**< if true, we are publishing sensor data (in HIL mode, we don't) */
	bool		_armed;				/**< arming status of the vehicle */

	struct SensorData {
		SensorData()
			: last_best_vote(0),
			  subscription_count(0),
			  voter(1),
			  last_failover_count(0)
		{
			for (unsigned i = 0; i < SENSOR_COUNT_MAX; i++) {
				subscription[i] = -1;
			}
		}

		int subscription[SENSOR_COUNT_MAX]; /**< raw sensor data subscription */
		uint8_t priority[SENSOR_COUNT_MAX]; /**< sensor priority */
		uint8_t last_best_vote; /**< index of the latest best vote */
		int subscription_count;
		DataValidatorGroup voter;
		unsigned int last_failover_count;
	};

	SensorData _gyro;
	SensorData _accel;
	SensorData _mag;
	SensorData _baro;

	int		_actuator_ctrl_0_sub;		/**< attitude controls sub */
	int		_diff_pres_sub;			/**< raw differential pressure subscription */
	int		_vcontrol_mode_sub;		/**< vehicle control mode subscription */
	int 		_params_sub;			/**< notification of parameter updates */

	orb_advert_t	_sensor_pub;			/**< combined sensor data topic */
	orb_advert_t	_battery_pub;			/**< battery status */
	orb_advert_t	_airspeed_pub;			/**< airspeed */
	orb_advert_t	_diff_pres_pub;			/**< differential_pressure */
	orb_advert_t	_mavlink_log_pub;
	orb_advert_t	_sensor_preflight;		/**< sensor preflight topic */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	DataValidator	_airspeed_validator;		/**< data validator to monitor airspeed */

	struct battery_status_s _battery_status;	/**< battery status */
	struct differential_pressure_s _diff_pres;
	struct airspeed_s _airspeed;

	math::Matrix<3, 3>	_board_rotation;	/**< rotation matrix for the orientation that the board is mounted */
	math::Matrix<3, 3>	_mag_rotation[3];	/**< rotation matrix for the orientation that the external mag0 is mounted */

	Battery		_battery;			/**< Helper lib to publish battery_status topic. */

	float _last_baro_pressure[SENSOR_COUNT_MAX]; /**< pressure from last baro sensors */
	float _last_best_baro_pressure; /**< pressure from last best baro */
	sensor_combined_s _last_sensor_data[SENSOR_COUNT_MAX]; /**< latest sensor data from all sensors instances */
	uint64_t _last_accel_timestamp[SENSOR_COUNT_MAX]; /**< latest full timestamp */
	uint64_t _last_mag_timestamp[SENSOR_COUNT_MAX]; /**< latest full timestamp */
	uint64_t _last_baro_timestamp[SENSOR_COUNT_MAX]; /**< latest full timestamp */

	float _accel_diff[3][2];	/**< filtered accel differences between IMU units (m/s/s) */
	float _gyro_diff[3][2];		/**< filtered gyro differences between IMU uinits (rad/s) */

	hrt_abstime _vibration_warning_timestamp;
	bool _vibration_warning;

	Parameters		_parameters;			/**< local copies of interesting parameters */
	ParameterHandles	_parameter_handles;		/**< handles for interesting parameters */

	RCUpdate		_rc_update;


	void	init_sensor_class(const struct orb_metadata *meta, SensorData &sensor_data);

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
	 * Poll the ADC and update readings to suit.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		adc_poll(struct sensor_combined_s &raw);

	/**
	 * Check & handle failover of a sensor
	 * @return true if a switch occured (could be for a non-critical reason)
	 */
	bool check_failover(SensorData &sensor, const char *sensor_name);

	/**
	 * check vibration levels and output a warning if they're high
	 * @return true on high vibration
	 */
	bool check_vibration();

	/**
	 * Calculates the magnitude in m/s/s of the largest difference between the primary and any other accel sensor
	 */
	void calc_accel_inconsistency(sensor_preflight_s &preflt);

	/**
	 * Calculates the magnitude in rad/s of the largest difference between the primary and any other gyro sensor
	 */
	void calc_gyro_inconsistency(struct sensor_preflight_s &preflt);

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
	_vcontrol_mode_sub(-1),
	_params_sub(-1),

	/* publications */
	_sensor_pub(nullptr),
	_battery_pub(nullptr),
	_airspeed_pub(nullptr),
	_diff_pres_pub(nullptr),
	_mavlink_log_pub(nullptr),
	_sensor_preflight(nullptr),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "sensors")),
	_airspeed_validator(),

	_board_rotation{},
	_mag_rotation{},

	_last_best_baro_pressure(0.f),

	_vibration_warning_timestamp(0),
	_vibration_warning(false),

	_rc_update(_parameters)
{
	_baro.voter.set_timeout(300000);
	_mag.voter.set_timeout(300000);

	memset(&_diff_pres, 0, sizeof(_diff_pres));
	memset(&_parameters, 0, sizeof(_parameters));
	memset(&_last_sensor_data, 0, sizeof(_last_sensor_data));
	memset(&_last_accel_timestamp, 0, sizeof(_last_accel_timestamp));
	memset(&_last_mag_timestamp, 0, sizeof(_last_mag_timestamp));
	memset(&_last_baro_timestamp, 0, sizeof(_last_baro_timestamp));
	memset(&_accel_diff, 0, sizeof(_accel_diff));
	memset(&_gyro_diff, 0, sizeof(_gyro_diff));

	/* basic r/c parameters */
	for (unsigned i = 0; i < RC_MAX_CHAN_COUNT; i++) {
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
	_parameter_handles.rc_map_trans_sw = param_find("RC_MAP_TRANS_SW");
	_parameter_handles.rc_map_gear_sw = param_find("RC_MAP_GEAR_SW");

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
	_parameter_handles.rc_trans_th = param_find("RC_TRANS_TH");
	_parameter_handles.rc_gear_th = param_find("RC_GEAR_TH");


	/* Differential pressure offset */
	_parameter_handles.diff_pres_offset_pa = param_find("SENS_DPRES_OFF");
	_parameter_handles.diff_pres_analog_scale = param_find("SENS_DPRES_ANSC");

	_parameter_handles.battery_voltage_scaling = param_find("BAT_CNT_V_VOLT");
	_parameter_handles.battery_current_scaling = param_find("BAT_CNT_V_CURR");
	_parameter_handles.battery_current_offset = param_find("BAT_V_OFFS_CURR");
	_parameter_handles.battery_v_div = param_find("BAT_V_DIV");
	_parameter_handles.battery_a_per_v = param_find("BAT_A_PER_V");
	_parameter_handles.battery_source = param_find("BAT_SOURCE");

	/* rotations */
	_parameter_handles.board_rotation = param_find("SENS_BOARD_ROT");

	/* rotation offsets */
	_parameter_handles.board_offset[0] = param_find("SENS_BOARD_X_OFF");
	_parameter_handles.board_offset[1] = param_find("SENS_BOARD_Y_OFF");
	_parameter_handles.board_offset[2] = param_find("SENS_BOARD_Z_OFF");

	/* Barometer QNH */
	_parameter_handles.baro_qnh = param_find("SENS_BARO_QNH");

	_parameter_handles.vibe_thresh = param_find("ATT_VIBE_THRESH");

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
	(void)param_find("CAL_MAG_SIDES");

	(void)param_find("CAL_MAG1_XOFF");
	(void)param_find("CAL_MAG1_XSCALE");
	(void)param_find("CAL_MAG1_YOFF");
	(void)param_find("CAL_MAG1_YSCALE");
	(void)param_find("CAL_MAG1_ZOFF");
	(void)param_find("CAL_MAG1_ZSCALE");

	(void)param_find("CAL_MAG2_XOFF");
	(void)param_find("CAL_MAG2_XSCALE");
	(void)param_find("CAL_MAG2_YOFF");
	(void)param_find("CAL_MAG2_YSCALE");
	(void)param_find("CAL_MAG2_ZOFF");
	(void)param_find("CAL_MAG2_ZSCALE");

	(void)param_find("CAL_GYRO1_XOFF");
	(void)param_find("CAL_GYRO1_XSCALE");
	(void)param_find("CAL_GYRO1_YOFF");
	(void)param_find("CAL_GYRO1_YSCALE");
	(void)param_find("CAL_GYRO1_ZOFF");
	(void)param_find("CAL_GYRO1_ZSCALE");

	(void)param_find("CAL_GYRO2_XOFF");
	(void)param_find("CAL_GYRO2_XSCALE");
	(void)param_find("CAL_GYRO2_YOFF");
	(void)param_find("CAL_GYRO2_YSCALE");
	(void)param_find("CAL_GYRO2_ZOFF");
	(void)param_find("CAL_GYRO2_ZSCALE");

	(void)param_find("CAL_ACC1_XOFF");
	(void)param_find("CAL_ACC1_XSCALE");
	(void)param_find("CAL_ACC1_YOFF");
	(void)param_find("CAL_ACC1_YSCALE");
	(void)param_find("CAL_ACC1_ZOFF");
	(void)param_find("CAL_ACC1_ZSCALE");

	(void)param_find("CAL_ACC2_XOFF");
	(void)param_find("CAL_ACC2_XSCALE");
	(void)param_find("CAL_ACC2_YOFF");
	(void)param_find("CAL_ACC2_YSCALE");
	(void)param_find("CAL_ACC2_ZOFF");
	(void)param_find("CAL_ACC2_ZSCALE");

	(void)param_find("SYS_PARAM_VER");
	(void)param_find("SYS_AUTOSTART");
	(void)param_find("SYS_AUTOCONFIG");
	(void)param_find("PWM_RATE");
	(void)param_find("PWM_MIN");
	(void)param_find("PWM_MAX");
	(void)param_find("PWM_DISARMED");
	(void)param_find("PWM_AUX_MIN");
	(void)param_find("PWM_AUX_MAX");
	(void)param_find("PWM_AUX_DISARMED");
	(void)param_find("TRIG_MODE");
	(void)param_find("UAVCAN_ENABLE");
	(void)param_find("SYS_MC_EST_GROUP");

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
	int ret = PX4_OK;

	/* rc values */
	for (unsigned int i = 0; i < RC_MAX_CHAN_COUNT; i++) {

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
			PX4_WARN("RC chan %u not sane, scaling: %8.6f, rev: %d", i, (double)tmpScaleFactor, (int)(_parameters.rev[i]));
			/* scaling factors do not make sense, lock them down */
			_parameters.scaling_factor[i] = 0.0f;
			rc_valid = false;

		} else {
			_parameters.scaling_factor[i] = tmpScaleFactor;
		}
	}

	/* handle wrong values */
	if (!rc_valid) {
		PX4_ERR("WARNING     WARNING     WARNING\n\nRC CALIBRATION NOT SANE!\n\n");
	}

	const char *paramerr = "FAIL PARM LOAD";

	/* channel mapping */
	if (param_get(_parameter_handles.rc_map_roll, &(_parameters.rc_map_roll)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_pitch, &(_parameters.rc_map_pitch)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_yaw, &(_parameters.rc_map_yaw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_throttle, &(_parameters.rc_map_throttle)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_failsafe, &(_parameters.rc_map_failsafe)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_mode_sw, &(_parameters.rc_map_mode_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_return_sw, &(_parameters.rc_map_return_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_rattitude_sw, &(_parameters.rc_map_rattitude_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_posctl_sw, &(_parameters.rc_map_posctl_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_loiter_sw, &(_parameters.rc_map_loiter_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_acro_sw, &(_parameters.rc_map_acro_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_offboard_sw, &(_parameters.rc_map_offboard_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_kill_sw, &(_parameters.rc_map_kill_sw)) != OK) {
		PX4_WARN("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_trans_sw, &(_parameters.rc_map_trans_sw)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_gear_sw, &(_parameters.rc_map_gear_sw)) != OK) {
		warnx("%s", paramerr);
	}

	if (param_get(_parameter_handles.rc_map_flaps, &(_parameters.rc_map_flaps)) != OK) {
		PX4_WARN("%s", paramerr);
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
	param_get(_parameter_handles.rc_trans_th, &(_parameters.rc_trans_th));
	_parameters.rc_trans_inv = (_parameters.rc_trans_th < 0);
	_parameters.rc_trans_th = fabs(_parameters.rc_trans_th);
	param_get(_parameter_handles.rc_gear_th, &(_parameters.rc_gear_th));
	_parameters.rc_gear_inv = (_parameters.rc_gear_th < 0);
	_parameters.rc_gear_th = fabs(_parameters.rc_gear_th);

	_rc_update.update_rc_functions();

	/* Airspeed offset */
	param_get(_parameter_handles.diff_pres_offset_pa, &(_parameters.diff_pres_offset_pa));
	param_get(_parameter_handles.diff_pres_analog_scale, &(_parameters.diff_pres_analog_scale));

	/* scaling of ADC ticks to battery voltage */
	if (param_get(_parameter_handles.battery_voltage_scaling, &(_parameters.battery_voltage_scaling)) != OK) {
		PX4_WARN("%s", paramerr);

	} else if (_parameters.battery_voltage_scaling < 0.0f) {
		/* apply scaling according to defaults if set to default */
		_parameters.battery_voltage_scaling = (3.3f / 4096);
		param_set(_parameter_handles.battery_voltage_scaling, &_parameters.battery_voltage_scaling);
	}

	/* scaling of ADC ticks to battery current */
	if (param_get(_parameter_handles.battery_current_scaling, &(_parameters.battery_current_scaling)) != OK) {
		PX4_WARN("%s", paramerr);

	} else if (_parameters.battery_current_scaling < 0.0f) {
		/* apply scaling according to defaults if set to default */
		_parameters.battery_current_scaling = (3.3f / 4096);
		param_set(_parameter_handles.battery_current_scaling, &_parameters.battery_current_scaling);
	}

	if (param_get(_parameter_handles.battery_current_offset, &(_parameters.battery_current_offset)) != OK) {
		PX4_WARN("%s", paramerr);

	}

	if (param_get(_parameter_handles.battery_v_div, &(_parameters.battery_v_div)) != OK) {
		PX4_WARN("%s", paramerr);
		_parameters.battery_v_div = 0.0f;

	} else if (_parameters.battery_v_div <= 0.0f) {
		/* apply scaling according to defaults if set to default */
#if defined (CONFIG_ARCH_BOARD_PX4FMU_V4)
		_parameters.battery_v_div = 13.653333333f;
#elif defined (CONFIG_ARCH_BOARD_PX4FMU_V2) || defined ( CONFIG_ARCH_BOARD_MINDPX_V2 )
		_parameters.battery_v_div = 10.177939394f;
#elif defined (CONFIG_ARCH_BOARD_AEROCORE)
		_parameters.battery_v_div = 7.8196363636f;
#elif defined (CONFIG_ARCH_BOARD_PX4FMU_V1)
		_parameters.battery_v_div = 5.7013919372f;
#elif defined (CONFIG_ARCH_BOARD_SITL)
		_parameters.battery_v_div = 10.177939394f;
#elif defined (CONFIG_ARCH_BOARD_TAP_V1)
		_parameters.battery_v_div = 9.0f;
#else
		/* ensure a missing default trips a low voltage lockdown */
		_parameters.battery_v_div = 0.0f;
#endif
		param_set(_parameter_handles.battery_v_div, &_parameters.battery_v_div);
	}

	if (param_get(_parameter_handles.battery_a_per_v, &(_parameters.battery_a_per_v)) != OK) {
		PX4_WARN("%s", paramerr);
		_parameters.battery_a_per_v = 0.0f;

	} else if (_parameters.battery_a_per_v <= 0.0f) {
		/* apply scaling according to defaults if set to default */
#if defined (CONFIG_ARCH_BOARD_PX4FMU_V4)
		/* current scaling for ACSP4 */
		_parameters.battery_a_per_v = 36.367515152f;
#elif defined (CONFIG_ARCH_BOARD_PX4FMU_V2) || defined (CONFIG_ARCH_BOARD_MINDPX_V2) || defined (CONFIG_ARCH_BOARD_AEROCORE) || defined (CONFIG_ARCH_BOARD_PX4FMU_V1)
		/* current scaling for 3DR power brick */
		_parameters.battery_a_per_v = 15.391030303f;
#elif defined (CONFIG_ARCH_BOARD_SITL)
		_parameters.battery_a_per_v = 15.391030303f;
#else
		/* ensure a missing default leads to an unrealistic current value */
		_parameters.battery_a_per_v = 0.0f;
#endif
		param_set(_parameter_handles.battery_a_per_v, &_parameters.battery_a_per_v);
	}

	param_get(_parameter_handles.battery_source, &(_parameters.battery_source));

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

#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	// TODO: this needs fixing for QURT and Raspberry Pi
	if (!h_baro.isValid()) {
		PX4_ERR("no barometer found on %s (%d)", BARO0_DEVICE_PATH, h_baro.getError());
		ret = PX4_ERROR;

	} else {
		int baroret = h_baro.ioctl(BAROIOCSMSLPRESSURE, (unsigned long)(_parameters.baro_qnh * 100));

		if (baroret) {
			PX4_ERR("qnh for baro could not be set");
			ret = PX4_ERROR;
		}
	}

#endif

	param_get(_parameter_handles.vibe_thresh, &_parameters.vibration_warning_threshold);

	return ret;
}


int
Sensors::adc_init()
{

	DevMgr::getHandle(ADC0_DEVICE_PATH, _h_adc);

	if (!_h_adc.isValid()) {
		PX4_ERR("no ADC found: %s (%d)", ADC0_DEVICE_PATH, _h_adc.getError());
		return PX4_ERROR;
	}

	return OK;
}

void
Sensors::accel_poll(struct sensor_combined_s &raw)
{
	bool got_update = false;

	for (unsigned i = 0; i < _accel.subscription_count; i++) {
		bool accel_updated;
		orb_check(_accel.subscription[i], &accel_updated);

		if (accel_updated) {
			struct accel_report accel_report;

			orb_copy(ORB_ID(sensor_accel), _accel.subscription[i], &accel_report);

			if (accel_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			got_update = true;

			if (accel_report.integral_dt != 0) {
				math::Vector<3> vect_int(accel_report.x_integral, accel_report.y_integral, accel_report.z_integral);
				vect_int = _board_rotation * vect_int;

				float dt = accel_report.integral_dt / 1.e6f;
				_last_sensor_data[i].accelerometer_integral_dt = dt;

				_last_sensor_data[i].accelerometer_m_s2[0] = vect_int(0) / dt;
				_last_sensor_data[i].accelerometer_m_s2[1] = vect_int(1) / dt;
				_last_sensor_data[i].accelerometer_m_s2[2] = vect_int(2) / dt;

			} else {
				//using the value instead of the integral (the integral is the prefered choice)
				math::Vector<3> vect_val(accel_report.x, accel_report.y, accel_report.z);
				vect_val = _board_rotation * vect_val;

				if (_last_accel_timestamp[i] == 0) {
					_last_accel_timestamp[i] = accel_report.timestamp - 1000;
				}

				_last_sensor_data[i].accelerometer_integral_dt =
					(accel_report.timestamp - _last_accel_timestamp[i]) / 1.e6f;
				_last_sensor_data[i].accelerometer_m_s2[0] = vect_val(0);
				_last_sensor_data[i].accelerometer_m_s2[1] = vect_val(1);
				_last_sensor_data[i].accelerometer_m_s2[2] = vect_val(2);
			}

			_last_accel_timestamp[i] = accel_report.timestamp;
			_accel.voter.put(i, accel_report.timestamp, _last_sensor_data[i].accelerometer_m_s2,
					 accel_report.error_count, _accel.priority[i]);
		}
	}

	if (got_update) {
		int best_index;
		_accel.voter.get_best(hrt_absolute_time(), &best_index);

		if (best_index >= 0) {
			raw.accelerometer_m_s2[0] = _last_sensor_data[best_index].accelerometer_m_s2[0];
			raw.accelerometer_m_s2[1] = _last_sensor_data[best_index].accelerometer_m_s2[1];
			raw.accelerometer_m_s2[2] = _last_sensor_data[best_index].accelerometer_m_s2[2];
			raw.accelerometer_integral_dt = _last_sensor_data[best_index].accelerometer_integral_dt;
			_accel.last_best_vote = (uint8_t)best_index;
		}
	}
}

void
Sensors::gyro_poll(struct sensor_combined_s &raw)
{
	bool got_update = false;

	for (unsigned i = 0; i < _gyro.subscription_count; i++) {
		bool gyro_updated;
		orb_check(_gyro.subscription[i], &gyro_updated);

		if (gyro_updated) {
			struct gyro_report gyro_report;

			orb_copy(ORB_ID(sensor_gyro), _gyro.subscription[i], &gyro_report);

			if (gyro_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			got_update = true;

			if (gyro_report.integral_dt != 0) {
				math::Vector<3> vect_int(gyro_report.x_integral, gyro_report.y_integral, gyro_report.z_integral);
				vect_int = _board_rotation * vect_int;

				float dt = gyro_report.integral_dt / 1.e6f;
				_last_sensor_data[i].gyro_integral_dt = dt;

				_last_sensor_data[i].gyro_rad[0] = vect_int(0) / dt;
				_last_sensor_data[i].gyro_rad[1] = vect_int(1) / dt;
				_last_sensor_data[i].gyro_rad[2] = vect_int(2) / dt;

			} else {
				//using the value instead of the integral (the integral is the prefered choice)
				math::Vector<3> vect_val(gyro_report.x, gyro_report.y, gyro_report.z);
				vect_val = _board_rotation * vect_val;

				if (_last_sensor_data[i].timestamp == 0) {
					_last_sensor_data[i].timestamp = gyro_report.timestamp - 1000;
				}

				_last_sensor_data[i].gyro_integral_dt =
					(gyro_report.timestamp - _last_sensor_data[i].timestamp) / 1.e6f;
				_last_sensor_data[i].gyro_rad[0] = vect_val(0);
				_last_sensor_data[i].gyro_rad[1] = vect_val(1);
				_last_sensor_data[i].gyro_rad[2] = vect_val(2);
			}

			_last_sensor_data[i].timestamp = gyro_report.timestamp;
			_gyro.voter.put(i, gyro_report.timestamp, _last_sensor_data[i].gyro_rad,
					gyro_report.error_count, _gyro.priority[i]);
		}
	}

	if (got_update) {
		int best_index;
		_gyro.voter.get_best(hrt_absolute_time(), &best_index);

		if (best_index >= 0) {
			raw.gyro_rad[0] = _last_sensor_data[best_index].gyro_rad[0];
			raw.gyro_rad[1] = _last_sensor_data[best_index].gyro_rad[1];
			raw.gyro_rad[2] = _last_sensor_data[best_index].gyro_rad[2];
			raw.gyro_integral_dt = _last_sensor_data[best_index].gyro_integral_dt;
			raw.timestamp = _last_sensor_data[best_index].timestamp;
			_gyro.last_best_vote = (uint8_t)best_index;
		}
	}
}

void
Sensors::mag_poll(struct sensor_combined_s &raw)
{
	bool got_update = false;

	for (unsigned i = 0; i < _mag.subscription_count; i++) {
		bool mag_updated;
		orb_check(_mag.subscription[i], &mag_updated);

		if (mag_updated) {
			struct mag_report mag_report;

			orb_copy(ORB_ID(sensor_mag), _mag.subscription[i], &mag_report);

			if (mag_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			got_update = true;
			math::Vector<3> vect(mag_report.x, mag_report.y, mag_report.z);
			vect = _mag_rotation[i] * vect;

			_last_sensor_data[i].magnetometer_ga[0] = vect(0);
			_last_sensor_data[i].magnetometer_ga[1] = vect(1);
			_last_sensor_data[i].magnetometer_ga[2] = vect(2);

			_last_mag_timestamp[i] = mag_report.timestamp;
			_mag.voter.put(i, mag_report.timestamp, vect.data,
				       mag_report.error_count, _mag.priority[i]);
		}
	}

	if (got_update) {
		int best_index;
		_mag.voter.get_best(hrt_absolute_time(), &best_index);

		if (best_index >= 0) {
			raw.magnetometer_ga[0] = _last_sensor_data[best_index].magnetometer_ga[0];
			raw.magnetometer_ga[1] = _last_sensor_data[best_index].magnetometer_ga[1];
			raw.magnetometer_ga[2] = _last_sensor_data[best_index].magnetometer_ga[2];
			_mag.last_best_vote = (uint8_t)best_index;
		}
	}
}

void
Sensors::baro_poll(struct sensor_combined_s &raw)
{
	bool got_update = false;

	for (unsigned i = 0; i < _baro.subscription_count; i++) {
		bool baro_updated;
		orb_check(_baro.subscription[i], &baro_updated);

		if (baro_updated) {
			struct baro_report baro_report;

			orb_copy(ORB_ID(sensor_baro), _baro.subscription[i], &baro_report);

			if (baro_report.timestamp == 0) {
				continue; //ignore invalid data
			}

			got_update = true;
			math::Vector<3> vect(baro_report.altitude, 0.f, 0.f);

			_last_sensor_data[i].baro_alt_meter = baro_report.altitude;
			_last_sensor_data[i].baro_temp_celcius = baro_report.temperature;
			_last_baro_pressure[i] = baro_report.pressure;

			_last_baro_timestamp[i] = baro_report.timestamp;
			_baro.voter.put(i, baro_report.timestamp, vect.data,
					baro_report.error_count, _baro.priority[i]);
		}
	}

	if (got_update) {
		int best_index;
		_baro.voter.get_best(hrt_absolute_time(), &best_index);

		if (best_index >= 0) {
			raw.baro_alt_meter = _last_sensor_data[best_index].baro_alt_meter;
			raw.baro_temp_celcius = _last_sensor_data[best_index].baro_temp_celcius;
			_last_best_baro_pressure = _last_baro_pressure[best_index];
			_baro.last_best_vote = (uint8_t)best_index;
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

		float air_temperature_celsius = (_diff_pres.temperature > -300.0f) ? _diff_pres.temperature :
						(raw.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG);

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
							calc_true_airspeed(_diff_pres.differential_pressure_filtered_pa + _last_best_baro_pressure * 1e2f,
									_last_best_baro_pressure * 1e2f, air_temperature_celsius));
		_airspeed.true_airspeed_unfiltered_m_s = math::max(0.0f,
				calc_true_airspeed(_diff_pres.differential_pressure_raw_pa + _last_best_baro_pressure * 1e2f,
						   _last_best_baro_pressure * 1e2f, air_temperature_celsius));

		_airspeed.air_temperature_celsius = air_temperature_celsius;
		_airspeed.differential_pressure_filtered_pa = _diff_pres.differential_pressure_filtered_pa;

		int instance;
		orb_publish_auto(ORB_ID(airspeed), &_airspeed_pub, &_airspeed, &instance, ORB_PRIO_DEFAULT);
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
		_armed = vcontrol_mode.flag_armed;

		/* switching from non-HIL to HIL mode */
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
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "gyro", i);

					} else {
						/* apply new scaling and offsets */
						config_ok = apply_gyro_calibration(h, &gscale, device_id);

						if (!config_ok) {
							PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "gyro ", i);
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
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "accel", i);

					} else {
						/* apply new scaling and offsets */
						config_ok = apply_accel_calibration(h, &ascale, device_id);

						if (!config_ok) {
							PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "accel ", i);
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
						PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "mag", i);

					} else {

						/* apply new scaling and offsets */
						config_ok = apply_mag_calibration(h, &mscale, device_id);

						if (!config_ok) {
							PX4_ERR(CAL_ERROR_APPLY_CAL_MSG, "mag ", i);
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

		_battery.updateParams();
	}
}

bool
Sensors::apply_gyro_calibration(DevHandle &h, const struct gyro_calibration_s *gcal, const int device_id)
{
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(GYROIOCSSCALE, (long unsigned int)gcal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
Sensors::apply_accel_calibration(DevHandle &h, const struct accel_calibration_s *acal, const int device_id)
{
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(ACCELIOCSSCALE, (long unsigned int)acal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
}

bool
Sensors::apply_mag_calibration(DevHandle &h, const struct mag_calibration_s *mcal, const int device_id)
{
#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)

	/* On most systems, we can just use the IOCTL call to set the calibration params. */
	return !h.ioctl(MAGIOCSSCALE, (long unsigned int)mcal);

#else
	/* On QURT, the params are read directly in the respective wrappers. */
	return true;
#endif
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

		float bat_voltage_v = 0.0f;
		float bat_current_a = 0.0f;
		bool updated_battery = false;

		if (ret >= (int)sizeof(buf_adc[0])) {

			/* Read add channels we got */
			for (unsigned i = 0; i < ret / sizeof(buf_adc[0]); i++) {

				/* look for specific channels and process the raw voltage to measurement data */
				if (ADC_BATTERY_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {
					/* Voltage in volts */
					bat_voltage_v = (buf_adc[i].am_data * _parameters.battery_voltage_scaling) * _parameters.battery_v_div;

					if (bat_voltage_v > 0.5f) {
						updated_battery = true;
					}

				} else if (ADC_BATTERY_CURRENT_CHANNEL == buf_adc[i].am_channel) {
					bat_current_a = ((buf_adc[i].am_data * _parameters.battery_current_scaling)
							 - _parameters.battery_current_offset) * _parameters.battery_a_per_v;

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

						int instance;
						orb_publish_auto(ORB_ID(differential_pressure), &_diff_pres_pub, &_diff_pres, &instance,
								 ORB_PRIO_DEFAULT);
					}

#endif
				}
			}

			if (_parameters.battery_source == 0 && updated_battery) {
				actuator_controls_s ctrl;
				orb_copy(ORB_ID(actuator_controls_0), _actuator_ctrl_0_sub, &ctrl);
				_battery.updateBatteryStatus(t, bat_voltage_v, bat_current_a, ctrl.control[actuator_controls_s::INDEX_THROTTLE],
							     _armed, &_battery_status);

				int instance;
				orb_publish_auto(ORB_ID(battery_status), &_battery_pub, &_battery_status, &instance, ORB_PRIO_DEFAULT);
			}

			_last_adc = t;

		}
	}
}

bool
Sensors::check_failover(SensorData &sensor, const char *sensor_name)
{
	if (sensor.last_failover_count != sensor.voter.failover_count()) {

		uint32_t flags = sensor.voter.failover_state();

		if (flags == DataValidator::ERROR_FLAG_NO_ERROR) {
			//we switched due to a non-critical reason. No need to panic.
			PX4_INFO("%s sensor switch from #%i", sensor_name, sensor.voter.failover_index());

		} else {
			mavlink_log_emergency(&_mavlink_log_pub, "%s #%i failover :%s%s%s%s%s!",
					      sensor_name,
					      sensor.voter.failover_index(),
					      ((flags & DataValidator::ERROR_FLAG_NO_DATA) ? " No data" : ""),
					      ((flags & DataValidator::ERROR_FLAG_STALE_DATA) ? " Stale data" : ""),
					      ((flags & DataValidator::ERROR_FLAG_TIMEOUT) ? " Data timeout" : ""),
					      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRCOUNT) ? " High error count" : ""),
					      ((flags & DataValidator::ERROR_FLAG_HIGH_ERRDENSITY) ? " High error density" : ""));
		}

		sensor.last_failover_count = sensor.voter.failover_count();
		return true;
	}

	return false;
}

bool
Sensors::check_vibration()
{
	bool ret = false;
	hrt_abstime cur_time = hrt_absolute_time();

	if (!_vibration_warning && (_gyro.voter.get_vibration_factor(cur_time) > _parameters.vibration_warning_threshold ||
				    _accel.voter.get_vibration_factor(cur_time) > _parameters.vibration_warning_threshold ||
				    _mag.voter.get_vibration_factor(cur_time) > _parameters.vibration_warning_threshold)) {

		if (_vibration_warning_timestamp == 0) {
			_vibration_warning_timestamp = cur_time;

		} else if (hrt_elapsed_time(&_vibration_warning_timestamp) > 10000 * 1000) {
			_vibration_warning = true;
			mavlink_log_critical(&_mavlink_log_pub, "HIGH VIBRATION! g: %d a: %d m: %d",
					     (int)(100 * _gyro.voter.get_vibration_factor(cur_time)),
					     (int)(100 * _accel.voter.get_vibration_factor(cur_time)),
					     (int)(100 * _mag.voter.get_vibration_factor(cur_time)));
			ret = true;
		}

	} else {
		_vibration_warning_timestamp = 0;
	}

	return ret;
}

void
Sensors::calc_accel_inconsistency(sensor_preflight_s &preflt)
{
	// skip check if less than 2 sensors
	if (_accel.subscription_count < 2) {
		preflt.accel_inconsistency_m_s_s = 0.0f;
		return;

	}

	float accel_diff_sum_max_sq = 0.0f; // the maximum sum of axis differences squared
	uint8_t check_index = 0; // the number of sensors the primary has been checked against

	// Check each sensor against the primary
	for (unsigned sensor_index = 0; sensor_index < _accel.subscription_count; sensor_index++) {

		// check that the sensor we are checking against is not the same as the primary
		if (sensor_index != _accel.last_best_vote) {

			float accel_diff_sum_sq = 0.0f; // sum of differences squared for a single sensor comparison agains the primary

			// calculate accel_diff_sum_sq for the specified sensor against the primary
			for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
				_accel_diff[axis_index][check_index] = 0.95f * _accel_diff[axis_index][check_index] + 0.05f *
								       (_last_sensor_data[_accel.last_best_vote].accelerometer_m_s2[axis_index] -
									_last_sensor_data[sensor_index].accelerometer_m_s2[axis_index]);
				accel_diff_sum_sq += _accel_diff[axis_index][check_index] * _accel_diff[axis_index][check_index];

			}

			// capture the largest sum value
			if (accel_diff_sum_sq > accel_diff_sum_max_sq) {
				accel_diff_sum_max_sq = accel_diff_sum_sq;

			}

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;

		}
	}

	// get the vector length of the largest difference and write to the combined sensor struct
	preflt.accel_inconsistency_m_s_s = sqrtf(accel_diff_sum_max_sq);
}

void Sensors::calc_gyro_inconsistency(sensor_preflight_s &preflt)
{
	// skip check if less than 2 sensors
	if (_gyro.subscription_count < 2) {
		preflt.gyro_inconsistency_rad_s = 0.0f;
		return;

	}

	float gyro_diff_sum_max_sq = 0.0f; // the maximum sum of axis differences squared
	uint8_t check_index = 0; // the number of sensors the primary has been checked against


	// Check each sensor against the primary
	for (unsigned sensor_index = 0; sensor_index < _gyro.subscription_count; sensor_index++) {

		// check that the sensor we are checking against is not the same as the primary
		if (sensor_index != _gyro.last_best_vote) {

			float gyro_diff_sum_sq = 0.0f; // sum of differences squared for a single sensor comparison against the primary

			// calculate gyro_diff_sum_sq for the specified sensor against the primary
			for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
				_gyro_diff[axis_index][check_index] = 0.95f * _gyro_diff[axis_index][check_index] + 0.05f *
								      (_last_sensor_data[_gyro.last_best_vote].gyro_rad[axis_index] -
								       _last_sensor_data[sensor_index].gyro_rad[axis_index]);
				gyro_diff_sum_sq += _gyro_diff[axis_index][check_index] * _gyro_diff[axis_index][check_index];

			}

			// capture the largest sum value
			if (gyro_diff_sum_sq > gyro_diff_sum_max_sq) {
				gyro_diff_sum_max_sq = gyro_diff_sum_sq;

			}

			// increment the check index
			check_index++;
		}

		// check to see if the maximum number of checks has been reached and break
		if (check_index >= 2) {
			break;

		}
	}

	// get the vector length of the largest difference and write to the combined sensor struct
	preflt.gyro_inconsistency_rad_s = sqrtf(gyro_diff_sum_max_sq);
}

void
Sensors::task_main_trampoline(int argc, char *argv[])
{
	sensors::g_sensors->task_main();
}

void
Sensors::init_sensor_class(const struct orb_metadata *meta, SensorData &sensor_data)
{
	unsigned group_count = orb_group_count(meta);

	if (group_count > SENSOR_COUNT_MAX) {
		group_count = SENSOR_COUNT_MAX;
	}

	for (unsigned i = 0; i < group_count; i++) {
		if (sensor_data.subscription[i] < 0) {
			sensor_data.subscription[i] = orb_subscribe_multi(meta, i);

			if (i > 0) {
				/* the first always exists, but for each further sensor, add a new validator */
				if (!sensor_data.voter.add_new_validator()) {
					PX4_ERR("failed to add validator for sensor %s %i", meta->o_name, i);
				}
			}
		}

		int32_t priority;
		orb_priority(sensor_data.subscription[i], &priority);
		sensor_data.priority[i] = (uint8_t)priority;
	}

	sensor_data.subscription_count = group_count;
}

void
Sensors::task_main()
{

	/* start individual sensors */
	int ret = 0;

	/* This calls a sensors_init which can have different implementations on NuttX, POSIX, QURT. */
	ret = sensors_init();

#if !defined(__PX4_QURT) && !defined(__PX4_POSIX_RPI) && !defined(__PX4_POSIX_BEBOP)
	// TODO: move adc_init into the sensors_init call.
	ret = ret || adc_init();
#endif

	if (ret) {
		PX4_ERR("sensor initialization failed");
	}

	_rc_update.init();

	struct sensor_combined_s raw = {};

	raw.accelerometer_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;

	raw.magnetometer_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;

	raw.baro_timestamp_relative = sensor_combined_s::RELATIVE_TIMESTAMP_INVALID;

	struct sensor_preflight_s preflt = {};

	/*
	 * do subscriptions
	 */
	init_sensor_class(ORB_ID(sensor_gyro), _gyro);

	init_sensor_class(ORB_ID(sensor_mag), _mag);

	init_sensor_class(ORB_ID(sensor_accel), _accel);

	init_sensor_class(ORB_ID(sensor_baro), _baro);

	_diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));

	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));

	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	_actuator_ctrl_0_sub = orb_subscribe(ORB_ID(actuator_controls_0));

	/* reload calibration params */
	parameter_update_poll(true);

	raw.timestamp = 0;

	_battery.reset(&_battery_status);

	/* get a set of initial values */
	accel_poll(raw);

	gyro_poll(raw);

	mag_poll(raw);

	baro_poll(raw);

	diff_pres_poll(raw);

	_rc_update.rc_parameter_map_poll(_parameter_handles, true /* forced */);

	/* advertise the sensor_combined topic and make the initial publication */
	_sensor_pub = orb_advertise(ORB_ID(sensor_combined), &raw);

	/* advertise the sensor_preflight topic and make the initial publication */
	preflt.accel_inconsistency_m_s_s = 0.0f;

	preflt.gyro_inconsistency_rad_s = 0.0f;

	_sensor_preflight = orb_advertise(ORB_ID(sensor_preflight), &preflt);

	/* wakeup source */
	px4_pollfd_struct_t poll_fds = {};

	poll_fds.events = POLLIN;

	_task_should_exit = false;

	uint64_t last_config_update = hrt_absolute_time();

	while (!_task_should_exit) {

		/* use the best-voted gyro to pace output */
		poll_fds.fd = _gyro.subscription[_gyro.last_best_vote];

		/* wait for up to 50ms for data (Note that this implies, we can have a fail-over time of 50ms,
		 * if a gyro fails) */
		int pret = px4_poll(&poll_fds, 1, 50);

		/* if pret == 0 it timed out - periodic check for _task_should_exit, etc. */

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			/* if the polling operation failed because no gyro sensor is available yet,
			 * then attempt to subscribe once again
			 */
			if (_gyro.subscription_count == 0) {
				init_sensor_class(ORB_ID(sensor_gyro), _gyro);
			}

			usleep(1000);

			continue;
		}

		perf_begin(_loop_perf);

		/* check vehicle status for changes to publication state */
		vehicle_control_mode_poll();

		/* the timestamp of the raw struct is updated by the gyro_poll() method (this makes the gyro
		 * a mandatory sensor) */
		gyro_poll(raw);
		accel_poll(raw);
		mag_poll(raw);
		baro_poll(raw);

		/* check battery voltage */
		adc_poll(raw);

		diff_pres_poll(raw);

		if (_publishing && raw.timestamp > 0) {

			/* construct relative timestamps */
			if (_last_accel_timestamp[_accel.last_best_vote]) {
				raw.accelerometer_timestamp_relative = (int32_t)(_last_accel_timestamp[_accel.last_best_vote] - raw.timestamp);
			}

			if (_last_mag_timestamp[_mag.last_best_vote]) {
				raw.magnetometer_timestamp_relative = (int32_t)(_last_mag_timestamp[_mag.last_best_vote] - raw.timestamp);
			}

			if (_last_baro_timestamp[_baro.last_best_vote]) {
				raw.baro_timestamp_relative = (int32_t)(_last_baro_timestamp[_baro.last_best_vote] - raw.timestamp);
			}

			orb_publish(ORB_ID(sensor_combined), _sensor_pub, &raw);

			check_failover(_accel, "Accel");
			check_failover(_gyro, "Gyro");
			check_failover(_mag, "Mag");
			check_failover(_baro, "Baro");

			/* If the the vehicle is disarmed calculate the length of the maximum difference between
			 * IMU units as a consistency metric and publish to the sensor preflight topic
			*/
			if (!_armed) {
				calc_accel_inconsistency(preflt);
				calc_gyro_inconsistency(preflt);
				orb_publish(ORB_ID(sensor_preflight), _sensor_preflight, &preflt);

			}

			//check_vibration(); //disabled for now, as it does not seem to be reliable
		}

		/* keep adding sensors as long as we are not armed,
		 * when not adding sensors poll for param updates
		 */
		if (!_armed && hrt_elapsed_time(&last_config_update) > 500 * 1000) {
			init_sensor_class(ORB_ID(sensor_gyro), _gyro);
			init_sensor_class(ORB_ID(sensor_mag), _mag);
			init_sensor_class(ORB_ID(sensor_accel), _accel);
			init_sensor_class(ORB_ID(sensor_baro), _baro);
			last_config_update = hrt_absolute_time();

		} else {

			/* check parameters for updates */
			parameter_update_poll();

			/* check rc parameter map for updates */
			_rc_update.rc_parameter_map_poll(_parameter_handles);
		}

		/* Look for new r/c input data */
		_rc_update.rc_poll(_parameter_handles);

		perf_end(_loop_perf);
	}

	for (unsigned i = 0; i < _gyro.subscription_count; i++) {
		orb_unsubscribe(_gyro.subscription[i]);
	}

	for (unsigned i = 0; i < _accel.subscription_count; i++) {
		orb_unsubscribe(_accel.subscription[i]);
	}

	for (unsigned i = 0; i < _mag.subscription_count; i++) {
		orb_unsubscribe(_mag.subscription[i]);
	}

	for (unsigned i = 0; i < _baro.subscription_count; i++) {
		orb_unsubscribe(_baro.subscription[i]);
	}

	orb_unsubscribe(_diff_pres_sub);
	orb_unsubscribe(_vcontrol_mode_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_actuator_ctrl_0_sub);
	orb_unadvertise(_sensor_pub);

	_rc_update.deinit();

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
					   1700,
					   (px4_main_t)&Sensors::task_main_trampoline,
					   nullptr);

	/* wait until the task is up and running or has failed */
	while (_sensors_task > 0 && _task_should_exit) {
		usleep(100);
	}

	if (_sensors_task < 0) {
		return -PX4_ERROR;
	}

	return OK;
}

void Sensors::print_status()
{
	PX4_INFO("gyro status:");
	_gyro.voter.print();
	PX4_INFO("accel status:");
	_accel.voter.print();
	PX4_INFO("mag status:");
	_mag.voter.print();
	PX4_INFO("baro status:");
	_baro.voter.print();
}


int sensors_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("usage: sensors {start|stop|status}");
		return 0;
	}

	if (!strcmp(argv[1], "start")) {

		if (sensors::g_sensors != nullptr) {
			PX4_INFO("already running");
			return 0;
		}

		sensors::g_sensors = new Sensors;

		if (sensors::g_sensors == nullptr) {
			PX4_ERR("alloc failed");
			return 1;
		}

		if (OK != sensors::g_sensors->start()) {
			delete sensors::g_sensors;
			sensors::g_sensors = nullptr;
			PX4_ERR("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (sensors::g_sensors == nullptr) {
			PX4_INFO("not running");
			return 1;
		}

		delete sensors::g_sensors;
		sensors::g_sensors = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (sensors::g_sensors) {
			sensors::g_sensors->print_status();
			return 0;

		} else {
			PX4_INFO("not running");
			return 1;
		}
	}

	PX4_ERR("unrecognized command");
	return 1;
}
