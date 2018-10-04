/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomas@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include <board_config.h>

#include <px4_config.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_getopt.h>
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
#include <drivers/drv_rc_input.h>
#include <drivers/drv_adc.h>
#include <drivers/drv_airspeed.h>

#include <airspeed/airspeed.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <perf/perf_counter.h>
#include <battery/battery.h>

#include <conversion/rotation.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/sensor_preflight.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>

#include <DevMgr.hpp>

#include "parameters.h"
#include "rc_update.h"
#include "voted_sensors_update.h"

using namespace DriverFramework;
using namespace sensors;
using namespace time_literals;

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

/**
 * Sensor app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int sensors_main(int argc, char *argv[]);

class Sensors : public ModuleBase<Sensors>, public ModuleParams
{
public:
	Sensors(bool hil_enabled);
	~Sensors() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static Sensors *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	DevHandle 	_h_adc;				/**< ADC driver handle */

	hrt_abstime	_last_adc{0};			/**< last time we took input from the ADC */

	const bool	_hil_enabled;			/**< if true, HIL is active */
	bool		_armed{false};				/**< arming status of the vehicle */

	int		_actuator_ctrl_0_sub{-1};		/**< attitude controls sub */
	int		_diff_pres_sub{-1};			/**< raw differential pressure subscription */
	int		_vcontrol_mode_sub{-1};		/**< vehicle control mode subscription */
	int 		_params_sub{-1};			/**< notification of parameter updates */

	orb_advert_t	_sensor_pub{nullptr};			/**< combined sensor data topic */
	orb_advert_t	_airdata_pub{nullptr};			/**< combined sensor data topic */
	orb_advert_t	_magnetometer_pub{nullptr};			/**< combined sensor data topic */

#if BOARD_NUMBER_BRICKS > 0
	orb_advert_t	_battery_pub[BOARD_NUMBER_BRICKS] {};			/**< battery status */

	Battery		_battery[BOARD_NUMBER_BRICKS];			/**< Helper lib to publish battery_status topic. */
#endif /* BOARD_NUMBER_BRICKS > 0 */

#if BOARD_NUMBER_BRICKS > 1
	int 			_battery_pub_intance0ndx {0}; /**< track the index of instance 0 */
#endif /* BOARD_NUMBER_BRICKS > 1 */

	orb_advert_t	_airspeed_pub{nullptr};			/**< airspeed */
	orb_advert_t	_sensor_preflight{nullptr};		/**< sensor preflight topic */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	DataValidator	_airspeed_validator;		/**< data validator to monitor airspeed */

#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL
	differential_pressure_s	_diff_pres {};

	orb_advert_t	_diff_pres_pub{nullptr};			/**< differential_pressure */
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */

	Parameters		_parameters{};			/**< local copies of interesting parameters */
	ParameterHandles	_parameter_handles{};		/**< handles for interesting parameters */

	RCUpdate		_rc_update;
	VotedSensorsUpdate _voted_sensors_update;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Do adc-related initialisation.
	 */
	int		adc_init();

	/**
	 * Poll the differential pressure sensor for updated data.
	 *
	 * @param raw			Combined sensor data structure into which
	 *				data should be returned.
	 */
	void		diff_pres_poll(const vehicle_air_data_s &airdata);

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
	void		adc_poll();
};

Sensors::Sensors(bool hil_enabled) :
	ModuleParams(nullptr),
	_hil_enabled(hil_enabled),
	_loop_perf(perf_alloc(PC_ELAPSED, "sensors")),
	_rc_update(_parameters),
	_voted_sensors_update(_parameters, hil_enabled)
{
	initialize_parameter_handles(_parameter_handles);

	_airspeed_validator.set_timeout(300000);
	_airspeed_validator.set_equal_value_threshold(100);

#if BOARD_NUMBER_BRICKS > 0

	for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {
		_battery[b].setParent(this);
	}

#endif /* BOARD_NUMBER_BRICKS > 0 */
}

int
Sensors::parameters_update()
{
	if (_armed) {
		return 0;
	}

	/* read the parameter values into _parameters */
	int ret = update_parameters(_parameter_handles, _parameters);

	if (ret) {
		return ret;
	}

	_rc_update.update_rc_functions();
	_voted_sensors_update.parameters_update();

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
Sensors::diff_pres_poll(const vehicle_air_data_s &raw)
{
	bool updated;
	orb_check(_diff_pres_sub, &updated);

	if (updated) {
		differential_pressure_s diff_pres;
		int ret = orb_copy(ORB_ID(differential_pressure), _diff_pres_sub, &diff_pres);

		if (ret != PX4_OK) {
			return;
		}

		float air_temperature_celsius = (diff_pres.temperature > -300.0f) ? diff_pres.temperature :
						(raw.baro_temp_celcius - PCB_TEMP_ESTIMATE_DEG);

		airspeed_s airspeed;
		airspeed.timestamp = diff_pres.timestamp;

		/* push data into validator */
		float airspeed_input[3] = { diff_pres.differential_pressure_raw_pa, diff_pres.temperature, 0.0f };

		_airspeed_validator.put(airspeed.timestamp, airspeed_input, diff_pres.error_count,
					ORB_PRIO_HIGH);

		airspeed.confidence = _airspeed_validator.confidence(hrt_absolute_time());

		enum AIRSPEED_SENSOR_MODEL smodel;

		switch ((diff_pres.device_id >> 16) & 0xFF) {
		case DRV_DIFF_PRESS_DEVTYPE_SDP31:

		/* fallthrough */
		case DRV_DIFF_PRESS_DEVTYPE_SDP32:

		/* fallthrough */
		case DRV_DIFF_PRESS_DEVTYPE_SDP33:
			/* fallthrough */
			smodel = AIRSPEED_SENSOR_MODEL_SDP3X;
			break;

		default:
			smodel = AIRSPEED_SENSOR_MODEL_MEMBRANE;
			break;
		}

		/* don't risk to feed negative airspeed into the system */
		airspeed.indicated_airspeed_m_s = calc_indicated_airspeed_corrected((enum AIRSPEED_COMPENSATION_MODEL)
						  _parameters.air_cmodel,
						  smodel, _parameters.air_tube_length, _parameters.air_tube_diameter_mm,
						  diff_pres.differential_pressure_filtered_pa, raw.baro_pressure_pa,
						  air_temperature_celsius);

		airspeed.true_airspeed_m_s = calc_true_airspeed_from_indicated(airspeed.indicated_airspeed_m_s, raw.baro_pressure_pa,
					     air_temperature_celsius);

		airspeed.air_temperature_celsius = air_temperature_celsius;

		if (PX4_ISFINITE(airspeed.indicated_airspeed_m_s) && PX4_ISFINITE(airspeed.true_airspeed_m_s)) {
			int instance;
			orb_publish_auto(ORB_ID(airspeed), &_airspeed_pub, &airspeed, &instance, ORB_PRIO_DEFAULT);
		}
	}
}

void
Sensors::vehicle_control_mode_poll()
{
	struct vehicle_control_mode_s vcontrol_mode;
	bool vcontrol_mode_updated;

	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &vcontrol_mode);
		_armed = vcontrol_mode.flag_armed;
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

		parameters_update();
		updateParams();

		/* update airspeed scale */
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
	}
}

void
Sensors::adc_poll()
{
	/* only read if not in HIL mode */
	if (_hil_enabled) {
		return;
	}

	hrt_abstime t = hrt_absolute_time();

	/* rate limit to 100 Hz */
	if (t - _last_adc >= 10000) {
		/* make space for a maximum of twelve channels (to ensure reading all channels at once) */
		px4_adc_msg_t buf_adc[PX4_MAX_ADC_CHANNELS];
		/* read all channels available */
		int ret = _h_adc.read(&buf_adc, sizeof(buf_adc));

#if BOARD_NUMBER_BRICKS > 0
		//todo:abosorb into new class Power

		/* For legacy support we publish the battery_status for the Battery that is
		 * associated with the Brick that is the selected source for VDD_5V_IN
		 * Selection is done in HW ala a LTC4417 or similar, or may be hard coded
		 * Like in the FMUv4
		 */

		/* The ADC channels that  are associated with each brick, in power controller
		 * priority order highest to lowest, as defined by the board config.
		 */
		int   bat_voltage_v_chan[BOARD_NUMBER_BRICKS] = BOARD_BATT_V_LIST;
		int   bat_voltage_i_chan[BOARD_NUMBER_BRICKS] = BOARD_BATT_I_LIST;

		if (_parameters.battery_adc_channel >= 0) {  // overwrite default
			bat_voltage_v_chan[0] = _parameters.battery_adc_channel;
		}

		/* The valid signals (HW dependent) are associated with each brick */
		bool  valid_chan[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;

		/* Per Brick readings with default unread channels at 0 */
		float bat_current_a[BOARD_NUMBER_BRICKS] = {0.0f};
		float bat_voltage_v[BOARD_NUMBER_BRICKS] = {0.0f};

		/* Based on the valid_chan, used to indicate the selected the lowest index
		 * (highest priority) supply that is the source for the VDD_5V_IN
		 * When < 0 none selected
		 */

		int selected_source = -1;

#endif /* BOARD_NUMBER_BRICKS > 0 */

		if (ret >= (int)sizeof(buf_adc[0])) {

			/* Read add channels we got */
			for (unsigned i = 0; i < ret / sizeof(buf_adc[0]); i++) {
#ifdef ADC_AIRSPEED_VOLTAGE_CHANNEL

				if (ADC_AIRSPEED_VOLTAGE_CHANNEL == buf_adc[i].am_channel) {

					/* calculate airspeed, raw is the difference from */
					const float voltage = (float)(buf_adc[i].am_data) * 3.3f / 4096.0f * 2.0f;  // V_ref/4096 * (voltage divider factor)

					/**
					 * The voltage divider pulls the signal down, only act on
					 * a valid voltage from a connected sensor. Also assume a non-
					 * zero offset from the sensor if its connected.
					 */
					if (voltage > 0.4f && (_parameters.diff_pres_analog_scale > 0.0f)) {

						const float diff_pres_pa_raw = voltage * _parameters.diff_pres_analog_scale - _parameters.diff_pres_offset_pa;

						_diff_pres.timestamp = t;
						_diff_pres.differential_pressure_raw_pa = diff_pres_pa_raw;
						_diff_pres.differential_pressure_filtered_pa = (_diff_pres.differential_pressure_filtered_pa * 0.9f) +
								(diff_pres_pa_raw * 0.1f);
						_diff_pres.temperature = -1000.0f;

						int instance;
						orb_publish_auto(ORB_ID(differential_pressure), &_diff_pres_pub, &_diff_pres, &instance, ORB_PRIO_DEFAULT);
					}

				} else
#endif /* ADC_AIRSPEED_VOLTAGE_CHANNEL */
				{

#if BOARD_NUMBER_BRICKS > 0

					for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {

						/* Once we have subscriptions, Do this once for the lowest (highest priority
						 * supply on power controller) that is valid.
						 */
						if (_battery_pub[b] != nullptr && selected_source < 0 && valid_chan[b]) {
							/* Indicate the lowest brick (highest priority supply on power controller)
							 * that is valid as the one that is the selected source for the
							 * VDD_5V_IN
							 */
							selected_source = b;

#if BOARD_NUMBER_BRICKS > 1

							/* Move the selected_source to instance 0 */
							if (_battery_pub_intance0ndx != selected_source) {

								orb_advert_t tmp_h = _battery_pub[_battery_pub_intance0ndx];
								_battery_pub[_battery_pub_intance0ndx] = _battery_pub[selected_source];
								_battery_pub[selected_source] = tmp_h;
								_battery_pub_intance0ndx = selected_source;
							}

#endif /* BOARD_NUMBER_BRICKS > 1 */
						}

						// todo:per brick scaling
						/* look for specific channels and process the raw voltage to measurement data */
						if (bat_voltage_v_chan[b] == buf_adc[i].am_channel) {
							/* Voltage in volts */
							bat_voltage_v[b] = (buf_adc[i].am_data * _parameters.battery_voltage_scaling) * _parameters.battery_v_div;

						} else if (bat_voltage_i_chan[b] == buf_adc[i].am_channel) {
							bat_current_a[b] = ((buf_adc[i].am_data * _parameters.battery_current_scaling)
									    - _parameters.battery_current_offset) * _parameters.battery_a_per_v;
						}
					}

#endif /* BOARD_NUMBER_BRICKS > 0 */
				}
			}

#if BOARD_NUMBER_BRICKS > 0

			if (_parameters.battery_source == 0) {

				for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {

					/* Consider the brick connected if there is a voltage */
					bool connected = bat_voltage_v[b] > BOARD_ADC_OPEN_CIRCUIT_V;

					/* In the case where the BOARD_ADC_OPEN_CIRCUIT_V is
					 * greater than the BOARD_VALID_UV let the HW qualify that it
					 * is connected.
					 */
					if (BOARD_ADC_OPEN_CIRCUIT_V > BOARD_VALID_UV) {
						connected &= valid_chan[b];
					}

					actuator_controls_s ctrl;
					orb_copy(ORB_ID(actuator_controls_0), _actuator_ctrl_0_sub, &ctrl);

					battery_status_s battery_status;
					_battery[b].updateBatteryStatus(t, bat_voltage_v[b], bat_current_a[b],
									connected, selected_source == b, b,
									ctrl.control[actuator_controls_s::INDEX_THROTTLE],
									_armed, &battery_status);
					int instance;
					orb_publish_auto(ORB_ID(battery_status), &_battery_pub[b], &battery_status, &instance, ORB_PRIO_DEFAULT);
				}
			}

#endif /* BOARD_NUMBER_BRICKS > 0 */

			_last_adc = t;
		}
	}
}


void
Sensors::run()
{
	if (!_hil_enabled) {
#if !defined(__PX4_QURT) && BOARD_NUMBER_BRICKS > 0
		adc_init();
#endif
	}

	sensor_combined_s raw = {};
	sensor_preflight_s preflt = {};
	vehicle_air_data_s airdata = {};
	vehicle_magnetometer_s magnetometer = {};

	_rc_update.init();
	_voted_sensors_update.init(raw);

	/* (re)load params and calibration */
	parameter_update_poll(true);

	/*
	 * do subscriptions
	 */
	_diff_pres_sub = orb_subscribe(ORB_ID(differential_pressure));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_actuator_ctrl_0_sub = orb_subscribe(ORB_ID(actuator_controls_0));

	/* get a set of initial values */
	_voted_sensors_update.sensors_poll(raw, airdata, magnetometer);

	diff_pres_poll(airdata);

	_rc_update.rc_parameter_map_poll(_parameter_handles, true /* forced */);

	_sensor_preflight = orb_advertise(ORB_ID(sensor_preflight), &preflt);

	/* wakeup source */
	px4_pollfd_struct_t poll_fds = {};
	poll_fds.events = POLLIN;

	uint64_t last_config_update = hrt_absolute_time();

	while (!should_exit()) {

		/* use the best-voted gyro to pace output */
		poll_fds.fd = _voted_sensors_update.best_gyro_fd();

		/* wait for up to 50ms for data (Note that this implies, we can have a fail-over time of 50ms,
		 * if a gyro fails) */
		int pret = px4_poll(&poll_fds, 1, 50);

		/* if pret == 0 it timed out - periodic check for should_exit(), etc. */

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			/* if the polling operation failed because no gyro sensor is available yet,
			 * then attempt to subscribe once again
			 */
			if (_voted_sensors_update.num_gyros() == 0) {
				_voted_sensors_update.initialize_sensors();
			}

			px4_usleep(1000);

			continue;
		}

		perf_begin(_loop_perf);

		/* check vehicle status for changes to publication state */
		vehicle_control_mode_poll();

		/* the timestamp of the raw struct is updated by the gyro_poll() method (this makes the gyro
		 * a mandatory sensor) */
		const uint64_t airdata_prev_timestamp = airdata.timestamp;
		const uint64_t magnetometer_prev_timestamp = magnetometer.timestamp;

		_voted_sensors_update.sensors_poll(raw, airdata, magnetometer);

		/* check battery voltage */
		adc_poll();

		diff_pres_poll(airdata);

		if (raw.timestamp > 0) {

			_voted_sensors_update.set_relative_timestamps(raw);

			int instance;
			orb_publish_auto(ORB_ID(sensor_combined), &_sensor_pub, &raw, &instance, ORB_PRIO_DEFAULT);

			if (airdata.timestamp != airdata_prev_timestamp) {
				orb_publish_auto(ORB_ID(vehicle_air_data), &_airdata_pub, &airdata, &instance, ORB_PRIO_DEFAULT);
			}

			if (magnetometer.timestamp != magnetometer_prev_timestamp) {
				orb_publish_auto(ORB_ID(vehicle_magnetometer), &_magnetometer_pub, &magnetometer, &instance, ORB_PRIO_DEFAULT);
			}

			_voted_sensors_update.check_failover();

			/* If the the vehicle is disarmed calculate the length of the maximum difference between
			 * IMU units as a consistency metric and publish to the sensor preflight topic
			*/
			if (!_armed) {
				preflt.timestamp = hrt_absolute_time();
				_voted_sensors_update.calc_accel_inconsistency(preflt);
				_voted_sensors_update.calc_gyro_inconsistency(preflt);
				_voted_sensors_update.calc_mag_inconsistency(preflt);
				orb_publish(ORB_ID(sensor_preflight), _sensor_preflight, &preflt);
			}
		}

		/* keep adding sensors as long as we are not armed,
		 * when not adding sensors poll for param updates
		 */
		if (!_armed && hrt_elapsed_time(&last_config_update) > 500_ms) {
			_voted_sensors_update.initialize_sensors();
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

	orb_unsubscribe(_diff_pres_sub);
	orb_unsubscribe(_vcontrol_mode_sub);
	orb_unsubscribe(_params_sub);
	orb_unsubscribe(_actuator_ctrl_0_sub);

	if (_sensor_pub) {
		orb_unadvertise(_sensor_pub);
	}

	if (_airdata_pub) {
		orb_unadvertise(_airdata_pub);
	}

	if (_magnetometer_pub) {
		orb_unadvertise(_magnetometer_pub);
	}

	_rc_update.deinit();
	_voted_sensors_update.deinit();
}

int Sensors::task_spawn(int argc, char *argv[])
{
	/* start the task */
	_task_id = px4_task_spawn_cmd("sensors",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_SENSOR_HUB,
				      2000,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int Sensors::print_status()
{
	_voted_sensors_update.print_status();

	PX4_INFO("Airspeed status:");
	_airspeed_validator.print();

	return 0;
}

int Sensors::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Sensors::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The sensors module is central to the whole system. It takes low-level output from drivers, turns
it into a more usable form, and publishes it for the rest of the system.

The provided functionality includes:
- Read the output from the sensor drivers (`sensor_gyro`, etc.).
  If there are multiple of the same type, do voting and failover handling.
  Then apply the board rotation and temperature calibration (if enabled). And finally publish the data; one of the
  topics is `sensor_combined`, used by many parts of the system.
- Do RC channel mapping: read the raw input channels (`input_rc`), then apply the calibration, map the RC channels
  to the configured channels & mode switches, low-pass filter, and then publish as `rc_channels` and
  `manual_control_setpoint`.
- Read the output from the ADC driver (via ioctl interface) and publish `battery_status`.
- Make sure the sensor drivers get the updated calibration parameters (scale & offset) when the parameters change or
  on startup. The sensor drivers use the ioctl interface for parameter updates. For this to work properly, the
  sensor drivers must already be running when `sensors` is started.
- Do preflight sensor consistency checks and publish the `sensor_preflight` topic.

### Implementation
It runs in its own thread and polls on the currently selected gyro topic.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("sensors", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('h', "Start in HIL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

Sensors *Sensors::instantiate(int argc, char *argv[])
{
	bool hil_enabled = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "h", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'h':
			hil_enabled = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	return new Sensors(hil_enabled);
}

int sensors_main(int argc, char *argv[])
{
	return Sensors::main(argc, argv);
}
