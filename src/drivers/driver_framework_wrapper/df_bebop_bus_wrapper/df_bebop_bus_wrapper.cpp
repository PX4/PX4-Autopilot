/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file df_bebop_bus_wrapper.cpp
 *
 * This is a wrapper around the Parrot Bebop bus driver of the DriverFramework. It sends the
 * motor and contol commands to the Bebop and reads its status and informations.
 */

#include <stdlib.h>
#include <stdint.h>

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>

#include <errno.h>
#include <string.h>
#include <math.h>

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/esc_status.h>

#include <lib/mixer/mixer.h>
#include <lib/mixer/mixer_load.h>
#include <battery/battery.h>

#include <bebop_bus/BebopBus.hpp>
#include <DevMgr.hpp>

extern "C" { __EXPORT int df_bebop_bus_wrapper_main(int argc, char *argv[]); }

using namespace DriverFramework;

class DfBebopBusWrapper : public BebopBus
{
public:
	DfBebopBusWrapper();
	~DfBebopBusWrapper() = default;

	/// Start and initialize the driver
	int start();

	/// Stop the driver
	int stop();

	/// Print various infos (version, type, flights, errors
	int print_info();

	/// Start the motors
	int start_motors();

	/// Stop the motors
	int stop_motors();

	/// Reset pending errors on the Bebop hardware
	int clear_errors();

	/// Set the ESC speeds [front left, front right, back right, back left]
	int set_esc_speeds(const float speed_scaled[4]);

	/// Capture the last throttle value for the battey computation
	void set_last_throttle(float throttle) {_last_throttle = throttle;};

private:
	orb_advert_t _battery_topic;
	orb_advert_t _esc_topic;

	Battery _battery;
	bool _armed;
	float _last_throttle;

	int _battery_orb_class_instance;

	// map for bebop motor index to PX4 motor index
	const uint8_t _esc_map[4] = {0, 2, 3, 1};

	int _publish(struct bebop_state_data &data);
};

DfBebopBusWrapper::DfBebopBusWrapper() :
	BebopBus(BEBOP_BUS_DEVICE_PATH), _battery_topic(nullptr), _esc_topic(nullptr), _battery(), _armed(false),
	_last_throttle(0.0f),
	_battery_orb_class_instance(-1)
{}

int DfBebopBusWrapper::start()
{
	/* Init device and start sensor. */
	int ret = init();

	if (ret != 0) {
		PX4_ERR("BebopBus init fail: %d", ret);
		return ret;
	}

	ret = BebopBus::start();

	if (ret < 0) {
		PX4_ERR("BebopBus start fail: %d", ret);
		return ret;
	}

	// Signal bus start on Bebop
	BebopBus::_play_sound(BebopBus::BOOT);

	return 0;
}

int DfBebopBusWrapper::stop()
{

	int ret = BebopBus::stop();

	if (ret < 0) {
		PX4_ERR("BebopBus stop fail: %d", ret);
		return ret;
	}

	return 0;
}

int DfBebopBusWrapper::print_info()
{
	bebop_bus_info info;
	int ret = _get_info(&info);

	if (ret < 0) {
		return -1;
	}

	PX4_INFO("Bebop Controller Info");
	PX4_INFO("  Software Version: %d.%d", info.version_major, info.version_minor);
	PX4_INFO("  Software Type: %d", info.type);
	PX4_INFO("  Number of controlled motors: %d", info.n_motors_controlled);
	PX4_INFO("  Number of flights: %d", info.n_flights);
	PX4_INFO("  Last flight time: %d", info.last_flight_time);
	PX4_INFO("  Total flight time: %d", info.total_flight_time);
	PX4_INFO("  Last Error: %d\n", info.last_error);

	return 0;
}

int DfBebopBusWrapper::start_motors()
{
	_armed = true;
	return BebopBus::_start_motors();
}

int DfBebopBusWrapper::stop_motors()
{
	_armed = false;
	return BebopBus::_stop_motors();
}

int DfBebopBusWrapper::clear_errors()
{
	return BebopBus::_clear_errors();
}

int DfBebopBusWrapper::set_esc_speeds(const float speed_scaled[4])
{
	return BebopBus::_set_esc_speed(speed_scaled);
}

int DfBebopBusWrapper::_publish(struct bebop_state_data &data)
{

	battery_status_s battery_report;
	const hrt_abstime timestamp = hrt_absolute_time();

	// TODO Check if this is the right way for the Bebop
	// We don't have current measurements
	_battery.updateBatteryStatus(timestamp, data.battery_voltage_v, 0.0, true, true, 0, _last_throttle, _armed,
				     &battery_report);

	esc_status_s esc_status = {};

	uint16_t esc_speed_setpoint_rpm[4] = {};
	BebopBus::_get_esc_speed_setpoint(esc_speed_setpoint_rpm);
	esc_status.timestamp = hrt_absolute_time();
	esc_status.esc_count = 4;

	for (int i = 0; i < 4; i++) {
		esc_status.esc[_esc_map[i]].timestamp = esc_status.timestamp;
		esc_status.esc[_esc_map[i]].esc_rpm = data.rpm[i];
		esc_status.esc[_esc_map[i]].esc_setpoint_raw = esc_speed_setpoint_rpm[i];
	}

	// TODO: when is this ever blocked?
	if (!(m_pub_blocked)) {

		if (_battery_topic == nullptr) {
			_battery_topic = orb_advertise_multi(ORB_ID(battery_status), &battery_report,
							     &_battery_orb_class_instance, ORB_PRIO_LOW);

		} else {
			orb_publish(ORB_ID(battery_status), _battery_topic, &battery_report);
		}

		if (_esc_topic == nullptr) {
			_esc_topic = orb_advertise(ORB_ID(esc_status), &esc_status);

		} else {
			orb_publish(ORB_ID(esc_status), _esc_topic, &esc_status);
		}

	}

	return 0;
}

namespace df_bebop_bus_wrapper
{

DfBebopBusWrapper *g_dev = nullptr;      // interface to the Bebop's I2C device
volatile bool _task_should_exit = false; // flag indicating if bebop esc control task should exit
static bool _is_running = false;         // flag indicating if bebop esc  app is running
static bool _motors_running = false;     // flag indicating if the motors are running
static px4_task_t _task_handle = -1;     // handle to the task main thread

static const char *MIXER_FILENAME = "/home/root/bebop.main.mix";

// subscriptions
int     _controls_sub;
int     _armed_sub;

// publications
orb_advert_t    _outputs_pub = nullptr;
orb_advert_t    _rc_pub = nullptr;

// topic structures
actuator_controls_s _controls[1];
actuator_outputs_s  _outputs;
actuator_armed_s    _armed;

MixerGroup *_mixers = nullptr;

int start();
int stop();
int info();
int clear_errors();
void usage();
void task_main(int argc, char *argv[]);

/* mixers initialization */
int initialize_mixers(const char *mixers_filename);
int mixers_control_callback(uintptr_t handle, uint8_t control_group,
			    uint8_t control_index, float &input);

int mixers_control_callback(uintptr_t handle,
			    uint8_t control_group,
			    uint8_t control_index,
			    float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;

	input = controls[control_group].control[control_index];

	return 0;
}

int initialize_mixers(const char *mixers_filename)
{
	char buf[2048] = {0};
	size_t buflen = sizeof(buf);

	PX4_INFO("Trying to initialize mixers from config file %s", mixers_filename);

	if (load_mixer_file(mixers_filename, &buf[0], sizeof(buf)) < 0) {
		PX4_ERR("can't load mixer: %s", mixers_filename);
		return -1;
	}

	if (_mixers == nullptr) {
		_mixers = new MixerGroup(mixers_control_callback, (uintptr_t)_controls);
	}

	if (_mixers == nullptr) {
		PX4_ERR("No mixers available");
		return -1;

	} else {
		int ret = _mixers->load_from_buf(buf, buflen);

		if (ret != 0) {
			PX4_ERR("Unable to parse mixers file");
			delete _mixers;
			_mixers = nullptr;
			ret = -1;
		}

		return 0;
	}
}

void task_main(int argc, char *argv[])
{
	_is_running = true;

	// Subscribe for orb topics
	_controls_sub = orb_subscribe(ORB_ID(actuator_controls_0));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	// Start disarmed
	_armed.armed = false;
	_armed.prearmed = false;
	_motors_running = false;

	// Set up poll topic
	px4_pollfd_struct_t fds[1];
	fds[0].fd     = _controls_sub;
	fds[0].events = POLLIN;
	/* Don't limit poll intervall for now, 250 Hz should be fine. */
	//orb_set_interval(_controls_sub, 10);

	// Set up mixers
	if (initialize_mixers(MIXER_FILENAME) < 0) {
		PX4_ERR("Mixer initialization failed.");
		return;
	}

	// Main loop
	while (!_task_should_exit) {

		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 10);

		/* Timed out, do a periodic check for _task_should_exit. */
		if (pret == 0) {
			continue;
		}

		/* This is undesirable but not much we can do. */
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(actuator_controls_0), _controls_sub, &_controls[0]);

			_outputs.timestamp = _controls[0].timestamp;


			if (_mixers != nullptr) {
				/* do mixing */
				_outputs.noutputs = _mixers->mix(_outputs.output, 4);
			}

			// Set last throttle for battery calculations
			g_dev->set_last_throttle(_controls[0].control[3]);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = _outputs.noutputs; i < sizeof(_outputs.output) / sizeof(_outputs.output[0]);
			     i++) {
				_outputs.output[i] = NAN;
			}

			// Check if the outputs are in range and rescale
			for (size_t i = 0; i < _outputs.noutputs; ++i) {
				if (i < _outputs.noutputs &&
				    PX4_ISFINITE(_outputs.output[i]) &&
				    _outputs.output[i] >= -1.0f &&
				    _outputs.output[i] <= 1.0f) {
					/* scale for Bebop output 0.0 - 1.0 */
					_outputs.output[i] = (_outputs.output[i] + 1.0f) / 2.0f;

				} else {
					/*
					 * Value is NaN, INF or out of band - set to the minimum value.
					 * This will be clearly visible on the servo status and will limit the risk of accidentally
					 * spinning motors. It would be deadly in flight.
					 */
					_outputs.output[i] = 0.0;
				}
			}

			// Adjust order of BLDCs from PX4 to Bebop
			float motor_out[4];
			motor_out[0] = _outputs.output[2];
			motor_out[1] = _outputs.output[0];
			motor_out[2] = _outputs.output[3];
			motor_out[3] = _outputs.output[1];

			g_dev->set_esc_speeds(motor_out);

			if (_outputs_pub != nullptr) {
				orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);

			} else {
				_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
			}
		}

		bool updated;
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
		}

		const bool lockdown = _armed.manual_lockdown || _armed.lockdown || _armed.force_failsafe;

		// Start the motors if armed but not alreay running
		if (_armed.armed && !lockdown && !_motors_running) {
			g_dev->start_motors();
			_motors_running = true;
		}

		// Stop motors if not armed or killed, but running
		if ((!_armed.armed || lockdown) && _motors_running) {
			g_dev->stop_motors();
			_motors_running = false;
		}
	}

	orb_unsubscribe(_controls_sub);
	orb_unsubscribe(_armed_sub);

	_is_running = false;

}

int start()
{
	g_dev = new DfBebopBusWrapper();

	if (g_dev == nullptr) {
		PX4_ERR("failed instantiating DfBebopBusWrapper object");
		return -1;
	}

	int ret = g_dev->start();

	if (ret != 0) {
		PX4_ERR("DfBebopBusWrapper start failed");
		return ret;
	}

	// Open the Bebop dirver
	DevHandle h;
	DevMgr::getHandle(BEBOP_BUS_DEVICE_PATH, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Error: unable to obtain a valid handle for the receiver at: %s (%d)",
			    BEBOP_BUS_DEVICE_PATH, h.getError());
		return -1;
	}

	DevMgr::releaseHandle(h);

	// Start the task to forward the motor control commands
	_task_handle = px4_task_spawn_cmd("bebop_bus_esc_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  2000,
					  (px4_main_t)&task_main,
					  nullptr);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	_is_running = true;
	return 0;
}

int stop()
{
	// Stop bebop motor control task
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;

	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	// Stop DF device
	int ret = g_dev->stop();

	if (ret != 0) {
		PX4_ERR("driver could not be stopped");
		return ret;
	}

	delete g_dev;
	g_dev = nullptr;
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);

	int ret = g_dev->print_info();

	if (ret != 0) {
		PX4_ERR("Unable to print info");
		return ret;
	}

	return 0;
}

/**
 * Clear errors present on the Bebop hardware
 */
int
clear_errors()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return 1;
	}

	PX4_DEBUG("state @ %p", g_dev);

	int ret = g_dev->clear_errors();

	if (ret != 0) {
		PX4_ERR("Unable to clear errors");
		return ret;
	}

	return 0;
}

void
usage()
{
	PX4_INFO("Usage: df_bebop_bus_wrapper 'start', 'info', 'clear_errors', 'stop'");
}

} /* df_bebop_bus_wrapper */


int
df_bebop_bus_wrapper_main(int argc, char *argv[])
{
	int ret = 0;
	int myoptind = 1;

	if (argc <= 1) {
		df_bebop_bus_wrapper::usage();
		return 1;
	}

	const char *verb = argv[myoptind];


	if (!strcmp(verb, "start")) {
		ret = df_bebop_bus_wrapper::start();
	}

	else if (!strcmp(verb, "stop")) {
		ret = df_bebop_bus_wrapper::stop();
	}

	else if (!strcmp(verb, "info")) {
		ret = df_bebop_bus_wrapper::info();
	}

	else if (!strcmp(verb, "clear_errors")) {
		ret = df_bebop_bus_wrapper::clear_errors();
	}

	else {
		df_bebop_bus_wrapper::usage();
		return 1;
	}

	return ret;
}
