/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include <iostream>
#include <string>
#include <pthread.h>

#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>

#include <drivers/device/qurt/uart.h>

#include <commander/px4_custom_mode.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <mavlink.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/sensor_optical_flow.h>

#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/drivers/device/Device.hpp> // For DeviceId union

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include <uORB/topics/vehicle_control_mode.h>

#include <unistd.h>

#define ASYNC_UART_READ_WAIT_US 2000

extern "C" { __EXPORT int dsp_hitl_main(int argc, char *argv[]); }

namespace dsp_hitl
{

using matrix::wrap_2pi;

static bool _is_running = false;
volatile bool _task_should_exit = false;
static px4_task_t _task_handle = -1;
int _uart_fd = -1;
bool _debug = false;
std::string port = "2";
int baudrate = 921600;
const unsigned mode_flag_custom = 1;
const unsigned mode_flag_armed = 128;
bool _send_mag = false;
bool _send_distance = false;

uORB::Publication<battery_status_s>				_battery_pub{ORB_ID(battery_status)};
uORB::PublicationMulti<sensor_gps_s>			_sensor_gps_pub{ORB_ID(sensor_gps)};
uORB::Publication<vehicle_odometry_s>			_visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};
uORB::Publication<vehicle_odometry_s>			_mocap_odometry_pub{ORB_ID(vehicle_mocap_odometry)};
uORB::PublicationMulti<sensor_baro_s>			_sensor_baro_pub{ORB_ID(sensor_baro)};
uORB::Publication<esc_status_s>					_esc_status_pub{ORB_ID(esc_status)};
uORB::Publication<distance_sensor_s>			_distance_sensor_pub{ORB_ID(distance_sensor)};
uORB::Publication<sensor_optical_flow_s>        _sensor_optical_flow_pub{ORB_ID(sensor_optical_flow)};
uORB::Subscription 								_battery_status_sub{ORB_ID(battery_status)};

int32_t _output_functions[actuator_outputs_s::NUM_ACTUATOR_OUTPUTS] {};

// hil_sensor and hil_state_quaternion
enum SensorSource {
	ACCEL		= 0b111,
	GYRO		= 0b111000,
	MAG		= 0b111000000,
	BARO		= 0b1101000000000,
	DIFF_PRESS	= 0b10000000000
};

PX4Accelerometer *_px4_accel{nullptr};
PX4Gyroscope *_px4_gyro{nullptr};
PX4Magnetometer *_px4_mag{nullptr};

bool got_first_sensor_msg = false;
float x_accel = 0;
float y_accel = 0;
float z_accel = 0;
float x_gyro = 0;
float y_gyro = 0;
float z_gyro = 0;
uint64_t gyro_accel_time = 0;

// Status counters
uint32_t heartbeat_received_counter = 0;
uint32_t heartbeat_sent_counter = 0;
uint32_t imu_counter = 0;
uint32_t hil_sensor_counter = 0;
uint32_t mag_counter = 0;
uint32_t baro_counter = 0;
uint32_t actuator_sent_counter = 0;
uint32_t odometry_received_counter = 0;
uint32_t odometry_sent_counter = 0;
uint32_t gps_received_counter = 0;
uint32_t gps_sent_counter = 0;
uint32_t distance_received_counter = 0;
uint32_t distance_sent_counter = 0;
uint32_t flow_received_counter = 0;
uint32_t flow_sent_counter = 0;
uint32_t unknown_msg_received_counter = 0;

enum class position_source {GPS, VIO, FLOW, NUM_POSITION_SOURCES};

struct position_source_data_s {
	char label[8];
	bool send;
	bool fail;
	uint32_t failure_duration;
	uint64_t failure_duration_start;
} position_source_data[(int) position_source::NUM_POSITION_SOURCES] = {
	{"GPS", false, false, 0, 0},
	{"VIO", false, false, 0, 0},
	{"FLOW", false, false, 0, 0}
};

uint64_t first_sensor_msg_timestamp = 0;
uint64_t first_sensor_report_timestamp = 0;
uint64_t last_sensor_report_timestamp = 0;

vehicle_status_s _vehicle_status{};
vehicle_control_mode_s _control_mode{};
actuator_outputs_s _actuator_outputs{};
battery_status_s _battery_status{};

sensor_accel_fifo_s accel_fifo{};
sensor_gyro_fifo_s gyro_fifo{};

int openPort(const char *dev, speed_t speed);
int closePort();

int readResponse(void *buf, size_t len);
int writeResponse(void *buf, size_t len);

int start(int argc, char *argv[]);
int stop();
void print_status();
void clear_status_counters();
bool isOpen() { return _uart_fd >= 0; };

void usage();
void task_main(int argc, char *argv[]);

void *send_actuator(void *);
void send_actuator_data();

void handle_message_hil_sensor_dsp(mavlink_message_t *msg);
void handle_message_hil_optical_flow(mavlink_message_t *msg);
void handle_message_distance_sensor(mavlink_message_t *msg);
void handle_message_hil_gps_dsp(mavlink_message_t *msg);
void handle_message_odometry_dsp(mavlink_message_t *msg);
void handle_message_command_long_dsp(mavlink_message_t *msg);

void handle_message_dsp(mavlink_message_t *msg);
void actuator_controls_from_outputs_dsp(mavlink_hil_actuator_controls_t *msg);
void send_esc_status(mavlink_hil_actuator_controls_t hil_act_control);

void
handle_message_dsp(mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_HIL_SENSOR:
		hil_sensor_counter++;
		handle_message_hil_sensor_dsp(msg);
		break;

	case MAVLINK_MSG_ID_HIL_GPS:
		gps_received_counter++;

		if (position_source_data[(int) position_source::GPS].send) { handle_message_hil_gps_dsp(msg); }

		break;

	case MAVLINK_MSG_ID_ODOMETRY:
		odometry_received_counter++;

		if (position_source_data[(int) position_source::VIO].send) { handle_message_odometry_dsp(msg); }

		break;

	case MAVLINK_MSG_ID_HEARTBEAT:
		heartbeat_received_counter++;

		if (_debug) { PX4_INFO("Heartbeat msg received"); }

		break;

	case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
		flow_received_counter++;

		if (position_source_data[(int) position_source::FLOW].send) { handle_message_hil_optical_flow(msg); }

		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		distance_received_counter++;

		if (_send_distance) { handle_message_distance_sensor(msg); }

		break;

	default:
		unknown_msg_received_counter++;

		if (_debug) { PX4_INFO("Unknown msg ID: %d", msg->msgid); }

		break;
	}
}

void *send_actuator(void *)
{
	send_actuator_data();
	return nullptr;
}

void send_actuator_data()
{

	int _actuator_outputs_sub = orb_subscribe_multi(ORB_ID(actuator_outputs_sim), 0);
	int _vehicle_control_mode_sub_ = orb_subscribe(ORB_ID(vehicle_control_mode));
	int previous_timestamp = 0;
	int previous_uorb_timestamp = 0;
	int differential = 0;
	bool first_sent = false;

	while (true) {
		bool controls_updated = false;
		(void) orb_check(_vehicle_control_mode_sub_, &controls_updated);

		if (controls_updated) {
			orb_copy(ORB_ID(vehicle_control_mode), _vehicle_control_mode_sub_, &_control_mode);
		}

		bool actuator_updated = false;
		(void) orb_check(_actuator_outputs_sub, &actuator_updated);

		uint8_t  newBuf[512];
		uint16_t newBufLen = 0;

		mavlink_hil_actuator_controls_t hil_act_control;
		actuator_controls_from_outputs_dsp(&hil_act_control);

		mavlink_message_t message{};
		mavlink_msg_hil_actuator_controls_encode(1, 1, &message, &hil_act_control);

		if (actuator_updated) {
			orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);

			if (_actuator_outputs.timestamp > 0) {
				previous_timestamp = _actuator_outputs.timestamp;
				previous_uorb_timestamp = _actuator_outputs.timestamp;

				newBufLen = mavlink_msg_to_send_buffer(newBuf, &message);
				int writeRetval = writeResponse(&newBuf, newBufLen);

				actuator_sent_counter++;

				if (_debug) { PX4_INFO("Succesful write of actuator back to jMAVSim: %d at %llu", writeRetval, hrt_absolute_time()); }

				first_sent = true;

				send_esc_status(hil_act_control);
			}

		} else if (! actuator_updated && first_sent && differential > 4000) {
			previous_timestamp = hrt_absolute_time();

			newBufLen = mavlink_msg_to_send_buffer(newBuf, &message);
			int writeRetval = writeResponse(&newBuf, newBufLen);

			actuator_sent_counter++;

			if (_debug) { PX4_INFO("Succesful write of actuator back to jMAVSim: %d at %llu", writeRetval, hrt_absolute_time()); }

			send_esc_status(hil_act_control);
		}

		differential = hrt_absolute_time() - previous_timestamp;

		px4_usleep(1000);
	}
}

void task_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "odmghfp:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			_debug = true;
			break;

		case 'p':
			port = myoptarg;
			break;

		case 'b':
			baudrate = atoi(myoptarg);
			break;

		case 'm':
			_send_mag = true;
			break;

		case 'g':
			position_source_data[(int) position_source::GPS].send = true;
			break;

		case 'o':
			position_source_data[(int) position_source::VIO].send = true;
			break;

		case 'h':
			_send_distance = true;
			break;

		case 'f':
			position_source_data[(int) position_source::FLOW].send = true;
			break;

		default:
			break;
		}
	}

	const char *charport = port.c_str();
	(void) openPort(charport, (speed_t) baudrate);

	if ((_debug) && (isOpen())) { PX4_INFO("DSP HITL serial port initialized. Baudrate: %d", baudrate); }

	if (! isOpen()) {
		PX4_ERR("DSP HITL failed to open serial port");
		return;
	}

	uint64_t last_heartbeat_timestamp = hrt_absolute_time();
	uint64_t last_imu_update_timestamp = last_heartbeat_timestamp;

	_px4_accel = new PX4Accelerometer(1310988);
	_px4_gyro = new PX4Gyroscope(1310988);
	_px4_mag = new PX4Magnetometer(197388);

	// Create a thread for sending data to the simulator.
	pthread_t sender_thread;
	pthread_attr_t sender_thread_attr;
	pthread_attr_init(&sender_thread_attr);
	pthread_attr_setstacksize(&sender_thread_attr, PX4_STACK_ADJUSTED(8000));
	pthread_create(&sender_thread, &sender_thread_attr, send_actuator, nullptr);
	pthread_attr_destroy(&sender_thread_attr);

	int _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	_is_running = true;

	while (!_task_should_exit) {
		uint8_t rx_buf[1024];

		uint64_t timestamp = hrt_absolute_time();

		// Send out sensor messages every 10ms
		if (got_first_sensor_msg) {
			uint64_t delta_time = timestamp - last_imu_update_timestamp;

			if ((imu_counter) && (delta_time > 15000)) {
				PX4_WARN("Sending updates at %llu, delta %llu", timestamp, delta_time);
			}

			uint64_t _px4_gyro_accel_timestamp = hrt_absolute_time();
			_px4_gyro->update(_px4_gyro_accel_timestamp, x_gyro, y_gyro, z_gyro);
			_px4_accel->update(_px4_gyro_accel_timestamp, x_accel, y_accel, z_accel);
			last_imu_update_timestamp = timestamp;
			imu_counter++;
		}

		// Check for incoming messages from the simulator
		int readRetval = readResponse(&rx_buf[0], sizeof(rx_buf));

		if (readRetval) {
			//Take readRetval and convert it into mavlink msg
			mavlink_message_t msg;
			mavlink_status_t _status{};

			for (int i = 0; i <= readRetval; i++) {
				if (mavlink_parse_char(MAVLINK_COMM_0, rx_buf[i], &msg, &_status)) {
					//PX4_INFO("Value of msg id: %i", msg.msgid);
					handle_message_dsp(&msg);
				}
			}
		}

		if ((timestamp - last_heartbeat_timestamp) > 1000000) {
			mavlink_heartbeat_t hb = {};
			mavlink_message_t hb_message = {};
			hb.autopilot = 12;
			hb.base_mode |= (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) ? 128 : 0;
			mavlink_msg_heartbeat_encode(1, 1, &hb_message, &hb);

			uint8_t  hb_newBuf[MAVLINK_MAX_PACKET_LEN];
			uint16_t hb_newBufLen = 0;
			hb_newBufLen = mavlink_msg_to_send_buffer(hb_newBuf, &hb_message);
			(void) writeResponse(&hb_newBuf, hb_newBufLen);
			last_heartbeat_timestamp = timestamp;
			heartbeat_sent_counter++;
		}

		bool vehicle_updated = false;
		(void) orb_check(_vehicle_status_sub, &vehicle_updated);

		if (vehicle_updated) {
			// PX4_INFO("Value of updated vehicle status: %d", vehicle_updated);
			orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
		}

		uint64_t elapsed_time = hrt_absolute_time() - timestamp;
		// if (elapsed_time < 10000) usleep(10000 - elapsed_time);

		if (elapsed_time < 5000) { usleep(5000 - elapsed_time); }
	}

	_is_running = false;
}

void send_esc_status(mavlink_hil_actuator_controls_t hil_act_control)
{
	esc_status_s esc_status{};
	esc_status.timestamp = hrt_absolute_time();
	const int max_esc_count = math::min(actuator_outputs_s::NUM_ACTUATOR_OUTPUTS, esc_status_s::CONNECTED_ESC_MAX);

	const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	int max_esc_index = 0;
	_battery_status_sub.update(&_battery_status);

	for (int i = 0; i < max_esc_count; i++) {
		if (_output_functions[i] != 0) {
			max_esc_index = i;
		}

		esc_status.esc[i].actuator_function = _output_functions[i]; // TODO: this should be in pwm_sim...
		esc_status.esc[i].timestamp = esc_status.timestamp;
		esc_status.esc[i].esc_errorcount = 0; // TODO
		esc_status.esc[i].esc_voltage = _battery_status.voltage_v;
		esc_status.esc[i].esc_current = armed ? 1.0f + math::abs_t(hil_act_control.controls[i]) * 15.0f :
						0.0f; // TODO: magic number
		esc_status.esc[i].esc_rpm = hil_act_control.controls[i] * 6000;  // TODO: magic number
		esc_status.esc[i].esc_temperature = 20.0 + math::abs_t((double)hil_act_control.controls[i]) * 40.0;
	}

	esc_status.esc_count = max_esc_index + 1;
	esc_status.esc_armed_flags = (1u << esc_status.esc_count) - 1;
	esc_status.esc_online_flags = (1u << esc_status.esc_count) - 1;

	_esc_status_pub.publish(esc_status);
}

void
handle_message_command_long_dsp(mavlink_message_t *msg)
{
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	if (_debug) { PX4_INFO("Value of command_long.command: %d", cmd_mavlink.command); }

	mavlink_command_ack_t ack = {};
	ack.result = MAV_RESULT_UNSUPPORTED;

	mavlink_message_t ack_message = {};
	mavlink_msg_command_ack_encode(1, 1, &ack_message, &ack);

	uint8_t  acknewBuf[512];
	uint16_t acknewBufLen = 0;
	acknewBufLen = mavlink_msg_to_send_buffer(acknewBuf, &ack_message);
	int writeRetval = writeResponse(&acknewBuf, acknewBufLen);

	if (_debug) { PX4_INFO("Succesful write of ACK back over UART: %d at %llu", writeRetval, hrt_absolute_time()); }
}

int	flow_debug_counter = 0;

void
handle_message_hil_optical_flow(mavlink_message_t *msg)
{
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);

	if ((_debug) && (!(flow_debug_counter % 10))) {
		PX4_INFO("optflow: time: %llu, quality %d", flow.time_usec, (int) flow.quality);
		PX4_INFO("optflow: x: %.2f y: %.2f", (double) flow.integrated_x, (double) flow.integrated_y);
	}

	flow_debug_counter++;

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = 1;
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_FLOW_DEVTYPE_SIM;

	sensor_optical_flow_s sensor_optical_flow{};

	sensor_optical_flow.timestamp_sample = hrt_absolute_time();
	sensor_optical_flow.device_id = device_id.devid;

	sensor_optical_flow.pixel_flow[0] = flow.integrated_x;
	sensor_optical_flow.pixel_flow[1] = flow.integrated_y;

	sensor_optical_flow.integration_timespan_us = flow.integration_time_us;
	sensor_optical_flow.quality = flow.quality;

	int index = (int) position_source::FLOW;

	if (position_source_data[index].fail) {
		uint32_t duration = position_source_data[index].failure_duration;
		hrt_abstime start = position_source_data[index].failure_duration_start;

		if (duration) {
			if (hrt_elapsed_time(&start) > (duration * 1000000)) {
				PX4_INFO("Optical flow failure ending");
				position_source_data[index].fail = false;
				position_source_data[index].failure_duration = 0;
				position_source_data[index].failure_duration_start = 0;

			} else {
				sensor_optical_flow.quality = 0;
			}

		} else {
			sensor_optical_flow.quality = 0;
		}
	}

	const matrix::Vector3f integrated_gyro(flow.integrated_xgyro, flow.integrated_ygyro, flow.integrated_zgyro);

	if (integrated_gyro.isAllFinite()) {
		integrated_gyro.copyTo(sensor_optical_flow.delta_angle);
		sensor_optical_flow.delta_angle_available = true;
	}

	sensor_optical_flow.max_flow_rate       = NAN;
	sensor_optical_flow.min_ground_distance = NAN;
	sensor_optical_flow.max_ground_distance = NAN;

	// Use distance value for distance sensor topic
	// if (PX4_ISFINITE(flow.distance) && (flow.distance >= 0.f)) {
	// 	// Positive value (including zero): distance known. Negative value: Unknown distance.
	// 	sensor_optical_flow.distance_m = flow.distance;
	// 	sensor_optical_flow.distance_available = true;
	// }

	// Emulate voxl-flow-server where distance comes in a separate
	// distance sensor topic message
	sensor_optical_flow.distance_m = 0.0f;
	sensor_optical_flow.distance_available = false;

	sensor_optical_flow.timestamp = hrt_absolute_time();

	_sensor_optical_flow_pub.publish(sensor_optical_flow);

	flow_sent_counter++;
}

int distance_debug_counter = 0;

void handle_message_distance_sensor(mavlink_message_t *msg)
{
	mavlink_distance_sensor_t dist_sensor;
	mavlink_msg_distance_sensor_decode(msg, &dist_sensor);

	if ((_debug) && (!(distance_debug_counter % 10))) {
		PX4_INFO("distance: time: %u, quality: %u, height: %u",
			 dist_sensor.time_boot_ms, dist_sensor.signal_quality,
			 dist_sensor.current_distance);
	}

	distance_debug_counter++;

	distance_sensor_s ds{};

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = 1;
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_MAVLINK;

	ds.timestamp        = hrt_absolute_time(); /* Use system time for now, don't trust sender to attach correct timestamp */
	ds.min_distance     = static_cast<float>(dist_sensor.min_distance) * 1e-2f;     /* cm to m */
	ds.max_distance     = static_cast<float>(dist_sensor.max_distance) * 1e-2f;     /* cm to m */
	ds.current_distance = static_cast<float>(dist_sensor.current_distance) * 1e-2f; /* cm to m */
	ds.variance         = dist_sensor.covariance * 1e-4f;                           /* cm^2 to m^2 */
	ds.h_fov            = dist_sensor.horizontal_fov;
	ds.v_fov            = dist_sensor.vertical_fov;
	ds.q[0]             = dist_sensor.quaternion[0];
	ds.q[1]             = dist_sensor.quaternion[1];
	ds.q[2]             = dist_sensor.quaternion[2];
	ds.q[3]             = dist_sensor.quaternion[3];
	ds.type             = dist_sensor.type;
	ds.device_id        = device_id.devid;
	ds.orientation      = dist_sensor.orientation;

	// MAVLink DISTANCE_SENSOR signal_quality value of 0 means unset/unknown
	// quality value. Also it comes normalised between 1 and 100 while the uORB
	// signal quality is normalised between 0 and 100.
	ds.signal_quality = dist_sensor.signal_quality == 0 ? -1 : 100 * (dist_sensor.signal_quality - 1) / 99;

	_distance_sensor_pub.publish(ds);

	distance_sent_counter++;
}

void
handle_message_odometry_dsp(mavlink_message_t *msg)
{
	mavlink_odometry_t odom_in;
	mavlink_msg_odometry_decode(msg, &odom_in);

	odometry_sent_counter++;

	// fill vehicle_odometry from Mavlink ODOMETRY
	vehicle_odometry_s odom{};
	uint64_t timestamp = hrt_absolute_time();
	odom.timestamp_sample = timestamp;

	const matrix::Vector3f odom_in_p(odom_in.x, odom_in.y, odom_in.z);

	// position x/y/z (m)
	if (odom_in_p.isAllFinite()) {
		// frame_id: Coordinate frame of reference for the pose data.
		switch (odom_in.frame_id) {
		case MAV_FRAME_LOCAL_NED:
			// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom_in_p.copyTo(odom.position);
			break;

		case MAV_FRAME_LOCAL_ENU:
			// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom.position[0] =  odom_in.y; // y: North
			odom.position[1] =  odom_in.x; // x: East
			odom.position[2] = -odom_in.z; // z: Up
			break;

		case MAV_FRAME_LOCAL_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
			odom_in_p.copyTo(odom.position);
			break;

		case MAV_FRAME_LOCAL_FLU:
			// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
			odom.position[0] =  odom_in.x; // x: Forward
			odom.position[1] = -odom_in.y; // y: Left
			odom.position[2] = -odom_in.z; // z: Up
			break;

		default:
			break;
		}

		// pose_covariance
		//  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw)
		//  first six entries are the first ROW, next five entries are the second ROW, etc.
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			switch (odom_in.frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
			case MAV_FRAME_LOCAL_FLU:
				// position variances copied directly
				odom.position_variance[0] = odom_in.pose_covariance[0];  // X  row 0, col 0
				odom.position_variance[1] = odom_in.pose_covariance[6];  // Y  row 1, col 1
				odom.position_variance[2] = odom_in.pose_covariance[11]; // Z  row 2, col 2
				break;

			case MAV_FRAME_LOCAL_ENU:
				// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
				odom.position_variance[0] = odom_in.pose_covariance[6];  // Y  row 1, col 1
				odom.position_variance[1] = odom_in.pose_covariance[0];  // X  row 0, col 0
				odom.position_variance[2] = odom_in.pose_covariance[11]; // Z  row 2, col 2
				break;

			default:
				break;
			}
		}
	}

	// q: the quaternion of the ODOMETRY msg represents a rotation from body frame to a local frame
	if (matrix::Quatf(odom_in.q).isAllFinite()) {

		odom.q[0] = odom_in.q[0];
		odom.q[1] = odom_in.q[1];
		odom.q[2] = odom_in.q[2];
		odom.q[3] = odom_in.q[3];

		// pose_covariance (roll, pitch, yaw)
		//  states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.
		//  TODO: fix pose_covariance for MAV_FRAME_LOCAL_ENU, MAV_FRAME_LOCAL_FLU
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			odom.orientation_variance[0] = odom_in.pose_covariance[15]; // R  row 3, col 3
			odom.orientation_variance[1] = odom_in.pose_covariance[18]; // P  row 4, col 4
			odom.orientation_variance[2] = odom_in.pose_covariance[20]; // Y  row 5, col 5
		}
	}

	const matrix::Vector3f odom_in_v(odom_in.vx, odom_in.vy, odom_in.vz);

	// velocity vx/vy/vz (m/s)
	if (odom_in_v.isAllFinite()) {
		// child_frame_id: Coordinate frame of reference for the velocity in free space (twist) data.
		switch (odom_in.child_frame_id) {
		case MAV_FRAME_LOCAL_NED:
			// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
			odom_in_v.copyTo(odom.velocity);
			break;

		case MAV_FRAME_LOCAL_ENU:
			// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
			odom.velocity[0] =  odom_in.vy; // y: East
			odom.velocity[1] =  odom_in.vx; // x: North
			odom.velocity[2] = -odom_in.vz; // z: Up
			break;

		case MAV_FRAME_LOCAL_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
			odom_in_v.copyTo(odom.velocity);
			break;

		case MAV_FRAME_LOCAL_FLU:
			// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
			odom.velocity[0] =  odom_in.vx; // x: Forward
			odom.velocity[1] = -odom_in.vy; // y: Left
			odom.velocity[2] = -odom_in.vz; // z: Up
			break;

		case MAV_FRAME_BODY_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
		case MAV_FRAME_BODY_OFFSET_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
		case MAV_FRAME_BODY_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
			odom.velocity[0] = odom_in.vx;
			odom.velocity[1] = odom_in.vy;
			odom.velocity[2] = odom_in.vz;
			break;

		default:
			// unsupported child_frame_id
			break;
		}

		// velocity_covariance (vx, vy, vz)
		//  states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.
		//  TODO: fix velocity_covariance for MAV_FRAME_LOCAL_ENU, MAV_FRAME_LOCAL_FLU, MAV_FRAME_LOCAL_FLU
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			switch (odom_in.child_frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
			case MAV_FRAME_LOCAL_FLU:
			case MAV_FRAME_BODY_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
			case MAV_FRAME_BODY_OFFSET_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
			case MAV_FRAME_BODY_FRD:
				// velocity covariances copied directly
				odom.velocity_variance[0] = odom_in.velocity_covariance[0];  // X  row 0, col 0
				odom.velocity_variance[1] = odom_in.velocity_covariance[6];  // Y  row 1, col 1
				odom.velocity_variance[2] = odom_in.velocity_covariance[11]; // Z  row 2, col 2
				break;

			case MAV_FRAME_LOCAL_ENU:
				// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
				odom.velocity_variance[0] = odom_in.velocity_covariance[6];  // Y  row 1, col 1
				odom.velocity_variance[1] = odom_in.velocity_covariance[0];  // X  row 0, col 0
				odom.velocity_variance[2] = odom_in.velocity_covariance[11]; // Z  row 2, col 2
				break;

			default:
				// unsupported child_frame_id
				break;
			}
		}
	}

	// Roll/Pitch/Yaw angular speed (rad/s)
	if (PX4_ISFINITE(odom_in.rollspeed)
	    && PX4_ISFINITE(odom_in.pitchspeed)
	    && PX4_ISFINITE(odom_in.yawspeed)) {

		odom.angular_velocity[0] = odom_in.rollspeed;
		odom.angular_velocity[1] = odom_in.pitchspeed;
		odom.angular_velocity[2] = odom_in.yawspeed;
	}

	odom.reset_counter = odom_in.reset_counter;
	odom.quality = odom_in.quality;

	int index = (int) position_source::VIO;

	if (position_source_data[index].fail) {
		uint32_t duration = position_source_data[index].failure_duration;
		hrt_abstime start = position_source_data[index].failure_duration_start;

		if (duration) {
			if (hrt_elapsed_time(&start) > (duration * 1000000)) {
				PX4_INFO("VIO failure ending");
				position_source_data[index].fail = false;
				position_source_data[index].failure_duration = 0;
				position_source_data[index].failure_duration_start = 0;

			} else {
				odom.quality = 0;
			}

		} else {
			odom.quality = 0;
		}
	}

	switch (odom_in.estimator_type) {
	case MAV_ESTIMATOR_TYPE_UNKNOWN: // accept MAV_ESTIMATOR_TYPE_UNKNOWN for legacy support
	case MAV_ESTIMATOR_TYPE_NAIVE:
	case MAV_ESTIMATOR_TYPE_VISION:
	case MAV_ESTIMATOR_TYPE_VIO:
		odom.timestamp = hrt_absolute_time();
		_visual_odometry_pub.publish(odom);
		break;

	case MAV_ESTIMATOR_TYPE_MOCAP:
		odom.timestamp = hrt_absolute_time();
		_mocap_odometry_pub.publish(odom);
		break;

	case MAV_ESTIMATOR_TYPE_GPS:
	case MAV_ESTIMATOR_TYPE_GPS_INS:
	case MAV_ESTIMATOR_TYPE_LIDAR:
	case MAV_ESTIMATOR_TYPE_AUTOPILOT:
	default:
		//mavlink_log_critical(&_mavlink_log_pub, "ODOMETRY: estimator_type %" PRIu8 " unsupported\t",
		//		     odom_in.estimator_type);
		//events::send<uint8_t>(events::ID("mavlink_rcv_odom_unsup_estimator_type"), events::Log::Error,
		//		      "ODOMETRY: unsupported estimator_type {1}", odom_in.estimator_type);
		return;
	}
}

void actuator_controls_from_outputs_dsp(mavlink_hil_actuator_controls_t *msg)
{
	memset(msg, 0, sizeof(mavlink_hil_actuator_controls_t));

	msg->time_usec = hrt_absolute_time();

	bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);

	if (armed) {
		for (unsigned i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
			msg->controls[i] = _actuator_outputs.output[i];
		}

		//PX4_INFO("Value of actuator data: %f, %f, %f, %f", (double)msg->controls[0], (double)msg->controls[1], (double)msg->controls[2], (double)msg->controls[3]);
	}

	msg->mode = mode_flag_custom;
	msg->mode |= (armed) ? mode_flag_armed : 0;
	msg->flags = 0;
}

int openPort(const char *dev, speed_t speed)
{
	if (_uart_fd >= 0) {
		PX4_ERR("Port in use: %s (%i)", dev, errno);
		return -1;
	}

	_uart_fd = qurt_uart_open(dev, speed);

	if (_debug) { PX4_INFO("qurt_uart_opened"); }

	if (_uart_fd < 0) {
		PX4_ERR("Error opening port: %s (%i)", dev, errno);
		return -1;
	}

	return 0;
}

int closePort()
{
	_uart_fd = -1;

	return 0;
}

int readResponse(void *buf, size_t len)
{
	if (_uart_fd < 0 || buf == NULL) {
		PX4_ERR("invalid state for reading or buffer");
		return -1;
	}

	return qurt_uart_read(_uart_fd, (char *) buf, len, ASYNC_UART_READ_WAIT_US);
}

int writeResponse(void *buf, size_t len)
{
	if (_uart_fd < 0 || buf == NULL) {
		PX4_ERR("invalid state for writing or buffer");
		return -1;
	}

	return qurt_uart_write(_uart_fd, (const char *) buf, len);
}

int start(int argc, char *argv[])
{
	if (_is_running) {
		PX4_WARN("already running");
		return -1;
	}

	_task_should_exit = false;

	_task_handle = px4_task_spawn_cmd("dsp_hitl__main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_DEFAULT,
					  2000,
					  (px4_main_t)&task_main,
					  (char *const *)argv);

	if (_task_handle < 0) {
		PX4_ERR("task start failed");
		return -1;
	}

	return 0;
}

int stop()
{
	if (!_is_running) {
		PX4_WARN("not running");
		return -1;
	}

	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
	return 0;
}

void usage()
{
	PX4_INFO("Usage: dsp_hitl {start|status|clear|failure|stop}");
	PX4_INFO("       failure  <source> <duration>");
	PX4_INFO("                source: gps, vio, flow");
	PX4_INFO("                duration: 0 (toggle state), else seconds");
}

void print_status()
{
	PX4_INFO("Running: %s", _is_running ? "yes" : "no");
	PX4_INFO("HIL Sensor received: %i", hil_sensor_counter);
	PX4_INFO("IMU updates: %i", imu_counter);
	PX4_INFO("\tCurrent accel x, y, z: %f, %f, %f", double(x_accel), double(y_accel), double(z_accel));
	PX4_INFO("\tCurrent gyro x, y, z: %f, %f, %f", double(x_gyro), double(y_gyro), double(z_gyro));
	PX4_INFO("Magnetometer sent: %i", mag_counter);
	PX4_INFO("Barometer sent: %i", baro_counter);
	PX4_INFO("Heartbeat received: %i, sent: %i", heartbeat_received_counter, heartbeat_sent_counter);
	PX4_INFO("Odometry received: %i, sent: %i", odometry_received_counter, odometry_sent_counter);
	PX4_INFO("GPS received: %i, sent: %i", gps_received_counter, gps_sent_counter);
	PX4_INFO("Distance sensor received: %i, sent: %i", distance_received_counter, distance_sent_counter);
	PX4_INFO("Optical flow received: %i, sent: %i", flow_received_counter, flow_sent_counter);
	PX4_INFO("Actuator updates sent: %i", actuator_sent_counter);
	PX4_INFO("Unknown messages received: %i", unknown_msg_received_counter);
}

void
clear_status_counters()
{
	heartbeat_received_counter = 0;
	heartbeat_sent_counter = 0;
	imu_counter = 0;
	hil_sensor_counter = 0;
	mag_counter = 0;
	baro_counter = 0;
	actuator_sent_counter = 0;
	odometry_received_counter = 0;
	odometry_sent_counter = 0;
	gps_received_counter = 0;
	gps_sent_counter = 0;
	distance_received_counter = 0;
	distance_sent_counter = 0;
	flow_received_counter = 0;
	flow_sent_counter = 0;
	unknown_msg_received_counter = 0;
}

void
handle_message_hil_sensor_dsp(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t hil_sensor;
	mavlink_msg_hil_sensor_decode(msg, &hil_sensor);

	// temperature only updated with baro
	gyro_accel_time = hrt_absolute_time();

	// temperature only updated with baro
	float temperature = NAN;

	got_first_sensor_msg = true;

	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		temperature = hil_sensor.temperature;
	}

	// gyro
	if ((hil_sensor.fields_updated & SensorSource::GYRO) == SensorSource::GYRO) {
		if (_px4_gyro == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_gyro = new PX4Gyroscope(1310988);
		}

		if (_px4_gyro != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_gyro->set_temperature(temperature);
			}

			x_gyro = hil_sensor.xgyro;
			y_gyro = hil_sensor.ygyro;
			z_gyro = hil_sensor.zgyro;
		}
	}

	// accelerometer
	if ((hil_sensor.fields_updated & SensorSource::ACCEL) == SensorSource::ACCEL) {
		if (_px4_accel == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_accel = new PX4Accelerometer(1310988);
		}

		if (_px4_accel != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_accel->set_temperature(temperature);
			}

			x_accel = hil_sensor.xacc;
			y_accel = hil_sensor.yacc;
			z_accel = hil_sensor.zacc;
		}
	}


	// magnetometer
	if ((_send_mag) && ((hil_sensor.fields_updated & SensorSource::MAG) == SensorSource::MAG)) {
		if (_px4_mag == nullptr) {
			// 197388: DRV_MAG_DEVTYPE_MAGSIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
			_px4_mag = new PX4Magnetometer(197388);
		}

		if (_px4_mag != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_mag->set_temperature(temperature);
			}

			_px4_mag->update(gyro_accel_time, hil_sensor.xmag, hil_sensor.ymag, hil_sensor.zmag);

			mag_counter++;
		}
	}

	// baro
	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		// publish
		sensor_baro_s sensor_baro{};
		sensor_baro.timestamp_sample = gyro_accel_time;
		sensor_baro.device_id = 6620172; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
		sensor_baro.pressure = hil_sensor.abs_pressure * 100.0f; // hPa to Pa
		sensor_baro.temperature = hil_sensor.temperature;
		sensor_baro.error_count = 0;
		sensor_baro.timestamp = hrt_absolute_time();
		_sensor_baro_pub.publish(sensor_baro);

		baro_counter++;
	}

	// battery status
	{
		battery_status_s hil_battery_status{};

		hil_battery_status.timestamp = gyro_accel_time;
		hil_battery_status.voltage_v = 16.0f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;
		hil_battery_status.connected = true;
		hil_battery_status.remaining = 0.70;
		hil_battery_status.time_remaining_s = NAN;

		_battery_pub.publish(hil_battery_status);
	}
}

void
handle_message_hil_gps_dsp(mavlink_message_t *msg)
{
	mavlink_hil_gps_t hil_gps;
	mavlink_msg_hil_gps_decode(msg, &hil_gps);

	sensor_gps_s gps{};

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = 1;
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;

	gps.device_id = device_id.devid;

	gps.latitude_deg = hil_gps.lat * 1e-7;
	gps.longitude_deg = hil_gps.lon * 1e-7;
	gps.altitude_msl_m = hil_gps.alt * 1e-3;
	gps.altitude_ellipsoid_m = hil_gps.alt * 1e-3;

	gps.s_variance_m_s = 0.25f;
	gps.c_variance_rad = 0.5f;

	gps.satellites_used = hil_gps.satellites_visible;
	gps.fix_type = hil_gps.fix_type;

	int index = (int) position_source::GPS;

	if (position_source_data[index].fail) {
		uint32_t duration = position_source_data[index].failure_duration;
		hrt_abstime start = position_source_data[index].failure_duration_start;

		if (duration) {
			if (hrt_elapsed_time(&start) > (duration * 1000000)) {
				PX4_INFO("GPS failure ending");
				position_source_data[index].fail = false;
				position_source_data[index].failure_duration = 0;
				position_source_data[index].failure_duration_start = 0;

			} else {
				gps.satellites_used = 1;
				gps.fix_type = 0;
			}

		} else {
			gps.satellites_used = 1;
			gps.fix_type = 0;
		}
	}

	gps.eph = (float)hil_gps.eph * 1e-2f; // cm -> m
	gps.epv = (float)hil_gps.epv * 1e-2f; // cm -> m

	gps.hdop = 0; // TODO
	gps.vdop = 0; // TODO

	gps.noise_per_ms = 0;
	gps.automatic_gain_control = 0;
	gps.jamming_indicator = 0;
	gps.jamming_state = 0;
	gps.spoofing_state = 0;

	gps.vel_m_s = (float)(hil_gps.vel) / 100.0f; // cm/s -> m/s
	gps.vel_n_m_s = (float)(hil_gps.vn) / 100.0f; // cm/s -> m/s
	gps.vel_e_m_s = (float)(hil_gps.ve) / 100.0f; // cm/s -> m/s
	gps.vel_d_m_s = (float)(hil_gps.vd) / 100.0f; // cm/s -> m/s
	gps.cog_rad = ((hil_gps.cog == 65535) ? (float)NAN : matrix::wrap_2pi(math::radians(
				hil_gps.cog * 1e-2f))); // cdeg -> rad
	gps.vel_ned_valid = true;

	gps.timestamp_time_relative = 0;
	gps.time_utc_usec = hil_gps.time_usec;


	gps.heading = NAN;
	gps.heading_offset = NAN;

	gps.timestamp = hrt_absolute_time();

	_sensor_gps_pub.publish(gps);

	gps_sent_counter++;
}

int
process_failure(dsp_hitl::position_source src, int duration)
{
	if (src >= position_source::NUM_POSITION_SOURCES) {
		return 1;
	}

	int index = (int) src;

	if (position_source_data[index].send) {
		if (duration <= 0) {
			// Toggle state
			if (position_source_data[index].fail) {
				PX4_INFO("Ending indefinite %s failure", position_source_data[index].label);
				position_source_data[index].fail = false;

			} else {
				PX4_INFO("Starting indefinite %s failure", position_source_data[index].label);
				position_source_data[index].fail = true;
			}

			position_source_data[index].failure_duration = 0;
			position_source_data[index].failure_duration_start = 0;

		} else {
			PX4_INFO("%s failure for %d seconds", position_source_data[index].label, duration);
			position_source_data[index].fail = true;
			position_source_data[index].failure_duration = duration;
			position_source_data[index].failure_duration_start = hrt_absolute_time();
		}

	} else {
		PX4_ERR("%s not active, cannot create failure", position_source_data[index].label);
		return 1;
	}

	return 0;
}

} // End dsp_hitl namespace

int dsp_hitl_main(int argc, char *argv[])
{
	int myoptind = 1;

	if (argc <= 1) {
		dsp_hitl::usage();
		return 1;
	}

	const char *verb = argv[myoptind];

	if (!strcmp(verb, "start")) {
		return dsp_hitl::start(argc - 1, argv + 1);

	} else if (!strcmp(verb, "stop")) {
		return dsp_hitl::stop();

	} else if (!strcmp(verb, "status")) {
		dsp_hitl::print_status();
		return 0;

	} else if (!strcmp(verb, "clear")) {
		dsp_hitl::clear_status_counters();
		return 0;

	} else if (!strcmp(verb, "failure")) {
		if (argc != 4) {
			dsp_hitl::usage();
			return 1;
		}

		const char *source = argv[myoptind + 1];
		int duration = atoi(argv[myoptind + 2]);

		if (!strcmp(source, "gps")) {
			return dsp_hitl::process_failure(dsp_hitl::position_source::GPS, duration);

		} else if (!strcmp(source, "vio")) {
			return dsp_hitl::process_failure(dsp_hitl::position_source::VIO, duration);

		} else if (!strcmp(source, "flow")) {
			return dsp_hitl::process_failure(dsp_hitl::position_source::FLOW, duration);

		} else {
			PX4_ERR("Unknown failure source %s, duration %d", source, duration);
			dsp_hitl::usage();
			return 1;
		}

		return 0;

	} else {
		dsp_hitl::usage();
		return 1;
	}
}
