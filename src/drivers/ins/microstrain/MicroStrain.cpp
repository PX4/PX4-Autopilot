/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include <lib/drivers/device/Device.hpp>

#include "MicroStrain.hpp"
#include "mip_sdk/src/mip/mip_all.h"
#include "mip_sdk/src/mip/mip_parser.h"

uint8_t external_heading_sensor_id = 1;
uint8_t gnss_antenna_sensor_id = 2;
uint8_t vehicle_frame_velocity_sensor_id = 3;

static CvIns *cv7_ins{nullptr};


ModalIoSerial device_uart;

const uint8_t FILTER_ROLL_EVENT_ACTION_ID  = 1;
const uint8_t FILTER_PITCH_EVENT_ACTION_ID = 2;


CvIns::CvIns(const char *uart_port, int32_t rot) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::test1)
{
	/* store port name */
	memset(_port, '\0', 20);
	int max_len = math::min((unsigned int)20, strlen(uart_port));
	strncpy(_port, uart_port, max_len);
	/* enforce null termination */
	_port[19] = '\0';
	_config.rot = static_cast<Rotation>(rot);

	// // Clamp rate to allowable ranges
	_config.sens_imu_update_rate_hz = math::constrain<uint16_t>(_param_cv7_update_rate.get(), 100, 1000);

	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_MS;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.bus = 2;
	_config.device_id = device_id.devid;
	// Default to ROTATION_NONE
	_px4_accel.set_device_id(_config.device_id);
	_px4_gyro.set_device_id(_config.device_id);
	_px4_mag.set_device_id(_config.device_id);

	// Set the default values for the baro (which may not change)
	_sensor_baro.device_id = _config.device_id;
	_sensor_baro.pressure = 0;
	_sensor_baro.temperature = 0;
	_sensor_baro.error_count = 0;
}

CvIns::~CvIns()
{
	if (device_uart.is_open()) {
		device_uart.uart_close();
	}

	_sensor_baro_pub.unadvertise();
	//delete &_sensor_baro_pub;

	PX4_INFO("Destructor");
	_sensor_baro_pub.unadvertise();
	_sensor_selection_pub.unadvertise();
	_vehicle_local_position_pub.unadvertise();
	_vehicle_angular_velocity_pub.unadvertise();
	_vehicle_attitude_pub.unadvertise();
	_global_position_pub.unadvertise();
	_vehicle_odometry_pub.unadvertise();
	_debug_array_pub.unadvertise();
	_estimator_status_pub.unadvertise();


	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool mip_interface_user_recv_from_device(mip_interface *device, uint8_t *buffer, size_t max_length,
		timeout_type wait_time,
		size_t *out_length, timestamp_type *timestamp_out)
{
	(void)device;

	*timestamp_out = hrt_absolute_time();

	int res = device_uart.uart_read(buffer, max_length);

	if (res == -1 && errno != EAGAIN) {
		PX4_DEBUG("RX 1 %d(%d)", res, max_length);
		*out_length = 0;
		return false;
	}

	if (res >= 0) {
		*out_length = res;
		PX4_DEBUG("RX 2 %d(%d)", *out_length, max_length);
	}

	PX4_DEBUG("RX 3 %d(%d)", *out_length, max_length);

	cv7_ins->_debug_rx_bytes[0] = math::min<uint32_t>(cv7_ins->_debug_rx_bytes[0], *out_length);
	cv7_ins->_debug_rx_bytes[1] += *out_length;
	cv7_ins->_debug_rx_bytes[2] = math::max<uint32_t>(cv7_ins->_debug_rx_bytes[2], *out_length);
	cv7_ins->_debug_rx_bytes[3]++;
	return true;
}

bool mip_interface_user_send_to_device(mip_interface *device, const uint8_t *data, size_t length)
{

	PX4_DEBUG("TX %d", length);
	int res = device_uart.uart_write(const_cast<uint8_t *>(data), length);

	if (cv7_ins) {
		cv7_ins->_debug_tx_bytes += length;
	}

	if (res >= 0) {
		return true;
	}

	return false;

}

int CvIns::connect_at_baud(int32_t baud)
{
	if (device_uart.is_open()) {
		if (device_uart.uart_set_baud(baud) == PX4_ERROR) {
			PX4_INFO(" - Failed to set UART %" PRIu32 " baud", baud);
		}

	} else if (device_uart.uart_open(_port, baud) == PX4_ERROR) {
		PX4_INFO(" - Failed to open UART");
		PX4_ERR("ERROR: Could not open device port!");
		return PX4_ERROR;
	}

	PX4_INFO("Serial Port %s with baud of %" PRIu32 " baud", (device_uart.is_open() ? "CONNECTED" : "NOT CONNECTED"), baud);

	// Re-init the interface with the correct timeouts
	// mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms);
	mip_interface_init(&device, parse_buffer, sizeof(parse_buffer), mip_timeout_from_baudrate(baud) * 1_ms, 250_ms,
			   &mip_interface_user_send_to_device, &mip_interface_user_recv_from_device, &mip_interface_default_update, NULL);

	PX4_INFO("mip_base_ping");

	if (mip_base_ping(&device) != MIP_ACK_OK) {
		PX4_INFO(" - Failed to Ping 1");
		usleep(200_ms);

		if (mip_base_ping(&device) != MIP_ACK_OK) {
			PX4_INFO(" - Failed to Ping 2");
			return PX4_ERROR;
		}
	}

	PX4_INFO("Successfully opened and pinged");
	return PX4_OK;
}

void CvIns::set_sensor_rate(mip_descriptor_rate *descriptors, uint16_t len, bool is_sensor)
{
	// Get the base rate
	uint16_t base_rate;

	if (is_sensor) {
		if (mip_3dm_get_base_rate(&device, MIP_SENSOR_DATA_DESC_SET, &base_rate) != MIP_ACK_OK) {
			PX4_ERR("ERROR: Could not get sensor base rate format!");
			return;
		}

	} else {
		if (mip_3dm_get_base_rate(&device, MIP_FILTER_DATA_DESC_SET, &base_rate) != MIP_ACK_OK) {
			PX4_ERR("ERROR: Could not get sensor base rate format!");
			return;
		}

	}

	PX4_INFO("The CV7 base rate is %d", base_rate);

	for (uint16_t i = 0; i < len; i++) {
		// Compute the desired decimation and update all of the sensors in this set
		float decimation = static_cast<float>(base_rate) / static_cast<float>(descriptors[i].decimation);

		descriptors[i].decimation = static_cast<uint16_t>(decimation);
	}

	// Write the settings
	mip_cmd_result res = MIP_NACK_COMMAND_FAILED;

	if (is_sensor) {
		res = mip_3dm_write_message_format(&device, MIP_SENSOR_DATA_DESC_SET, len, descriptors);

	} else {
		res = mip_3dm_write_message_format(&device, MIP_FILTER_DATA_DESC_SET, len, descriptors);
	}

	if (res != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set sensor message format! Result of %d", res);
		return;
	}
}



void CvIns::initialize_cv7()
{
	if (_is_initialized) {
		return;
	}

	// first try default baudrate
	const uint32_t DEFAULT_BAUDRATE = 115200;
	const uint32_t DESIRED_BAUDRATE = 921600;

	if (connect_at_baud(DEFAULT_BAUDRATE) == PX4_ERROR) {

		static constexpr uint32_t BAUDRATES[] {9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600};
		bool is_connected = false;

		for (auto &baudrate : BAUDRATES) {
			if (connect_at_baud(baudrate) == PX4_OK) {
				PX4_INFO("found baudrate %" PRIu32, baudrate);
				is_connected = true;
				break;
			}
		}

		if (!is_connected) {
			_is_init_failed = true;
			PX4_WARN("Could not connect to the device, exiting");
			return;
		}
	}


	PX4_INFO("mip_base_set_idle");

	if (mip_base_set_idle(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set the device to idle!");
		return;
	}

	PX4_INFO("Setting the baud to desired baud rate");

	usleep(500_ms);

	if (mip_3dm_write_uart_baudrate(&device, DESIRED_BAUDRATE) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set the baudrate!");
		_is_init_failed = true;
		return;
	}

	tcflush(device_uart.uart_get_fd(), TCIOFLUSH);

	usleep(500_ms);

	for (int i = 0; i < 10; i++) {
		PX4_INFO("Connection Attempt: %d", i);

		if (connect_at_baud(DESIRED_BAUDRATE) == PX4_OK) {
			break;
		}

		if (i >= 9) {
			PX4_ERR("ERROR: Could not reconnect at desired baud!");
			_is_init_failed = true;
			return;
		}
	}


	// Scaled Gyro and Accel at a high rate
	mip_descriptor_rate imu_sensors[4] = {
		{ MIP_DATA_DESC_SENSOR_ACCEL_SCALED, _config.sens_imu_update_rate_hz},
		{ MIP_DATA_DESC_SENSOR_GYRO_SCALED, _config.sens_imu_update_rate_hz},
		{ MIP_DATA_DESC_SENSOR_MAG_SCALED,  _config.sens_other_update_rate_hz},
		{ MIP_DATA_DESC_SENSOR_PRESSURE_SCALED, _config.sens_other_update_rate_hz},

	};

	set_sensor_rate(imu_sensors, 4, true);


	//
	// Register data callbacks
	//
	mip_interface_register_packet_callback(&device, &sensor_data_handler, MIP_SENSOR_DATA_DESC_SET, false, &sensor_callback,
					       this);


	//
	//External GNSS antenna reference frame
	//
	mip_aiding_frame_config_command_rotation rotation{0};

	for (uint8_t i = 0; i < 3; i++) {
		rotation.euler[i] = 0.0;
	}

	float translation[3] = { (float)_param_cv7_gps_x.get(), (float)_param_cv7_gps_y.get(), (float)_param_cv7_gps_z.get() };
	mip_aiding_write_frame_config(&device, gnss_antenna_sensor_id, MIP_AIDING_FRAME_CONFIG_COMMAND_FORMAT_EULER, false,
				      translation, &rotation);


	mip_descriptor_rate filter_data[11] = {
		{ MIP_DATA_DESC_FILTER_POS_LLH, _config.sens_imu_update_rate_hz},
		{ MIP_DATA_DESC_FILTER_ATT_QUATERNION,  _config.sens_imu_update_rate_hz},
		{ MIP_DATA_DESC_FILTER_VEL_NED, _config.sens_imu_update_rate_hz},
		{ MIP_DATA_DESC_FILTER_REL_POS_NED, _config.sens_imu_update_rate_hz},
		{ MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION, _config.sens_other_update_rate_hz},
		{ MIP_DATA_DESC_FILTER_POS_UNCERTAINTY, _config.sens_imu_update_rate_hz},
		{ MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY, _config.sens_imu_update_rate_hz},
		{ MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER, _config.sens_imu_update_rate_hz},
		{ MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE, _config.sens_imu_update_rate_hz},
		{ MIP_DATA_DESC_FILTER_FILTER_STATUS, _config.sens_status_update_rate_hz},
		{ MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY, _config.sens_status_update_rate_hz},
	};

	set_sensor_rate(filter_data, 11, false);


	mip_interface_register_packet_callback(&device, &filter_data_handler, MIP_FILTER_DATA_DESC_SET, false, &filter_callback,
					       this);


	// Selectively turn on the mag aiding source
	if (_param_cv7_int_mag_en.get() == 1) {
		if (mip_filter_write_aiding_measurement_enable(&device,
				MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER, true) != MIP_ACK_OK) {
			PX4_ERR("ERROR: Could not set filter aiding measurement enable!");
		}

	} else {
		if (mip_filter_write_aiding_measurement_enable(&device,
				MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_MAGNETOMETER, false) == MIP_ACK_OK) {
			PX4_INFO("Mag Disabled");
		}
	}

	if (mip_filter_write_aiding_measurement_enable(&device,
			MIP_FILTER_AIDING_MEASUREMENT_ENABLE_COMMAND_AIDING_SOURCE_GNSS_POS_VEL, true) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set filter aiding measurement enable!");
	}

	// Reset the filter, then set the initial conditions
	if (mip_filter_reset(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not reset the filter!");
	}


	float filter_init_pos[3] = {0};
	float filter_init_vel[3] = {0};
	uint8_t initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_MAGNETOMETER;

	if (_param_cv7_alignment.get() == 1) {
		initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_KINEMATIC;
	}

	if (_param_cv7_alignment.get() == 2) {
		initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_EXTERNAL;
	}

	if (_param_cv7_alignment.get() == 3) {
		initial_alignment = MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_ALIGNMENT_SELECTOR_DUAL_ANTENNA;
	}

	// Config for Position, Velocity and Attitude, use kinematic alignment for initialization
	if (mip_filter_write_initialization_configuration(&device, 0,
			MIP_FILTER_INITIALIZATION_CONFIGURATION_COMMAND_INITIAL_CONDITION_SOURCE_AUTO_POS_VEL_ATT,
			initial_alignment,
			0.0, 0.0, 0.0, filter_init_pos, filter_init_vel, MIP_FILTER_REFERENCE_FRAME_LLH) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set filter initialization configuration!");
	}


	//
	// Setup the rotation based on PX4 standard rotation sets
	//

	if (mip_3dm_write_sensor_2_vehicle_transform_euler(&device, math::radians<float>(rot_lookup[_config.rot].roll),
			math::radians<float>(rot_lookup[_config.rot].pitch),
			math::radians<float>(rot_lookup[_config.rot].yaw)) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not set sensor-to-vehicle transformation!");
		return;
	}


	if (mip_3dm_write_datastream_control(&device, MIP_3DM_DATASTREAM_CONTROL_COMMAND_ALL_STREAMS, true) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not enable the data stream");
		return;
	}

	//
	//Resume the device
	//

	if (mip_base_resume(&device) != MIP_ACK_OK) {
		PX4_ERR("ERROR: Could not resume the device!");
		return;
	}

	_is_initialized = true;

}

void CvIns::service_cv7()
{
	mip_interface_update(&device, false);

	sensor_gps_s gps{0};

	// No new data
	if (!_sensor_gps_sub.update(&gps)) {
		return;
	}

	// Fix isn't 3D or RTK or RTCM
	if ((gps.fix_type < 3) || (gps.fix_type > 6)) {
		return;
	}

#define deg_conv(x) (double((x*1.0) / 10000000.0))

	// If the timestamp has not been set, then don't send any data
	// into the cv7 filter
	if (gps.time_utc_usec == 0) {
		return;
	}

	mip_time t;
	t.timebase = MIP_TIME_TIMEBASE_TIME_OF_ARRIVAL;
	t.reserved = 0x01;
	t.nanoseconds = 0; // No offset


	// float llh_uncertainty[3] = {gps.eph,gps.eph,gps.eph}; // What is the uncertainty?
	float llh_uncertainty[3] = {1.0, 1.0, 1.0};
	mip_aiding_llh_pos(&device, &t, MIP_FILTER_REFERENCE_FRAME_LLH, deg_conv(gps.latitude_deg), deg_conv(gps.longitude_deg),
			   ((gps.altitude_ellipsoid_m * 1.0) / 1000.0), llh_uncertainty, MIP_AIDING_LLH_POS_COMMAND_VALID_FLAGS_ALL);

	if (gps.vel_ned_valid) {
		float ned_v[3] = {gps.vel_n_m_s, gps.vel_e_m_s, gps.vel_d_m_s};
		// float ned_velocity_uncertainty[3] = {gps.s_variance_m_s,gps.s_variance_m_s,gps.s_variance_m_s}; // What is the uncertainty of NED velocity?
		float ned_velocity_uncertainty[3] = {0.1, 0.1, 0.1};
		mip_aiding_ned_vel(&device, &t, MIP_FILTER_REFERENCE_FRAME_LLH, ned_v, ned_velocity_uncertainty,
				   MIP_AIDING_NED_VEL_COMMAND_VALID_FLAGS_ALL);
	}

	if (PX4_ISFINITE(gps.heading)) {
		// float heading = PX4_ISFINITE(gps.heading_offset) ? gps.heading + gps.heading_offset : gps.heading;
		float heading = gps.heading;
		// There are no pre-defined flags for the heading (that I can find), setting everything to 1 for now
		mip_aiding_true_heading(&device, &t, MIP_FILTER_REFERENCE_FRAME_LLH, heading, gps.heading_accuracy, 0xff);
	}

}

void CvIns::sensor_callback(void *user, const mip_packet *packet, mip::Timestamp timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);

	if (mip_packet_descriptor_set(packet) != MIP_SENSOR_DATA_DESC_SET) {
		return;
	}

	mip_sensor_scaled_accel_data accel;
	mip_sensor_scaled_gyro_data gyro;
	mip_sensor_scaled_mag_data mag;
	mip_sensor_scaled_pressure_data baro;

	bool accel_valid = false;
	bool gyro_valid = false;
	bool mag_valid = false;
	bool baro_valid = false;

	auto t = hrt_absolute_time() - (ref->_delay_offset * 1_us);

	for (mip_field field = mip_field_first_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field)) {
		switch (mip_field_field_descriptor(&field)) {
		case MIP_DATA_DESC_SENSOR_ACCEL_SCALED: extract_mip_sensor_scaled_accel_data_from_field(&field, &accel);
			accel_valid = true; break;

		case MIP_DATA_DESC_SENSOR_GYRO_SCALED: extract_mip_sensor_scaled_gyro_data_from_field(&field, &gyro); gyro_valid = true;
			break;

		case MIP_DATA_DESC_SENSOR_MAG_SCALED: extract_mip_sensor_scaled_mag_data_from_field(&field, &mag); mag_valid = true;
			break;

		case MIP_DATA_DESC_SENSOR_PRESSURE_SCALED: extract_mip_sensor_scaled_pressure_data_from_field(&field, &baro);
			baro_valid = true; break;

		default: break;

		}
	}

	if (accel_valid) {
		ref->_px4_accel.update(t, accel.scaled_accel[0]*CONSTANTS_ONE_G,
				       accel.scaled_accel[1]*CONSTANTS_ONE_G,
				       accel.scaled_accel[2]*CONSTANTS_ONE_G);
	}

	if (gyro_valid) {
		ref->_px4_gyro.update(t, gyro.scaled_gyro[0],
				      gyro.scaled_gyro[1],
				      gyro.scaled_gyro[2]);
	}

	if (mag_valid) {
		ref->_px4_mag.update(t, mag.scaled_mag[0],
				     mag.scaled_mag[1],
				     mag.scaled_mag[2]);
	}

	if (baro_valid) {
		ref->_sensor_baro.timestamp = timestamp;
		ref->_sensor_baro.timestamp_sample = t;
		ref->_sensor_baro.pressure = baro.scaled_pressure * 100.f; // convert [Pa] to [mBar]
		ref->_sensor_baro_pub.publish(ref->_sensor_baro);

	}

}

void CvIns::filter_callback(void *user, const mip_packet *packet, mip::Timestamp timestamp)
{
	CvIns *ref = static_cast<CvIns *>(user);

	if (mip_packet_descriptor_set(packet) != MIP_FILTER_DATA_DESC_SET) {
		return;
	}

	mip_filter_position_llh_data pos_llh;
	mip_filter_attitude_quaternion_data att_quat;
	mip_filter_comp_angular_rate_data ang_rate;
	mip_filter_rel_pos_ned_data rel_pos;
	mip_filter_velocity_ned_data vel_ned;
	mip_filter_status_data stat;
	mip_filter_position_llh_uncertainty_data llh_uncert;
	mip_filter_velocity_ned_uncertainty_data vel_uncert;
	mip_filter_euler_angles_uncertainty_data att_euler_uncert;
	mip_filter_linear_accel_data lin_accel;
	mip_filter_aiding_measurement_summary_data aid_sum;

	bool updated_data[11] = {false};

	auto t = hrt_absolute_time() - (ref->_delay_offset * 1_us);;
	float yaw = NAN;

	for (mip_field field = mip_field_first_from_packet(packet); mip_field_is_valid(&field); mip_field_next(&field)) {
		switch (mip_field_field_descriptor(&field)) {
		case MIP_DATA_DESC_FILTER_POS_LLH: extract_mip_filter_position_llh_data_from_field(&field, &pos_llh);
			updated_data[0] = true; break;

		case MIP_DATA_DESC_FILTER_ATT_QUATERNION: extract_mip_filter_attitude_quaternion_data_from_field(&field, &att_quat);
			updated_data[1] = true; break;

		case MIP_DATA_DESC_FILTER_VEL_NED: extract_mip_filter_velocity_ned_data_from_field(&field, &vel_ned);
			updated_data[2] = true ; break;

		case MIP_DATA_DESC_FILTER_REL_POS_NED: extract_mip_filter_rel_pos_ned_data_from_field(&field, &rel_pos);
			updated_data[3] = true; break;

		case MIP_DATA_DESC_FILTER_LINEAR_ACCELERATION: extract_mip_filter_linear_accel_data_from_field(&field, &lin_accel);
			updated_data[4] = true; break;

		case MIP_DATA_DESC_FILTER_POS_UNCERTAINTY: extract_mip_filter_position_llh_uncertainty_data_from_field(&field,
					&llh_uncert); updated_data[5] = true; break;

		case MIP_DATA_DESC_FILTER_VEL_UNCERTAINTY: extract_mip_filter_velocity_ned_uncertainty_data_from_field(&field,
					&vel_uncert); updated_data[6] = true; break;

		case MIP_DATA_DESC_FILTER_ATT_UNCERTAINTY_EULER: extract_mip_filter_euler_angles_uncertainty_data_from_field(&field,
					&att_euler_uncert); updated_data[7] = true; break;

		case MIP_DATA_DESC_FILTER_COMPENSATED_ANGULAR_RATE: extract_mip_filter_comp_angular_rate_data_from_field(&field,
					&ang_rate); updated_data[8] = true; break;

		case MIP_DATA_DESC_FILTER_FILTER_STATUS: extract_mip_filter_status_data_from_field(&field, &stat);
			updated_data[9] = true; break;

		case MIP_DATA_DESC_FILTER_AID_MEAS_SUMMARY: extract_mip_filter_aiding_measurement_summary_data_from_field(&field,
					&aid_sum); updated_data[10] = true; break;

		default: break;

		}
	}

	bool vgp_valid = updated_data[0];
	bool va_valid = updated_data[1];
	bool vlp_valid = va_valid && updated_data[2] && updated_data[3] && updated_data[4] && updated_data[5]
			 && updated_data[6];
	bool vo_valid = updated_data[2] && updated_data[3] && updated_data[5] && updated_data[6] && updated_data[7];
	bool vav_valid = updated_data[8];
	bool debug_valid = updated_data[9] && updated_data[10];

	if (vgp_valid) {
		vehicle_global_position_s gp{0};
		gp.timestamp = t;
		gp.timestamp_sample = t;
		gp.lat = pos_llh.latitude;
		gp.lon = pos_llh.longitude;
		gp.alt_ellipsoid = pos_llh.ellipsoid_height;

		gp.alt = pos_llh.ellipsoid_height;
		gp.eph = 0.1f;
		gp.epv = 0.1f;

		ref->_global_position_pub.publish(gp);
	}

	if (va_valid) {
		vehicle_attitude_s att_data{0};
		att_data.timestamp = t;
		att_data.timestamp_sample = t;
		att_data.q[0] = att_quat.q[0];
		att_data.q[1] = att_quat.q[1];
		att_data.q[2] = att_quat.q[2];
		att_data.q[3] = att_quat.q[3];
		att_data.quat_reset_counter = 0;

		ref->_vehicle_attitude_pub.publish(att_data);

		// convert to YAW
		matrix::Eulerf euler_attitude(matrix::Quatf(att_data.q));
		yaw = euler_attitude.psi();
	}

	if (vlp_valid) {
		vehicle_local_position_s vp{0};
		vp.timestamp = t;
		vp.timestamp_sample = t;

		vp.x = rel_pos.relative_position[0];
		vp.y = rel_pos.relative_position[1];
		vp.z = rel_pos.relative_position[2];

		vp.vx = vel_ned.north;
		vp.vy = vel_ned.east;
		vp.vz = vel_ned.down;

		vp.ax = lin_accel.accel[0];
		vp.ay = lin_accel.accel[1];
		vp.az = lin_accel.accel[2];

		vp.eph = sqrt(llh_uncert.north * llh_uncert.north + llh_uncert.east * llh_uncert.east);
		vp.epv = llh_uncert.down;

		vp.evh = sqrt(vel_uncert.north * vel_uncert.north + vel_uncert.east * vel_uncert.east);
		vp.evv = vel_uncert.down;

		vp.delta_xy[0] = 0.f;
		vp.delta_xy[1] = 0.f;
		vp.xy_reset_counter = 0;

		vp.z_valid = true;
		vp.xy_valid = true;
		vp.v_xy_valid = true;
		vp.v_z_valid = true;
		vp.z_deriv = vp.vz;
		vp.z_global = true;
		vp.xy_global = true;
		// Set heading as valid if we have any yaw
		vp.heading_good_for_control = (yaw != NAN);
		vp.heading = yaw; // This is extracted from vehicle attitude

		vp.vxy_max = INFINITY;
		vp.vz_max = INFINITY;
		vp.hagl_min = INFINITY;
		vp.hagl_max = INFINITY;

		ref->_vehicle_local_position_pub.publish(vp);

	}

	if (vo_valid) {
		vehicle_odometry_s vo{0};
		vo.timestamp = t;
		vo.timestamp_sample = t;

		vo.position[0] = rel_pos.relative_position[0];
		vo.position[1] = rel_pos.relative_position[1];
		vo.position[2] = rel_pos.relative_position[2];

		vo.velocity[0] = vel_ned.north;
		vo.velocity[1] = vel_ned.east;
		vo.velocity[2] = vel_ned.down;

		vo.position_variance[0] = llh_uncert.north;
		vo.position_variance[1] = llh_uncert.east;
		vo.position_variance[2] = llh_uncert.down;

		vo.velocity_variance[0] = vel_uncert.north;
		vo.velocity_variance[1] = vel_uncert.east;
		vo.velocity_variance[2] = vel_uncert.down;

		vo.orientation_variance[0] = att_euler_uncert.roll;
		vo.orientation_variance[1] = att_euler_uncert.pitch;
		vo.orientation_variance[2] = att_euler_uncert.yaw;

		ref->_vehicle_odometry_pub.publish(vo);
	}

	if (vav_valid) {
		vehicle_angular_velocity_s av{0};
		av.timestamp = t;
		av.timestamp_sample = t;

		av.xyz[0] = ang_rate.gyro[0];
		av.xyz[1] = ang_rate.gyro[1];
		av.xyz[2] = ang_rate.gyro[2];

		// xyz_derivative ??
		ref->_vehicle_angular_velocity_pub.publish(av);
	}

	if (debug_valid) {
		debug_array_s dbg{0};
		dbg.id = 0x01;
		dbg.timestamp = t;
		strcpy(dbg.name, "CV7");

		dbg.data[0] = stat.filter_state * 1.0f;
		dbg.data[1] = stat.dynamics_mode * 1.0f;
		dbg.data[2] = stat.status_flags * 1.0f;
		dbg.data[3] = aid_sum.indicator * 1.0f;
		dbg.data[4] = aid_sum.time_of_week * 1.0f;
		dbg.data[5] = aid_sum.source * 1.0f;
		dbg.data[6] = aid_sum.type * 1.0f;

		ref->_debug_array_pub.publish(dbg);

	}

	//check if condition if correct - Joel
	//Keep status and llh_uncert at same rate? - Joel
	if (true) {
		estimator_status_s status{0};
		status.timestamp = t;
		status.timestamp_sample = t;
		// Note: These flags require insight into the inner operations of the filter.
		//       Below is a minimal mapping of error flags from CV7 to the PX4 health flags
		// Filter State Mapping, if the filter is in 4, set merging GPS and Height being fused flags
		// Only the gps flag is inspected in the checks
		status.control_mode_flags = stat.filter_state == 4 ? (0x1 << estimator_status_s::CS_GPS) |
					    (0x1 << estimator_status_s::CS_GPS_HGT) : 0x00;
		// If there is a general filter condition, set a fault flag to trigger an issue
		status.filter_fault_flags = (stat.status_flags & MIP_FILTER_STATUS_FLAGS_GQ7_FILTER_CONDITION) ? 0x1 : 0x00;
		status.gps_check_fail_flags = 0;

		status.vel_test_ratio = 0.1f;
		status.pos_test_ratio = 0.1f;
		status.hgt_test_ratio = 0.1f;
		status.tas_test_ratio = 0.1f;
		status.hagl_test_ratio = 0.1f;
		status.beta_test_ratio = 0.1f;

		status.pos_horiz_accuracy = sqrt(llh_uncert.north * llh_uncert.north + llh_uncert.east * llh_uncert.east);
		status.pos_vert_accuracy = llh_uncert.down;
		status.solution_status_flags = 0;

		status.time_slip = 0;
		status.pre_flt_fail_innov_heading = false;
		status.pre_flt_fail_innov_vel_horiz = false;
		status.pre_flt_fail_innov_vel_vert = false;
		status.pre_flt_fail_innov_height = false;
		status.pre_flt_fail_mag_field_disturbed = false;
		status.accel_device_id = ref->_config.device_id;
		status.gyro_device_id = ref->_config.device_id;
		status.mag_device_id = ref->_config.device_id;
		status.baro_device_id = ref->_config.device_id;
		ref->_estimator_status_pub.publish(status);

		sensor_selection_s sensor_selection{};
		sensor_selection.accel_device_id = ref->_config.device_id;
		sensor_selection.gyro_device_id = ref->_config.device_id;
		sensor_selection.timestamp = t;
		ref->_sensor_selection_pub.publish(sensor_selection);
	}

}

bool CvIns::init()
{
	// Run on fixed interval
	ScheduleOnInterval(_param_cv7_schedule.get());

	return true;
}


void CvIns::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	initialize_cv7();

	// Initialization failed, stop the module
	if (_is_init_failed) {
		request_stop();
		perf_end(_loop_perf);
		return;
	}

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)

		_delay_offset = _param_cv7_delay.get();

		// Disable this for now, work in progress
		// apply_mag_cal();
	}

	service_cv7();

	perf_end(_loop_perf);
}

int CvIns::task_spawn(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	const char *dev = "/dev/ttyS4";
	int32_t rot = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "d:r:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			dev = myoptarg;
			break;

		case 'r':
			rot = atoi(myoptarg);

			if (rot >= ROTATION_MAX) {
				rot = ROTATION_NONE;
			}

			break;
		}
	}

	if (dev == nullptr || strlen(dev) == 0) {
		print_usage("no device specified");
		_object.store(nullptr);
		_task_id = -1;

		return PX4_ERROR;
	}

	PX4_INFO("Opening device port %s", dev);
	CvIns *instance = new CvIns(dev, rot);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		// Get a local reference
		cv7_ins = instance;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int CvIns::print_status()
{
	PX4_INFO_RAW("Serial Port Open %d Handle %d Device %s\n", device_uart.is_open(), device_uart.uart_get_fd(),
		     _port);
	PX4_INFO_RAW("TX Bytes %lu\n", _debug_tx_bytes);
	PX4_INFO_RAW("Min %lu\n", _debug_rx_bytes[0]);
	PX4_INFO_RAW("Total %lu\n", _debug_rx_bytes[1]);
	PX4_INFO_RAW("Max %lu\n", _debug_rx_bytes[2]);
	PX4_INFO_RAW("Avg %f\n", static_cast<double>(_debug_rx_bytes[1] * 1.f / _debug_rx_bytes[3] * 1.f));
	_debug_rx_bytes[0] = UINT32_MAX;

	for (int i = 1; i < 4; i++) {
		_debug_rx_bytes[i] = 0;
	}

	_debug_tx_bytes = 0;

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int CvIns::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CvIns::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
CV7 IMU Driver.

Communicates over serial port an utilizes the manufacturer provided MIP SDK.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cv7_ins", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS2", "<file:dev>", "CV7 Port", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 0, ROTATION_MAX, "See enum Rotation for values", true);

	return 0;
}

extern "C" __EXPORT int microstrain_main(int argc, char *argv[])
{
	return CvIns::main(argc, argv);
}

