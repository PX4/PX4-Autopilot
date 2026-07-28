/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "naviguider.hpp"
#include <cmath>
#include <cstdlib>

Naviguider::Naviguider(int bus, uint8_t address)
    : I2C(DRV_INS_DEVTYPE_NAVIGUIDER, MODULE_NAME, bus, address, 400000),
     ModuleParams(nullptr),
     ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
     _addr(address),
     _count_start_time(0),
     _accel_count(0),
     _gyro_count(0),
     _mag_count(0),
     _baro_count(0),
     _attitude_count(0),
     _param_ng_mode_initial_value(_param_ng_mode.get()),
     _param_ng_rot_initial_value(_param_ng_rot.get()) {
    // Seed the pseudo-random number generator
    srand(hrt_absolute_time());

    // Create unique device IDs for each sensor type
    device::Device::DeviceId accel_id;
    accel_id.devid_s.bus_type = device::Device::DeviceBusType_I2C;
    accel_id.devid_s.bus = bus;
    accel_id.devid_s.address = address;
    accel_id.devid_s.devtype = DRV_ACC_DEVTYPE_NAVIGUIDER;

    device::Device::DeviceId gyro_id;
    gyro_id.devid_s.bus_type = device::Device::DeviceBusType_I2C;
    gyro_id.devid_s.bus = bus;
    gyro_id.devid_s.address = address;
    gyro_id.devid_s.devtype = DRV_GYR_DEVTYPE_NAVIGUIDER;

    device::Device::DeviceId mag_id;
    mag_id.devid_s.bus_type = device::Device::DeviceBusType_I2C;
    mag_id.devid_s.bus = bus;
    mag_id.devid_s.address = address;
    mag_id.devid_s.devtype = DRV_MAG_DEVTYPE_NAVIGUIDER;

    device::Device::DeviceId synth_mag_id;
    synth_mag_id.devid_s.bus_type = device::Device::DeviceBusType_I2C;
    synth_mag_id.devid_s.bus = bus;
    synth_mag_id.devid_s.address = address+1;
    synth_mag_id.devid_s.devtype = DRV_MAG_DEVTYPE_NAVIGUIDER;

    // Initialize to set both the device ID and rotation
    _px4_accel     = new PX4Accelerometer(   accel_id.devid, Rotation(this->_param_ng_rot_initial_value));
    _px4_gyro      = new PX4Gyroscope(        gyro_id.devid, Rotation(this->_param_ng_rot_initial_value));
    _px4_mag       = new PX4Magnetometer(      mag_id.devid, Rotation(this->_param_ng_rot_initial_value));
    _px4_mag_synth = new PX4Magnetometer(synth_mag_id.devid, Rotation(this->_param_ng_rot_initial_value));

    // Set scale factors
    _px4_accel->set_scale(1.0f);
    _px4_gyro->set_scale(1.0f);
    _px4_mag->set_scale(1.0f);

    // Initialize _last_mag_ned_vec to be all 0.0 to indicate it isn't set yet
    this->_last_mag_ned_vec[0] = 0.0;
    this->_last_mag_ned_vec[1] = 0.0;
    this->_last_mag_ned_vec[2] = 0.0;
}

Naviguider::~Naviguider() {
    perf_free(_sample_perf);
    perf_free(_comms_errors);
    perf_free(_accel_pub_interval_perf);
    perf_free(_gyro_pub_interval_perf);
    perf_free(_mag_pub_interval_perf);
    perf_free(_baro_pub_interval_perf);

    ScheduleClear();
}

int Naviguider::init() {

    parameters_update();

    if (!_param_sens_en_ng.get()) {
	PX4_WARN("Naviguider disabled");
	return PX4_ERROR;
}

    // Debug value chooser
    this->_dbg_round_robin_sensor = 0;


    this->_last_print_time = 0;
    this->_last_sens_select_time = 0;

    // reset the sensor accel & gyro frequency calculations.
    reset_sensor_counts();

    // Check the mode, measurements only or EKF override
    if (this->_param_ng_mode_initial_value == 1) {
	this->_att_pub = new uORB::PublicationMulti<vehicle_attitude_s>(ORB_ID(vehicle_attitude));

	int32_t v = 0;

	// EKF2_EN 0 (disabled)
	v = 0;
	param_set(param_find("EKF2_EN"), &v);

	// SENS_IMU_MODE (VN handles sensor selection)
	v = 0;
	param_set(param_find("SENS_IMU_MODE"), &v);

	// SENS_MAG_MODE (VN handles sensor selection)
	v = 0;
	param_set(param_find("SENS_MAG_MODE"), &v);
    } else {
	this->_att_pub = new uORB::PublicationMulti<vehicle_attitude_s>(ORB_ID(external_ins_attitude));

	int32_t v = 1;

	// EKF2_EN 0 (disabled)
	v = 1;
	param_set(param_find("EKF2_EN"), &v);

	// SENS_IMU_MODE (VN handles sensor selection)
	v = 1;
	param_set(param_find("SENS_IMU_MODE"), &v);

	// SENS_MAG_MODE (VN handles sensor selection)
	v = 1;
	param_set(param_find("SENS_MAG_MODE"), &v);
    }

    // initialize the base I2C device
    const int ret = device::I2C::init();
    if (ret != PX4_OK) {
	PX4_ERR("I2C init failed (%d)", ret);
	return ret;
    }

    // Reset device
    if (this->write_reg_u8(RESET_REQ_REG, 1) != PX4_OK) {
	PX4_ERR("RESET_REQ_REG write failed");
	return PX4_ERROR;
    }
    px4_usleep(RESET_DELAY_US);

    // Put Sentral CPU into RUN
    if (this->write_reg_u8(CHIP_CONTROL_REG, CHIP_CONTROL_CPU_RUN) != PX4_OK) {
	PX4_ERR("CHIP_CONTROL_REG write failed");
	return PX4_ERROR;
    }

    // Flush anything stale
    if (this->write_reg_u8(FIFO_FLUSH_REG, FIFO_FLUSH_DISCARD_ALL) != PX4_OK) {
	PX4_ERR("FIFO_FLUSH_REG write failed");
	return PX4_ERROR;
    }

    // Wait for device to be ready for parameter transactions
    uint8_t host = 0, chip = 0, err = 0;
    bool ready = false;
    int last_read_ret = PX4_OK;

    for (uint32_t i = 0; i < BOOT_TIMEOUT_ITERATIONS; i++) {
	last_read_ret = this->read_reg_u8(HOST_STATUS_REG, host);  // reset host status
	if (last_read_ret != PX4_OK) {
	    px4_usleep(STATUS_CHECK_DELAY_US);
	    continue;
	}

	last_read_ret = this->read_reg_u8(CHIP_STATUS_REG, chip);  // EEPROM detected
	if (last_read_ret != PX4_OK) {
	    px4_usleep(STATUS_CHECK_DELAY_US);
	    continue;
	}

	last_read_ret = this->read_reg_u8(ERR_REG, err);  // set error register to no error
	if (last_read_ret != PX4_OK) {
	    px4_usleep(STATUS_CHECK_DELAY_US);
	    continue;
	}

	const bool host_reset_cleared = ((host & HOST_STATUS_RESET) == 0);

	// For EM7186-style parts, EEPROM upload done is the key "boot completed" signal.
	// Do NOT require CHIP_STATUS_FIRMWARE_IDLE
	const bool eeprom_done = ((chip & CHIP_STATUS_EEPROM_UPLOAD_DONE) != 0);
	const bool eeprom_err  = ((chip & CHIP_STATUS_EEPROM_UPLOAD_ERROR) != 0);

	if (host_reset_cleared && eeprom_done && !eeprom_err) {
	    ready = true;
	    break;
	}

	px4_usleep(STATUS_CHECK_DELAY_US);
    }

    PX4_INFO("NG host=0x%02x chip=0x%02x err=0x%02x ready=%d last_read_ret=%d",
	     host, chip, err, (int)ready, last_read_ret);

    if (!ready) {
	PX4_ERR("NG not ready for param writes (host=0x%02x chip=0x%02x err=0x%02x last_read_ret=%d)",
		host, chip, err, last_read_ret);
	return PX4_ERROR;
    }

    // If algorithm standby is set, clear/adjust host interface ctrl as needed
    if (host & HOST_STATUS_ALGORITHM_STANDBY) {
	if (this->write_reg_u8(HOST_INTERFACE_CTRL_REG, 0x00) != PX4_OK) {
	    PX4_ERR("HOST_INTERFACE_CTRL_REG write failed");
	    return PX4_ERROR;
	}
	px4_usleep(STATUS_CHECK_DELAY_US);
    }

    this->_last_rate_acc    = 0;
    this->_last_rate_gyro   = 0;
    this->_last_rate_mag    = 0;
    this->_last_rate_baro   = 0;
    this->_last_rate_rotvec = 0;

    // Update the sensor rates based on the params, passing any error through
    err = this->update_sensor_rates();
    if (err) {
	return err;
    }

    this->set_scale_factors();

    // Clear out our last sensor info struct
    memset(&(this->_last_sensor_info), 0x00, sizeof(SensorInfo));
    memset(&(this->_last_phys_sensor_info), 0x00, sizeof(PhysSensorInfo));

    // Schedule periodic polling
    ScheduleOnInterval(100);

    PX4_INFO("NaviGuider init on I2C addr=0x%02x", _addr);
    return PX4_OK;
}

int Naviguider::print_status() {
    PX4_INFO("Running on I2C bus %d, address 0x%02X", get_device_bus(), get_device_address());

    return 0;
}

void Naviguider::Run() {
    if (should_exit()) {
	ScheduleClear();
	//exit_and_cleanup();
	return;
    }

    const hrt_abstime now = hrt_absolute_time();

    perf_begin(_sample_perf);

    const uint32_t bytes = this->read_fifo(_fifo_buf);

    if (bytes > 0 && bytes <= FIFO_BUF_SIZE) {
	this->parse_fifo(_fifo_buf, bytes);

    } else if (bytes > FIFO_BUF_SIZE) {
	PX4_WARN("FIFO overflow: bytes=%lu > buf=%u",
		 static_cast<uint32_t>(bytes), (unsigned)FIFO_BUF_SIZE);
    }

    perf_end(_sample_perf);

    // If we're in INS mode, then every 100ms, publish sensor selection
    if (   (this->_param_ng_mode_initial_value == 1)
	&& (now - this->_last_sens_select_time > 100000)) {
	this->publish_sensor_selection();
    }

    if (now - this->_last_print_time > 1000000) {
	// Print out sensor update rate info
	print_sensor_counts();
	// Check for any parameter updates
	this->parameters_update();
	this->debug_pub();
	this->_last_print_time = now;
    }

    // Update the local mag field value for the synth mag, if necessary
    this->update_synthetic_mag_field_from_readings();
}

/* ---- Data Handling Begin ---- */
uint32_t Naviguider::read_fifo(uint8_t *buffer) {
    // Query the device for how many bytes are available in the FIFO
    uint16_t raw = 0;
    if (this->read_reg(BYTES_REMAINING_REG, &raw, sizeof(raw)) != PX4_OK) {
	    PX4_WARN("BYTES_REMAINING_REG read failed");
	    return 0;
    }

    // Handle potential endianness mismatch
    uint16_t bytes_remaining = raw;
    const uint16_t swapped = static_cast<uint16_t>(((raw >> 8) | (raw << 8)));

    /* If the raw value seems too large but the swapped value is reasonable,
    use the swapped value. Defensive check for byte-order issues.*/
    if (bytes_remaining > FIFO_BUF_SIZE && swapped <= FIFO_BUF_SIZE) {
		bytes_remaining = swapped;
    }

    // No data is available
    if (bytes_remaining == 0) {
	    return 0;
    }

    // Cap the read amount to our buffer size
    uint32_t to_read = bytes_remaining;
    if (to_read > FIFO_BUF_SIZE) {
	    to_read = FIFO_BUF_SIZE;
    }

    /* Read FIFO data in chunks. The FIFO has a 50-byte register window that wraps around,
    so we can't read more than 50 bytes at a time from sequential registers.*/
    uint32_t offset = 0;

    while (offset < to_read) {
	// Calculate register address (wraps at FIFO_WINDOW_BYTES)
	const uint32_t reg = offset % FIFO_WINDOW_BYTES;

	// Calculate how many bytes until we hit the window boundary
	const uint32_t until_boundary = FIFO_WINDOW_BYTES - reg;

	/* Determine chunk size: smallest of:
	1. Bytes remaining to read
	2. Bytes until window wraps
	3. Max I2C transaction size
	*/
	const uint32_t chunk = math::min<uint32_t>(
	    to_read - offset,
	    math::min<uint32_t>(until_boundary, I2C_MAX_READ));

	// Read chunk into buffer
	if (this->read_reg((uint8_t)reg, buffer + offset, chunk) != PX4_OK) {
	    PX4_WARN("FIFO read failed at reg=%u chunk=%lu", (unsigned)reg, static_cast<uint32_t>(chunk));
	    return 0;
		}

	offset += chunk;
    }

    return to_read;
}

uint32_t Naviguider::parse_fifo(uint8_t *buffer, uint32_t len) {
    uint32_t idx = 0;

    // Parse all packets in the buffer sequentially
    while (idx < len) {
	// Parse one packet starting at current position
	const uint32_t used = this->parse_next_fifo_block(&buffer[idx], len - idx);
	// If parsing failed (incomplete packet or unknown sensor ID), stop
	if (used == 0) {
		PX4_WARN("parse stopped: idx=%lu sensorId=0x%02x remaining=%lu",
		 (uint32_t)idx, buffer[idx], (uint32_t)(len - idx));
		break;
	}

	// Move onto the next packet
	idx += used;
	}

    // Return total bytes successfully parsed
    return idx;
}

/*Parse a single sensor packet from the FIFO buffer.
Extracts raw data, converts to physical units, and publishes to appropriate uORB topic.
Returns number of bytes consumed (0 if incomplete packet or unknown sensor type).*/
uint32_t Naviguider::parse_next_fifo_block(uint8_t *buf, uint32_t remaining) {
    if (remaining == 0) {
	return 0;
    }

    // Every packet starts with a sensor ID byte.
    const uint8_t sensorId = buf[0];

    // Skip null padding bytes.
    if (sensorId == 0) {
	uint32_t n = 0;
	while (n < remaining && buf[n] == 0) {
	    n++;
	}
	return n;
    }

    switch (sensorId) {
    /* 3-axis sensors: accelerometer, gyroscope, magnetometer.
    Packet format: [ID][X_lo][X_hi][Y_lo][Y_hi][Z_lo][Z_hi][status] = 8 bytes. */
    case SENSOR_TYPE_ACCELEROMETER:
    case SENSOR_TYPE_ACCELEROMETER_WAKE:
    case SENSOR_TYPE_GYROSCOPE:
    case SENSOR_TYPE_GYROSCOPE_WAKE:
    case SENSOR_TYPE_MAGNETIC_FIELD:
    case SENSOR_TYPE_MAGNETIC_FIELD_WAKE:
	return handle_sensor_data_3axis(remaining, sensorId, buf+1);

    /* Barometer sensor
    Packet format: [ID][P_lo][P_mid][P_hi] = 4 bytes (24-bit pressure value). */
    case SENSOR_TYPE_PRESSURE:
    case SENSOR_TYPE_PRESSURE_WAKE:
	return handle_pressure(remaining, sensorId, buf+1);

    /*Vehichle Attitude vector (fused attitude quaternion from onboard sensor fusion)
    Packet format: [ID][X_lo][X_hi][Y_lo][Y_hi][Z_lo][Z_hi][W_lo][W_hi][acc_lo][acc_hi] = 11 bytes*/
    case SENSOR_TYPE_ROTATION_VECTOR:
    case SENSOR_TYPE_ROTATION_VECTOR_WAKE:
	return handle_rotation_vector(remaining, sensorId, buf+1);

    // Timestamp packets (3 bytes) - currently unused, just skip
    case SENSOR_TYPE_TIMESTAMP:
    case SENSOR_TYPE_TIMESTAMP_WAKE:
    case SENSOR_TYPE_TIMESTAMP_OVERFLOW:
    case SENSOR_TYPE_TIMESTAMP_OVERFLOW_WAKE:
	return (remaining >= 3) ? 3 : 0;

    // Metadata packets (4 bytes) - currently unused, just skip
    case SENSOR_TYPE_META:
    case SENSOR_TYPE_META_WAKE:
	return handle_meta_event(remaining, buf+1);

    // Unknown sensor ID - stop parsing to prevent corruption
    default:
	PX4_WARN("Unknown FIFO sensorId=0x%02x, dropping %lu bytes", sensorId, (uint32_t)remaining);
	return 0;
    }
}

uint32_t Naviguider::handle_sensor_data_3axis(uint32_t remaining,
						uint8_t sensor_id,
						uint8_t* data_start) {
    if (remaining < 8) {
	return 0;
    }

    const hrt_abstime timestamp = hrt_absolute_time();

    // Extract raw 16-bit X, Y, Z values and status byte.
    SensorData3AxisRaw raw{};
    memcpy(&raw, data_start, sizeof(raw));

    // Convert from raw integers to physical units and publish.
    const float s = _scale[sensor_id];

    // if(_param_ng_mode.get() == 1){
    if (sensor_id == SENSOR_TYPE_ACCELEROMETER || sensor_id == SENSOR_TYPE_ACCELEROMETER_WAKE) {
	// publish_accel(timestamp, raw.x * s, raw.y * s, raw.z * s);
	publish_accel(timestamp, raw.y * s, raw.x * s, -raw.z * s);
    } else if (sensor_id == SENSOR_TYPE_GYROSCOPE || sensor_id == SENSOR_TYPE_GYROSCOPE_WAKE) {
	// publish_gyro(timestamp, raw.x * s, raw.y * s, raw.z * s);
	 publish_gyro(timestamp, raw.y * s, raw.x * s, -raw.z * s);
    } else {
	// publish_mag(timestamp, raw.x * s, raw.y * s, raw.z * s);
	publish_mag(timestamp, raw.y * s, raw.x * s, -raw.z * s);
    }
    // }

    return 8;
}

uint32_t Naviguider::handle_pressure(uint32_t remaining,
				     uint8_t sensor_id,
				     uint8_t* data_start) {
    if (remaining < 4) {
	return 0;
    }

    const hrt_abstime timestamp = hrt_absolute_time();

    // Reconstruct 24-bit little-endian pressure value.
    uint32_t raw = static_cast<uint32_t>((0x00000000
			       | data_start[0]
			       | data_start[1] << 8
			       | data_start[2] << 16));

    // Convert to Pascals and publish.
    const float pressure_pa = static_cast<float>((raw * _scale[sensor_id]));

    publish_baro(timestamp, pressure_pa, NAN);
    return 4;
}

uint32_t Naviguider::handle_rotation_vector(uint32_t remaining,
					    uint8_t sensor_id,
					    uint8_t* data_start) {
    if (remaining < 11) {
	return 0;
    }

    const hrt_abstime timestamp = hrt_absolute_time();

    // Extract quaternion components (x, y, z, w) and accuracy estimate
    RotationVectorRaw10 raw{};
    memcpy(&raw, data_start, sizeof(raw));

    // Convert to unit quaternion and publish attitude
    const float s = _scale[sensor_id];
    publish_attitude(timestamp, raw.w * s, raw.x * s, raw.y * s, raw.z * s);
    return 11;
}
/* ---- Data Handling End ---- */

/* ---- Publishing Begin ---- */
void Naviguider::publish_sensor_selection() {
    sensor_selection_s sens_select{};
    sens_select.accel_device_id = get_device_id();
    sens_select.gyro_device_id = get_device_id();
    sens_select.timestamp = hrt_absolute_time();
    this->_sens_select_pub.publish(sens_select);
}

void Naviguider::publish_accel(const hrt_abstime &timestamp, float x, float y, float z) {
    _accel_count++;
    perf_count(_accel_pub_interval_perf);
    _px4_accel->update(timestamp, x, y, z);
}

void Naviguider::publish_gyro(const hrt_abstime &timestamp, float x, float y, float z) {
    _gyro_count++;
    perf_count(_gyro_pub_interval_perf);
    _px4_gyro->update(timestamp, (static_cast<float>(-1.0)) * x, (static_cast<float>(-1.0)) * y, z);
}

void Naviguider::publish_mag(const hrt_abstime &timestamp, float x, float y, float z) {
    // Update the last body frame mag value for updating the NED mag field
    this->_last_mag_body_vec[0] = x;
    this->_last_mag_body_vec[1] = y;
    this->_last_mag_body_vec[2] = z;

    _mag_count++;
    perf_count(_mag_pub_interval_perf);
    _px4_mag->update(timestamp, x, y, z);
}

void Naviguider::publish_baro(const hrt_abstime &timestamp, float pressure_pa, float temperature_c) {
    _baro_count++;
    perf_count(_baro_pub_interval_perf);

    sensor_baro_s baro{};
    baro.timestamp_sample = timestamp;
    baro.device_id = get_device_id();
    baro.pressure = pressure_pa;
    baro.temperature = temperature_c;
    baro.error_count = perf_event_count(_comms_errors);
    baro.timestamp = hrt_absolute_time();

    _baro_pub.publish(baro);
}

void Naviguider::publish_attitude(const hrt_abstime &timestamp, float qw, float qx, float qy, float qz) {
    const hrt_abstime time_now_us = hrt_absolute_time();

    _attitude_count++;

    vehicle_attitude_s msg{};
    msg.timestamp_sample = time_now_us;
    msg.timestamp = msg.timestamp_sample;

    float ned_qw = qw;
    float ned_qx = qy;
    float ned_qy = qx;
    float ned_qz = -qz;

    // Re-Normalize the quaternion just to be sure
    matrix::Quatf q{ned_qw, ned_qx, ned_qy, ned_qz};
    float q_norm = q.norm();

    msg.q[0] = ned_qw / q_norm;
    msg.q[1] = ned_qx / q_norm;
    msg.q[2] = ned_qy / q_norm;
    msg.q[3] = ned_qz / q_norm;

    publish_synthetic_mag(time_now_us, msg.q[0], msg.q[1], msg.q[2], msg.q[3]);

    msg.quat_reset_counter = 0;
    _att_pub->publish(msg);

    if (_param_ng_mode.get() == 1) {
	estimator_status_s est_status{};
	est_status.timestamp_sample = time_now_us;

	float test_ratio = 0.1f;

	est_status.hdg_test_ratio = test_ratio;  // heading test
	est_status.vel_test_ratio = test_ratio;  // velocity test
	est_status.pos_test_ratio = test_ratio;  // position test
	est_status.hgt_test_ratio = test_ratio;  // hight test

	est_status.accel_device_id = get_device_id();
	est_status.gyro_device_id = get_device_id();

	est_status.timestamp = time_now_us;
	_est_status_pub.publish(est_status);
    }
}

// Extract a synthetic magnetometer from the NaviGuider's attitude quaternion
void Naviguider::publish_synthetic_mag(const hrt_abstime &timestamp, float qw, float qx, float qy, float qz) {
    // Record this attitude for when we want to update the mag field
    this->_last_att_quat[0] = qw;
    this->_last_att_quat[1] = qx;
    this->_last_att_quat[2] = qy;
    this->_last_att_quat[3] = qz;

    // Check if _last_mag_ned_vec has not been initialized
    if (   this->_last_mag_ned_vec[0] < static_cast<float>(0.0001)  && this->_last_mag_ned_vec[0] > static_cast<float>(-0.0001)
	&& this->_last_mag_ned_vec[1] < static_cast<float>(0.0001) && this->_last_mag_ned_vec[1] > static_cast<float>(-0.0001)
	&& this->_last_mag_ned_vec[2] < static_cast<float>(0.0001)  && this->_last_mag_ned_vec[2] > static_cast<float>(-0.0001)) {
	// Do nothing until it's updated
	return;
    }

    float dcm[3][3];

    // extracting mag from naviguider ekf quaternion and applying the local mag field scalar.
    dcm[0][0] = qw*qw + qx*qx - qy*qy - qz*qz;
    dcm[0][1] = 2*(qx*qy - qw*qz);
    dcm[0][2] = 2*(qx*qz + qw*qy);

    dcm[1][0] = 2*(qx*qy + qw*qz);
    dcm[1][1] = qw*qw - qx*qx + qy*qy - qz*qz;
    dcm[1][2] = 2*(qy*qz - qw*qx);

    dcm[2][0] = 2*(qx*qz - qw*qy);
    dcm[2][1] = 2*(qy*qz + qw*qx);
    dcm[2][2] = qw*qw - qx*qx - qy*qy + qz*qz;

    // Multiplying by the transpose of the DCM because that seemed to work
    float x =   dcm[0][0]*this->_last_mag_ned_vec[0]
	      + dcm[1][0]*this->_last_mag_ned_vec[1]
	      + dcm[2][0]*this->_last_mag_ned_vec[2];

    float y =   dcm[0][1]*this->_last_mag_ned_vec[0]
	      + dcm[1][1]*this->_last_mag_ned_vec[1]
	      + dcm[2][1]*this->_last_mag_ned_vec[2];

    float z =   dcm[0][2]*this->_last_mag_ned_vec[0]
	      + dcm[1][2]*this->_last_mag_ned_vec[1]
	      + dcm[2][2]*this->_last_mag_ned_vec[2];

    // Add some small randomness to the data to ensure it never goes stale
    x = x + (((random() % 10) + 1) * (static_cast<float>(0.000001)));
    y = y + (((random() % 10) + 1) * (static_cast<float>(0.000001)));
    z = z + (((random() % 10) + 1) * (static_cast<float>(0.000001)));

    _px4_mag_synth->update(timestamp, x, y, z);
}

/* ---- Publishing End ---- */

/* ---- Initialization Begin ---- */
void Naviguider::update_synthetic_mag_field_from_readings() {
    // If both vehicle status and vehicle local pos have not been updated
    if (!(_vehicle_status_sub.updated()) && !(_vehicle_local_pos_sub.updated())) {
	// Do nothing
	return;
    }

    vehicle_status_s vs_update;
    _vehicle_status_sub.copy(&vs_update);
    vehicle_local_position_s vlp_update;
    _vehicle_local_pos_sub.copy(&vlp_update);

    // If we're using the automatic synth mag field, we're disarmed, and we're stationary
    float vel_norm = sqrt(pow(vlp_update.vx, 2) + pow(vlp_update.vy, 2) + pow(vlp_update.vz, 2));
    if ((this->_synth_mag_mode == 0)
	&& (vs_update.arming_state == vs_update.ARMING_STATE_DISARMED)
	&& (vel_norm < static_cast<float>(0.1))) {
	float dcm[3][3];

	// Use the last recorded attitude to rotate the mag field into the NED frame
	dcm[0][0] =   this->_last_att_quat[0]*this->_last_att_quat[0]
		    + this->_last_att_quat[1]*this->_last_att_quat[1]
		    - this->_last_att_quat[2]*this->_last_att_quat[2]
		    - this->_last_att_quat[3]*this->_last_att_quat[3];

	dcm[0][1] = 2*(this->_last_att_quat[1]*this->_last_att_quat[2]
		       - this->_last_att_quat[0]*this->_last_att_quat[3]);

	dcm[0][2] = 2*(this->_last_att_quat[1]*this->_last_att_quat[3]
		       + this->_last_att_quat[0]*this->_last_att_quat[2]);

	dcm[1][0] = 2*(this->_last_att_quat[1]*this->_last_att_quat[2]
		       + this->_last_att_quat[0]*this->_last_att_quat[3]);

	dcm[1][1] =   this->_last_att_quat[0]*this->_last_att_quat[0]
		    - this->_last_att_quat[1]*this->_last_att_quat[1]
		    + this->_last_att_quat[2]*this->_last_att_quat[2]
		    - this->_last_att_quat[3]*this->_last_att_quat[3];

	dcm[1][2] = 2*(this->_last_att_quat[2]*this->_last_att_quat[3]
		       - this->_last_att_quat[0]*this->_last_att_quat[1]);

	dcm[2][0] = 2*(this->_last_att_quat[1]*this->_last_att_quat[3]
		       - this->_last_att_quat[0]*this->_last_att_quat[2]);

	dcm[2][1] = 2*(this->_last_att_quat[2]*this->_last_att_quat[3]
		       + this->_last_att_quat[0]*this->_last_att_quat[1]);

	dcm[2][2] =   this->_last_att_quat[0]*this->_last_att_quat[0]
		    - this->_last_att_quat[1]*this->_last_att_quat[1]
		    - this->_last_att_quat[2]*this->_last_att_quat[2]
		    + this->_last_att_quat[3]*this->_last_att_quat[3];

	// Multiplying by the DCM to rotate from body to NED frame
	float n =   dcm[0][0]*this->_last_mag_body_vec[0]
		  + dcm[0][1]*this->_last_mag_body_vec[1]
		  + dcm[0][2]*this->_last_mag_body_vec[2];

	float e =   dcm[1][0]*this->_last_mag_body_vec[0]
		  + dcm[1][1]*this->_last_mag_body_vec[1]
		  + dcm[1][2]*this->_last_mag_body_vec[2];

	float d =   dcm[2][0]*this->_last_mag_body_vec[0]
		  + dcm[2][1]*this->_last_mag_body_vec[1]
		  + dcm[2][2]*this->_last_mag_body_vec[2];

	// Set the record of the last magnetic field vector
	this->_last_mag_ned_vec[0] = n;
	this->_last_mag_ned_vec[1] = e;
	this->_last_mag_ned_vec[2] = d;
    }
}
/* ---- Initialization End ---- */

/* ---- I2C Begin ---- */
int Naviguider::read_reg(uint8_t reg, void *data, uint32_t len) {
    return transfer(&reg, 1, reinterpret_cast<uint8_t *>(data), len);
}

int Naviguider::read_reg_u8(uint8_t reg, uint8_t &v) {
    return this->read_reg(reg, &v, 1);
}

int Naviguider::write_reg(uint8_t reg, const void *data, uint32_t len) {
    uint8_t buf[1 + 32];
    if (len > 32) {
	return -EINVAL;
    }
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return transfer(buf, 1 + len, nullptr, 0);
}

int Naviguider::write_reg_u8(uint8_t reg, uint8_t v) {
    return this->write_reg(reg, &v, 1);
}
/* ---- I2C End ---- */

/* ---- Configuration Begin ---- */
int Naviguider::update_sensor_rates() {
    // If the last set rate isn't equal to the current param rate
    uint16_t cur_rate_acc = _param_ng_rate_acc.get();
    if (this->_last_rate_acc != cur_rate_acc) {
	// Set sensor rate, if error occurred print error message and return error.
	if (this->set_sensor_rate(SENSOR_TYPE_ACCELEROMETER, cur_rate_acc) != PX4_OK) {
	    PX4_ERR("set accel rate failed");
	    return PX4_ERROR;
	}
	// Save the current rate as the last set rate
	this->_last_rate_acc = cur_rate_acc;
    }

    uint16_t cur_rate_gyro = _param_ng_rate_gyro.get();
    if (this->_last_rate_gyro != cur_rate_gyro) {
	if (this->set_sensor_rate(SENSOR_TYPE_GYROSCOPE, cur_rate_gyro) != PX4_OK) {
	    PX4_ERR("set gyro rate failed");
	    return PX4_ERROR;
	}
	this->_last_rate_gyro = cur_rate_gyro;
    }

    uint16_t cur_rate_mag = _param_ng_rate_mag.get();
    if (this->_last_rate_mag != cur_rate_mag) {
	if (this->set_sensor_rate(SENSOR_TYPE_MAGNETIC_FIELD, cur_rate_mag) != PX4_OK) {
	    PX4_ERR("set mag rate failed");
	    return PX4_ERROR;
	}
	this->_last_rate_mag = cur_rate_mag;
    }

    uint16_t cur_rate_baro = _param_ng_rate_baro.get();
    if (this->_last_rate_baro != cur_rate_baro) {
	if (this->set_sensor_rate(SENSOR_TYPE_PRESSURE, cur_rate_baro) != PX4_OK) {
	    PX4_ERR("set pressure rate failed");
	    return PX4_ERROR;
	}
	this->_last_rate_baro = cur_rate_baro;
    }

    uint16_t cur_rate_rotvec = _param_ng_rate_rotvec.get();
    if (this->_last_rate_rotvec != cur_rate_rotvec) {
	if (this->set_sensor_rate(SENSOR_TYPE_ROTATION_VECTOR, cur_rate_rotvec) != PX4_OK) {
	    PX4_ERR("set attitude rate failed");
	    return PX4_ERROR;
	}
	this->_last_rate_rotvec = cur_rate_rotvec;
    }

    return 0;
}

void Naviguider::update_synthetic_mag_field_from_params() {
    // Check whether or not we should even pull from params for the mag field
    this->_synth_mag_mode = _param_synth_mag_mode.get();
    if (this->_synth_mag_mode == 1) {
	// Set local mag north value
	this->_last_mag_ned_vec[0] = _param_ng_local_north_mag_field.get();
	// Set local mag east value
	this->_last_mag_ned_vec[1] = _param_ng_local_east_mag_field.get();
	// Set local mag down value
	this->_last_mag_ned_vec[2] = _param_ng_local_down_mag_field.get();
    }
}

int Naviguider::set_sensor_rate(uint8_t sensorId, uint16_t rate) {
    return this->param_write(PARAM_PAGE_SENSOR_CONF, sensorId, &rate, sizeof(rate));
}

void Naviguider::set_scale_factors() {
    for (unsigned i = 0; i < 128; i++) {
	_scale[i] = 1.0f;
    }

    constexpr float ONE_OVER_2P15 = 1.0f / 32768.0f;  // 2^15
    constexpr float ONE_OVER_2P14 = 1.0f / 16384.0f;  // 2^14
    constexpr float UT_TO_GAUSS = 1.0f / 100.0f;  // uT to gauss conversion
    constexpr float PI = 3.14159265358979323846f;

    constexpr float accelDynamicRange_g = 6.0f;     // g
    constexpr float gyroDynamicRange_dps = 2000.0f;  // deg/s
    constexpr float magDynamicRange_uT = 1000.0f;  // uT

    const float accel_scale  = 9.80665f * accelDynamicRange_g * ONE_OVER_2P15;  // m/s^2 per LSB
    const float gyro_scale   = (PI / 180.0f) * gyroDynamicRange_dps * ONE_OVER_2P15;  // rad/s per LSB
    const float mag_scale    = magDynamicRange_uT * ONE_OVER_2P15 * UT_TO_GAUSS;  // gauss per LSB
    const float orient_scale = 360.0f * ONE_OVER_2P15;  // deg per LSB
    const float quat_scale   = ONE_OVER_2P14;  // unitless

    _scale[SENSOR_TYPE_ACCELEROMETER]      = accel_scale;
    _scale[SENSOR_TYPE_ACCELEROMETER_WAKE] = accel_scale;

    _scale[SENSOR_TYPE_GYROSCOPE]      = gyro_scale;
    _scale[SENSOR_TYPE_GYROSCOPE_WAKE] = gyro_scale;

    _scale[SENSOR_TYPE_MAGNETIC_FIELD]      = mag_scale;
    _scale[SENSOR_TYPE_MAGNETIC_FIELD_WAKE] = mag_scale;

    _scale[SENSOR_TYPE_PRESSURE]      = 1.0f / 128.0f;
    _scale[SENSOR_TYPE_PRESSURE_WAKE] = 1.0f / 128.0f;

    _scale[SENSOR_TYPE_ROTATION_VECTOR]      = quat_scale;
    _scale[SENSOR_TYPE_ROTATION_VECTOR_WAKE] = quat_scale;

    _scale[SENSOR_TYPE_ORIENTATION]      = orient_scale;
    _scale[SENSOR_TYPE_ORIENTATION_WAKE] = orient_scale;
}

void Naviguider::set_ned_frame(bool ned_not_enu) {
    // Read the value of the Host Interface Control register first
    uint8_t host_iface_ctrl;
    this->read_reg_u8(HOST_INTERFACE_CTRL_REG, host_iface_ctrl);

    // Ensure the Abort Transfer and Update Transfer Count bits are cleared
    host_iface_ctrl = host_iface_ctrl & (~HIC_MASK_ABORT_TX);
    host_iface_ctrl = host_iface_ctrl & (~HIC_MASK_UPDATE_TX_COUNT);
    // Change the value of the NED coordinates bit
    if (ned_not_enu) {
	// Set the NED coord bit
	host_iface_ctrl |= HIC_MASK_NED_COORD;
    } else {
	// Clear the NED coord bit
	host_iface_ctrl &= (~HIC_MASK_NED_COORD);
    }

    // Send the Host Interface Control value back
    this->write_reg_u8(HOST_INTERFACE_CTRL_REG, host_iface_ctrl);
}
/* ---- Configuration End ---- */

/* ---- Interval Counters Begin ---- */
/*Calculates and logs the total number of accelerometer and gyroscope measurements
 received since initialization (or last reset), along with their average sampling
 rates in Hz. Called every 1 second. */
void Naviguider::print_sensor_counts() {
    const hrt_abstime now = hrt_absolute_time();
    const float elapsed_sec = (now - _count_start_time) / 1000000.0f;

    const float accel_rate = (_accel_count > 0 && elapsed_sec > 0) ?
			     (_accel_count / elapsed_sec) : 0.0f;
    const float gyro_rate = (_gyro_count > 0 && elapsed_sec > 0) ?
			    (_gyro_count / elapsed_sec) : 0.0f;
    const float baro_rate = (_baro_count > 0 && elapsed_sec > 0) ?
			    (_baro_count / elapsed_sec) : 0.0f;
    const float mag_rate =  (_mag_count > 0 && elapsed_sec > 0) ?
			    (_mag_count / elapsed_sec) : 0.0f;
    const float att_rate =  (_attitude_count > 0 && elapsed_sec > 0) ?
			    (_attitude_count /elapsed_sec) : 0.0f;

    PX4_INFO("Accel: %llu (%.1f Hz), Gyro: %llu (%.1f Hz), Mag: %llu (%.1f Hz), Baro: %llu (%.1f Hz), Att: %llu (%.1f Hz) Time: %.1f s",
	static_cast<uint64_t>(_accel_count), static_cast<double>(accel_rate),
	static_cast<uint64_t>(_gyro_count), static_cast<double>(gyro_rate),
	static_cast<uint64_t>(_mag_count), static_cast<double>(mag_rate),
	static_cast<uint64_t>(_baro_count), static_cast<double>(baro_rate),
	static_cast<uint64_t>(_attitude_count), static_cast<double>(att_rate),
	static_cast<double>(elapsed_sec));
}

// Reset the counter for accel & gyro count.
void Naviguider::reset_sensor_counts() {
    this->_accel_count = 0;
    this->_gyro_count = 0;
    this->_baro_count = 0;
    this->_mag_count = 0;
    this->_attitude_count = 0;
    this->_count_start_time = hrt_absolute_time();
}
/* ---- Interval Counters End ---- */

/* ---- Debug Begin ---- */
void Naviguider::debug_pub() {
    switch (this->_dbg_round_robin_sensor) {
    case 0:
	// Get the accel sensor info
	this->get_sensor_info(SENSOR_TYPE_ACCELEROMETER);
	this->pub_last_sensor_info();
	break;
    case 1:
	// Get the accel physical sensor info
	this->get_phys_sensor_info(SENSOR_TYPE_ACCELEROMETER);
	this->pub_last_phys_sensor_info();
	break;
    case 2:
	// Get the accel sensor config
	this->get_sensor_config(SENSOR_TYPE_ACCELEROMETER);
	this->pub_last_sensor_config();
	break;
    case 3:
	// Get the mag sensor info
	this->get_sensor_info(SENSOR_TYPE_MAGNETIC_FIELD);
	this->pub_last_sensor_info();
	break;
    case 4:
	// Get the mag physical sensor info
	this->get_phys_sensor_info(SENSOR_TYPE_MAGNETIC_FIELD);
	this->pub_last_phys_sensor_info();
	break;
    case 5:
	// Get the mag sensor config
	this->get_sensor_config(SENSOR_TYPE_MAGNETIC_FIELD);
	this->pub_last_sensor_config();
	break;
    case 6:
	// Get the pressure sensor info
	this->get_sensor_info(SENSOR_TYPE_PRESSURE);
	this->pub_last_sensor_info();
	break;
    case 7:
	// Get the pressure physical sensor info
	this->get_phys_sensor_info(SENSOR_TYPE_PRESSURE);
	this->pub_last_phys_sensor_info();
	break;
    case 8:
	// Get the pressure sensor config
	this->get_sensor_config(SENSOR_TYPE_PRESSURE);
	this->pub_last_sensor_config();
	break;
    case 9:
	// Get the gyro sensor info
	this->get_sensor_info(SENSOR_TYPE_GYROSCOPE);
	this->pub_last_sensor_info();
	break;
    case 10:
	// Get the gyro physical sensor info
	this->get_phys_sensor_info(SENSOR_TYPE_GYROSCOPE);
	this->pub_last_phys_sensor_info();
	break;
    case 11:
	// Get the gyro sensor config
	this->get_sensor_config(SENSOR_TYPE_GYROSCOPE);
	this->pub_last_sensor_config();
	break;
    case 12:
	// Get the rotation vector sensor info
	this->get_sensor_info(SENSOR_TYPE_ROTATION_VECTOR);
	this->pub_last_sensor_info();
	break;
    case 13:
	// Get the rotation vector physical sensor info
	this->get_phys_sensor_info(SENSOR_TYPE_ROTATION_VECTOR);
	this->pub_last_phys_sensor_info();
	break;
    case 14:
	// Get the rotation vector sensor config
	this->get_sensor_config(SENSOR_TYPE_ROTATION_VECTOR);
	this->pub_last_sensor_config();
	break;
    case 15:
	// Get the physical sensor status
	this->get_phys_sensor_status();
	this->pub_last_phys_sensor_status();
	break;
    case 16:
	// Get the physical sensors present
	this->get_phys_sensors_present();
	this->pub_last_phys_sensors_present();
	break;
    case 17:
	// Get current warm start calibration score
	this->get_warm_start_cal_score();
	this->pub_last_warm_start_cal_score();
	break;
    }

    // Move to the next sensor on the list
    this->_dbg_round_robin_sensor = (this->_dbg_round_robin_sensor + 1) % 18;
}

void Naviguider::pub_last_phys_sensor_status() {
    debug_array_s dbg_arr;

    dbg_arr.timestamp = hrt_absolute_time();

    dbg_arr.id = PARAM_NUM_PHYS_SENSOR_STATUS;
    strncpy(dbg_arr.name, "PhySensSt", 10);
    dbg_arr.data[ 0] = this->_last_phys_sensor_status.accel_sample_rate;
    dbg_arr.data[ 1] = this->_last_phys_sensor_status.accel_dynamic_range;
    dbg_arr.data[ 2] = this->_last_phys_sensor_status.accel_flags;
    dbg_arr.data[ 3] = this->_last_phys_sensor_status.gyro_sample_rate;
    dbg_arr.data[ 4] = this->_last_phys_sensor_status.gyro_dynamic_range;
    dbg_arr.data[ 5] = this->_last_phys_sensor_status.gyro_flags;
    dbg_arr.data[ 6] = this->_last_phys_sensor_status.mag_sample_rate;
    dbg_arr.data[ 7] = this->_last_phys_sensor_status.mag_dynamic_range;
    dbg_arr.data[ 8] = this->_last_phys_sensor_status.mag_flags;

    this->_dbg_arr_pub.publish(dbg_arr);
}

void Naviguider::pub_last_phys_sensors_present() {
    debug_array_s dbg_arr;

    dbg_arr.timestamp = hrt_absolute_time();

    dbg_arr.id = PARAM_NUM_PHYS_SENSORS_PRESENT;
    strncpy(dbg_arr.name, "PhySensPr", 10);
    dbg_arr.data[ 0] = (this->_last_phys_sensors_present >>  0) & 0xff;
    dbg_arr.data[ 1] = (this->_last_phys_sensors_present >>  8) & 0xff;
    dbg_arr.data[ 2] = (this->_last_phys_sensors_present >> 16) & 0xff;
    dbg_arr.data[ 3] = (this->_last_phys_sensors_present >> 24) & 0xff;
    dbg_arr.data[ 4] = (this->_last_phys_sensors_present >> 32) & 0xff;
    dbg_arr.data[ 5] = (this->_last_phys_sensors_present >> 40) & 0xff;
    dbg_arr.data[ 6] = (this->_last_phys_sensors_present >> 48) & 0xff;
    dbg_arr.data[ 7] = (this->_last_phys_sensors_present >> 56) & 0xff;

    this->_dbg_arr_pub.publish(dbg_arr);
}

void Naviguider::pub_last_sensor_info() {
    debug_array_s dbg_arr;

    dbg_arr.timestamp = hrt_absolute_time();

    dbg_arr.id = this->_last_sensor_info.type;
    strncpy(dbg_arr.name, "Sens", 10);
    dbg_arr.data[ 0] = this->_last_sensor_info.type;
    dbg_arr.data[ 1] = this->_last_sensor_info.driver_id;
    dbg_arr.data[ 2] = this->_last_sensor_info.driver_version;
    dbg_arr.data[ 3] = this->_last_sensor_info.power;
    dbg_arr.data[ 4] = this->_last_sensor_info.max_range;
    dbg_arr.data[ 5] = this->_last_sensor_info.resolution;
    dbg_arr.data[ 6] = this->_last_sensor_info.max_rate;
    dbg_arr.data[ 7] = this->_last_sensor_info.fifo_reserved;
    dbg_arr.data[ 8] = this->_last_sensor_info.fifo_max;
    dbg_arr.data[ 9] = this->_last_sensor_info.event_size;
    dbg_arr.data[10] = this->_last_sensor_info.min_rate;

    this->_dbg_arr_pub.publish(dbg_arr);
}

void Naviguider::pub_last_phys_sensor_info() {
    debug_array_s dbg_arr;

    dbg_arr.timestamp = hrt_absolute_time();

    dbg_arr.id = this->_last_phys_sensor_info.type;
    strncpy(dbg_arr.name, "PhySens", 10);
    dbg_arr.data[ 0] = this->_last_phys_sensor_info.type;
    dbg_arr.data[ 1] = this->_last_phys_sensor_info.driver_id;
    dbg_arr.data[ 2] = this->_last_phys_sensor_info.driver_version;
    dbg_arr.data[ 3] = this->_last_phys_sensor_info.current;
    dbg_arr.data[ 4] = this->_last_phys_sensor_info.current_range;
    dbg_arr.data[ 5] = this->_last_phys_sensor_info.flags;
    dbg_arr.data[ 6] = this->_last_phys_sensor_info.reserved;
    dbg_arr.data[ 7] = this->_last_phys_sensor_info.current_rate;
    dbg_arr.data[ 8] = this->_last_phys_sensor_info.num_axes;
    dbg_arr.data[ 9] = this->_last_phys_sensor_info.orient_matrix[0];
    dbg_arr.data[10] = this->_last_phys_sensor_info.orient_matrix[1];
    dbg_arr.data[11] = this->_last_phys_sensor_info.orient_matrix[2];
    dbg_arr.data[12] = this->_last_phys_sensor_info.orient_matrix[3];
    dbg_arr.data[13] = this->_last_phys_sensor_info.orient_matrix[4];

    this->_dbg_arr_pub.publish(dbg_arr);
}

void Naviguider::pub_last_sensor_config() {
    debug_array_s dbg_arr;

    dbg_arr.timestamp = hrt_absolute_time();

    dbg_arr.id = this->_last_sensor_config_sensor_id;
    strncpy(dbg_arr.name, "SensConf", 10);
    dbg_arr.data[ 0] = this->_last_sensor_config.sample_rate;
    dbg_arr.data[ 1] = this->_last_sensor_config.max_report_latency;
    dbg_arr.data[ 2] = this->_last_sensor_config.change_sensitivity;
    dbg_arr.data[ 3] = this->_last_sensor_config.dynamic_range;

    this->_dbg_arr_pub.publish(dbg_arr);
}

void Naviguider::pub_last_warm_start_cal_score() {
    debug_array_s dbg_arr;

    dbg_arr.timestamp = hrt_absolute_time();

    dbg_arr.id = PARAM_WARM_START_CAL_SCORE;
    strncpy(dbg_arr.name, "CalScore", 10);
    dbg_arr.data[0] = this->_last_warm_start_cal_score;

    this->_dbg_arr_pub.publish(dbg_arr);
}
/* ---- Debug End ---- */

/* ---- Sensor Info Begin ---- */
int Naviguider::param_read(uint8_t page, uint8_t param_no, void *data, uint8_t size) {
    const uint8_t pageSelectValue = uint8_t(page | (size << 4));
    const uint8_t request         = uint8_t(param_no & 0x7f);  // Clear MSB

    // Track last ack for diagnostics on timeout
    uint8_t ack = 0;

    auto cleanup = [&]() {
	(void)this->write_reg_u8(PARAM_REQUEST_REG, 0);
	(void)this->write_reg_u8(PARAM_PAGE_SELECT_REG, 0);
    };

    if (this->write_reg_u8(PARAM_PAGE_SELECT_REG, pageSelectValue) != PX4_OK) {
	cleanup();
	return 1;
    }

    if (this->write_reg_u8(PARAM_REQUEST_REG, request) != PX4_OK) {
	cleanup();
	return 2;
    }

    uint8_t ack_good = 0;
    // Loop until we either error out, or the request is acknowledged
    for (uint32_t i = 0; (i < PARAM_TIMEOUT_ITERATIONS) && !ack_good; i++) {
	if (this->read_reg_u8(PARAM_ACK_REG, ack) != PX4_OK) {
	    cleanup();
	    return 3;
	}

	if (ack == request) {
	    ack_good = 1;
	}

	if (ack == 0x80) {
	    cleanup();
	    return 4;
	}

	px4_usleep(PARAM_POLL_DELAY_US);
    }

    if (!ack_good) {
	cleanup();
	PX4_ERR("param_read timeout: page=%u param=%u size=%u last_ack=0x%02x",
		(unsigned)page, (unsigned)param_no, (unsigned)size, (unsigned)ack);
	return 5;
    }

    if (this->read_reg(PARAM_SAVED_REG, data, size) != PX4_OK) {
	cleanup();
	return 6;
    }

    return 0;
}

int Naviguider::param_write(uint8_t page, uint8_t param_no, const void *data, uint8_t size) {
    const uint8_t pageSelectValue = uint8_t(page | (size << 4));
    const uint8_t request         = uint8_t(param_no | 0x80);

    // Track last ack for diagnostics on timeout
    uint8_t ack = 0;

    // lambda function clearing the registers PARAM_REQUEST_REG & PARAM_PAGE_SELECT_REG back to zero
    auto cleanup = [&]() {
	(void) this->write_reg_u8(PARAM_REQUEST_REG, 0);
	(void) this->write_reg_u8(PARAM_PAGE_SELECT_REG, 0);
    };

    if (this->write_reg_u8(PARAM_PAGE_SELECT_REG, pageSelectValue) != PX4_OK) {
	cleanup();
	return PX4_ERROR;
    }

    if (this->write_reg(PARAM_LOAD_REG, data, size) != PX4_OK) {
	cleanup();
	return PX4_ERROR;
    }

    if (this->write_reg_u8(PARAM_REQUEST_REG, request) != PX4_OK) {
	cleanup();
	return PX4_ERROR;
    }

    for (uint32_t i = 0; i < PARAM_TIMEOUT_ITERATIONS; i++) {
	if (this->read_reg_u8(PARAM_ACK_REG, ack) != PX4_OK) {
	    cleanup();
	    return PX4_ERROR;
	}

	if (ack == request) {
	    cleanup();
	    return PX4_OK;
	}

	if (ack == 0x80) {
	    cleanup();
	    return PX4_ERROR;
	}

	px4_usleep(PARAM_POLL_DELAY_US);
    }

    // Add if statement for checking if successful, currently this can
    //  only return failure

    cleanup();
    PX4_ERR("param_write timeout: page=%u param=%u size=%u last_ack=0x%02x",
	    (unsigned)page, (unsigned)param_no, (unsigned)size, (unsigned)ack);
    return PX4_ERROR;
}

int Naviguider::get_phys_sensor_status() {
    int ret_val = this->param_read(PARAM_PAGE_SYSTEM,
				   PARAM_NUM_PHYS_SENSOR_STATUS,
				   reinterpret_cast<void*> (&(this->_last_phys_sensor_status)),
				   sizeof(PhysSensorStatus));
    // If something went wrong, fill the struct with all 1's aside from the first two fields
    if (ret_val != 0) {
	memset(&(this->_last_phys_sensor_status), 0xff, sizeof(PhysSensorStatus));
	this->_last_phys_sensor_status.accel_dynamic_range = PARAM_NUM_PHYS_SENSOR_STATUS;
	this->_last_phys_sensor_status.accel_flags = ret_val;
    }

    return ret_val;
}

int Naviguider::get_phys_sensors_present() {
    int ret_val = this->param_read(PARAM_PAGE_SYSTEM,
				   PARAM_NUM_PHYS_SENSORS_PRESENT,
				   reinterpret_cast<void*> (&(this->_last_phys_sensors_present)),
				   sizeof(PhysSensorsPresent));

    // If something went wrong, fill the struct with all 1's aside from the first two fields
    if (ret_val != 0) {
	memset(&(this->_last_phys_sensors_present), 0xff, sizeof(PhysSensorsPresent));
	this->_last_phys_sensors_present &= ((static_cast<uint64_t> (PARAM_NUM_PHYS_SENSORS_PRESENT)) << 8);
	this->_last_phys_sensors_present &= ((static_cast<uint64_t> (ret_val)) << 16);
    }

    return ret_val;
}

int Naviguider::get_sensor_info(uint8_t sensorId) {
    int ret_val = this->param_read(PARAM_PAGE_SENSOR_INFO,
				   sensorId,
				   reinterpret_cast<void*>(&(this->_last_sensor_info)),
				   sizeof(SensorInfo));

    // If something went wrong, fill the struct with all 1's aside from the first two fields
    if (ret_val != 0) {
	memset(&(this->_last_sensor_info), 0xff, sizeof(SensorInfo));
	this->_last_sensor_info.driver_id = sensorId;
	this->_last_sensor_info.driver_version = ret_val;
    }

    return ret_val;
}

int Naviguider::get_phys_sensor_info(uint8_t sensorId) {
    int ret_val = this->param_read(PARAM_PAGE_SYSTEM,
				   sensorId + 32,
				   reinterpret_cast<void*> (&(this->_last_phys_sensor_info)),
				   sizeof(PhysSensorInfo));

    // If something went wrong, fill the struct with all 1's aside from the first two fields
    if (ret_val != 0) {
	memset(&(this->_last_phys_sensor_info), 0xff, sizeof(PhysSensorInfo));
	this->_last_phys_sensor_info.driver_id = sensorId;
	this->_last_phys_sensor_info.driver_version = ret_val;
    }

    return ret_val;
}

int Naviguider::get_sensor_config(uint8_t sensorId) {
    int ret_val = this->param_read(PARAM_PAGE_SENSOR_CONF,
				   sensorId,
				   reinterpret_cast<void*> (&(this->_last_sensor_config)),
				   sizeof(SensorConfig));

    this->_last_sensor_config_sensor_id = sensorId;

    // If something went wrong, fill the struct with all 1's aside from the first two fields
    if (ret_val != 0) {
	memset(&(this->_last_sensor_config), 0xff, sizeof(SensorConfig));
	this->_last_sensor_config_sensor_id = sensorId;
	this->_last_sensor_config.sample_rate = ret_val;
    }

    return ret_val;
}
/* ---- Sensor Info End ---- */

/* ---- Warm Start Begin ---- */
int Naviguider::get_warm_start_cal_score() {
    int ret_val = this->param_read(PARAM_PAGE_WARM_START,
				   PARAM_WARM_START_CAL_SCORE,
				   reinterpret_cast<void*> (&(this->_last_warm_start_cal_score)),
				   sizeof(WarmStartCalScore));

    // If something went wrong, fill the struct with all 1's aside from the first two fields
    if (ret_val != 0) {
	memset(&(this->_last_warm_start_cal_score), 0xff, sizeof(WarmStartCalScore));
    }

    return ret_val;
}
/* ---- Warm Start End ---- */

/* ---- Meta Events Begin ---- */
uint32_t Naviguider::handle_meta_event(uint32_t remaining, uint8_t* data_start) {
    if (remaining < 4) {
	return 0;
    }

    // First byte is the event ID
    uint8_t event_id = data_start[0];

    switch (event_id) {
	case META_FLUSH_COMPLETE:
	    break;
	case META_SAMPLE_RATE_CHANGED:
	    this->handle_meta_sample_rate_changed(data_start+1);
	    break;
	case META_POWER_MODE_CHANGED:
	    break;
	case META_ERROR:
	    break;
	case META_MAGNETIC_TRANSIENT:
	    this->handle_meta_magnetic_transient(data_start+1);
	    break;
	case META_CAL_STATUS_CHANGED:
	    this->handle_meta_cal_status_changed(data_start+1);
	    break;
	case META_STILLNESS_CHANGED:
	    break;
	case META_AVAILABLE:
	    break;
	case META_CALIBRATION_STABLE:
	    this->handle_meta_calibration_stable(data_start+1);
	    break;
	case META_SENSOR_ERROR:
	    this->handle_meta_sensor_error(data_start+1);
	    break;
	case META_FIFO_OVERFLOW:
	    break;
	case META_DYNAMIC_RANGE_CHANGED:
	    this->handle_meta_dynamic_range_changed(data_start+1);
	    break;
	case META_FIFO_WATERMARK:
	    break;
	case META_SELF_TEST_RESULTS:
	    this->handle_meta_self_test_results(data_start+1);
	    break;
	case META_INITIALIZED:
	    this->handle_meta_initialized(data_start+1);
	    break;
	case META_TRANSFER_CAUSE:
	    break;
    }

    // Return that we consumed the event bytes
    return 4;
}

void Naviguider::handle_meta_sample_rate_changed(uint8_t* data_start) {
    debug_array_s meta_arr;

    meta_arr.timestamp = hrt_absolute_time();

    uint8_t sensor_type = data_start[0];
    // Use this to confirm that our attempt at changing the sample rate was successful.
    meta_arr.id = META_SAMPLE_RATE_CHANGED;
    strncpy(meta_arr.name, "SmplRtCH", 10);
    meta_arr.data[0] = sensor_type;

    this->_dbg_arr_pub.publish(meta_arr);
}

void Naviguider::handle_meta_power_mode_changed(uint8_t* data_start) {
    debug_array_s meta_arr;

    meta_arr.timestamp = hrt_absolute_time();

    uint8_t sensor_type = data_start[0];
    uint8_t power_mode = data_start[1];
    meta_arr.id = META_POWER_MODE_CHANGED;
    strncpy(meta_arr.name, "PwrModeCh", 10);
    meta_arr.data[0] = sensor_type;
    meta_arr.data[1] = power_mode;
    this->_dbg_arr_pub.publish(meta_arr);
}

void Naviguider::handle_meta_magnetic_transient(uint8_t* data_start) {
    debug_array_s meta_arr;
    meta_arr.timestamp = hrt_absolute_time();

    uint8_t transient = data_start[0];
    meta_arr.id = META_MAGNETIC_TRANSIENT;
    strncpy(meta_arr.name, "MagTrans", 10);
    meta_arr.data[0] = transient;
    this->_dbg_arr_pub.publish(meta_arr);
}

void Naviguider::handle_meta_cal_status_changed(uint8_t* data_start) {
    debug_array_s meta_arr;
    meta_arr.timestamp = hrt_absolute_time();

    uint8_t cal_status_value = data_start[0];
    uint8_t trans_comp = data_start[1];
    // Use this to set a flag on our calibration state.
    meta_arr.id = META_CAL_STATUS_CHANGED;
    strncpy(meta_arr.name, "CalStat", 10);
    meta_arr.data[0] = cal_status_value;
    meta_arr.data[1] = trans_comp;
    this->_dbg_arr_pub.publish(meta_arr);
}

void Naviguider::handle_meta_calibration_stable(uint8_t* data_start) {
    debug_array_s meta_arr;
    meta_arr.timestamp = hrt_absolute_time();

    uint8_t calibration_stable = data_start[0];
    // Use this to inform whether or not to trust the data.
    meta_arr.id = META_CALIBRATION_STABLE;
    strncpy(meta_arr.name, "CalStbl", 10);
    meta_arr.data[0] = calibration_stable;
    this->_dbg_arr_pub.publish(meta_arr);
}

void Naviguider::handle_meta_sensor_error(uint8_t* data_start) {
    debug_array_s meta_arr;
    meta_arr.timestamp = hrt_absolute_time();

    uint8_t sensor_type = data_start[0];
    uint8_t sensor_status = data_start[1];
    meta_arr.id = META_SENSOR_ERROR;
    strncpy(meta_arr.name, "SensErr", 10);
    meta_arr.data[0] = sensor_type;
    meta_arr.data[1] = sensor_status;
    this->_dbg_arr_pub.publish(meta_arr);
}

void Naviguider::handle_meta_dynamic_range_changed(uint8_t* data_start) {
    debug_array_s meta_arr;
    meta_arr.timestamp = hrt_absolute_time();

    uint8_t sensor_type = data_start[0];
    // Use this to change the data scaling for the given sensor.
    meta_arr.id = META_DYNAMIC_RANGE_CHANGED;
    strncpy(meta_arr.name, "DynRngCH", 10);
    meta_arr.data[0] = sensor_type;
    this->_dbg_arr_pub.publish(meta_arr);
}

void Naviguider::handle_meta_self_test_results(uint8_t* data_start) {
    debug_array_s meta_arr;
    meta_arr.timestamp = hrt_absolute_time();

    uint8_t sensor_type = data_start[0];
    uint8_t test_result = data_start[1];
    // Use this to inform whether or not to trust the data.
    meta_arr.id = META_SELF_TEST_RESULTS;
    strncpy(meta_arr.name, "SlfAbsT", 10);
    meta_arr.data[0] = sensor_type;
    meta_arr.data[1] = test_result;
    this->_dbg_arr_pub.publish(meta_arr);
}

void Naviguider::handle_meta_initialized(uint8_t* data_start) {
    debug_array_s meta_arr;
    meta_arr.timestamp = hrt_absolute_time();

    uint8_t ram_ver_lsb = data_start[0];
    uint8_t ram_ver_msb = data_start[1];
    meta_arr.id = META_INITIALIZED;
    strncpy(meta_arr.name, "Intlzd", 10);
    meta_arr.data[0] = ram_ver_lsb;
    meta_arr.data[1] = ram_ver_msb;
    this->_dbg_arr_pub.publish(meta_arr);
}
/* ---- Meta Events End ---- */

/* ---- Parameters Begin ---- */
void Naviguider::parameters_update() {
    if (_parameter_update_sub.updated()) {
	parameter_update_s param_update;
	_parameter_update_sub.copy(&param_update);
	// Note: We do not do anything with the param_update struct, it only
	//       says that an update has occurred, not what has been updated.

	// Update all the param values to potentially new values
	updateParams();

	// Update sensor rates based on the potentially changed rate values
	this->update_sensor_rates();

	// Update the local mag field based on the potentially changed values
	this->update_synthetic_mag_field_from_params();
    }
}
/* ---- Parameters End ---- */

ModuleBase::Descriptor Naviguider::module_desc {
	&Naviguider::task_spawn,
	&Naviguider::custom_command,
	&Naviguider::print_usage
};
