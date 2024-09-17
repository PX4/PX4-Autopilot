/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "VectorNav.hpp"

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/getopt.h>

#include <fcntl.h>

using matrix::Vector2f;

VectorNav::VectorNav(const char *port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_attitude_pub((_param_vn_mode.get() == 0) ? ORB_ID(external_ins_attitude) : ORB_ID(vehicle_attitude)),
	_local_position_pub((_param_vn_mode.get() == 0) ? ORB_ID(external_ins_local_position) : ORB_ID(vehicle_local_position)),
	_global_position_pub((_param_vn_mode.get() == 0) ? ORB_ID(external_ins_global_position) : ORB_ID(
				     vehicle_global_position))
{
	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';

	// VN_MODE 1 full INS
	if (_param_vn_mode.get() == 1) {
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
	}

	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_VN300;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	_px4_accel.set_device_id(device_id.devid);
	_px4_gyro.set_device_id(device_id.devid);
	_px4_mag.set_device_id(device_id.devid);

	// uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	// if (bus_num < 10) {
	// 	device_id.devid_s.bus = bus_num;
	// }

	_attitude_pub.advertise();
	_local_position_pub.advertise();
	_global_position_pub.advertise();
}

VectorNav::~VectorNav()
{
	VnSensor_unregisterAsyncPacketReceivedHandler(&_vs);
	VnSensor_unregisterErrorPacketReceivedHandler(&_vs);
	VnSensor_disconnect(&_vs);

	perf_free(_sample_perf);
	perf_free(_comms_errors);

	perf_free(_accel_pub_interval_perf);
	perf_free(_gyro_pub_interval_perf);
	perf_free(_mag_pub_interval_perf);
	perf_free(_gnss_pub_interval_perf);
	perf_free(_baro_pub_interval_perf);

	perf_free(_attitude_pub_interval_perf);
	perf_free(_local_position_pub_interval_perf);
	perf_free(_global_position_pub_interval_perf);
}

void VectorNav::binaryAsyncMessageReceived(void *userData, VnUartPacket *packet, size_t runningIndex)
{
	if (VnUartPacket_isError(packet)) {
		uint8_t error = 0;
		VnUartPacket_parseError(packet, &error);

		char buffer[128] {};
		strFromSensorError(buffer, (SensorError)error);

		PX4_ERR("%s", buffer);

	} else if (userData && (VnUartPacket_type(packet) == PACKETTYPE_BINARY)) {
		static_cast<VectorNav *>(userData)->sensorCallback(packet);
	}
}

void VectorNav::sensorCallback(VnUartPacket *packet)
{
	const hrt_abstime time_now_us = hrt_absolute_time();

	//BinaryGroupType groups = (BinaryGroupType)VnUartPacket_groups(packet);

	//size_t curGroupFieldIndex = 0;

	// low pass offset between VN and hrt
	// use timestamp for publish and timestamp_sample

	// binary output 1
	if (VnUartPacket_isCompatible(packet,
				      COMMONGROUP_NONE,
				      TIMEGROUP_TIMESTARTUP,
				      (ImuGroup)(IMUGROUP_ACCEL | IMUGROUP_ANGULARRATE),
				      GPSGROUP_NONE,
				      ATTITUDEGROUP_NONE,
				      INSGROUP_NONE,
				      GPSGROUP_NONE)
	   ) {
		// TIMEGROUP_TIMESTARTUP
		uint64_t time_startup = VnUartPacket_extractUint64(packet);
		(void)time_startup;

		// IMUGROUP_ACCEL
		vec3f accel = VnUartPacket_extractVec3f(packet);

		// IMUGROUP_ANGULARRATE
		vec3f angular_rate = VnUartPacket_extractVec3f(packet);

		// publish sensor_accel
		_px4_accel.update(time_now_us, accel.c[0], accel.c[1], accel.c[2]);
		perf_count(_accel_pub_interval_perf);

		// publish sensor_gyro
		_px4_gyro.update(time_now_us, angular_rate.c[0], angular_rate.c[1], angular_rate.c[2]);
		perf_count(_gyro_pub_interval_perf);

		_time_last_valid_imu_us.store(hrt_absolute_time());
	}

	// binary output 2
	if (VnUartPacket_isCompatible(packet,
				      COMMONGROUP_NONE,
				      TIMEGROUP_TIMESTARTUP,
				      (ImuGroup)(IMUGROUP_TEMP | IMUGROUP_PRES | IMUGROUP_MAG),
				      GPSGROUP_NONE,
				      (AttitudeGroup)(ATTITUDEGROUP_QUATERNION | ATTITUDEGROUP_LINEARACCELNED),
				      (InsGroup)(INSGROUP_INSSTATUS | INSGROUP_POSLLA | INSGROUP_VELNED | INSGROUP_POSU | INSGROUP_VELU),
				      GPSGROUP_NONE)
	   ) {
		// TIMEGROUP_TIMESTARTUP
		const uint64_t time_startup = VnUartPacket_extractUint64(packet);
		(void)time_startup;

		// IMUGROUP_TEMP
		const float temperature = VnUartPacket_extractFloat(packet);

		// IMUGROUP_PRES
		const float pressure = VnUartPacket_extractFloat(packet) * 1000.f; // kPa -> Pa

		// IMUGROUP_MAG
		const vec3f mag = VnUartPacket_extractVec3f(packet);

		// ATTITUDEGROUP_QUATERNION
		const vec4f quaternion = VnUartPacket_extractVec4f(packet);

		// ATTITUDEGROUP_LINEARACCELNED
		const vec3f accelerationLinearNed = VnUartPacket_extractVec3f(packet);

		// INSGROUP_INSSTATUS
		const uint16_t insStatus = VnUartPacket_extractUint16(packet);

		// INSGROUP_POSLLA
		const vec3d positionEstimatedLla = VnUartPacket_extractVec3d(packet);

		// INSGROUP_VELNED
		const vec3f velocityEstimatedNed = VnUartPacket_extractVec3f(packet);

		// INSGROUP_POSU
		const float positionUncertaintyEstimated = VnUartPacket_extractFloat(packet);

		// INSGROUP_VELU
		const float velocityUncertaintyEstimated = VnUartPacket_extractFloat(packet);


		// update all temperatures
		_px4_accel.set_temperature(temperature);
		_px4_gyro.set_temperature(temperature);
		_px4_mag.set_temperature(temperature);

		// publish sensor_baro
		sensor_baro_s sensor_baro{};
		sensor_baro.device_id = 0; // TODO: DRV_INS_DEVTYPE_VN300;
		sensor_baro.pressure = pressure;
		sensor_baro.temperature = temperature;
		sensor_baro.timestamp = hrt_absolute_time();
		_sensor_baro_pub.publish(sensor_baro);
		perf_count(_baro_pub_interval_perf);

		// publish sensor_mag
		_px4_mag.update(time_now_us, mag.c[0], mag.c[1], mag.c[2]);
		perf_count(_mag_pub_interval_perf);

		// publish attitude
		vehicle_attitude_s attitude{};
		attitude.timestamp_sample = time_now_us;
		attitude.q[0] = quaternion.c[3];
		attitude.q[1] = quaternion.c[0];
		attitude.q[2] = quaternion.c[1];
		attitude.q[3] = quaternion.c[2];
		attitude.timestamp = hrt_absolute_time();
		_attitude_pub.publish(attitude);
		perf_count(_attitude_pub_interval_perf);

		// mode
		const uint16_t mode = (insStatus & 0b11);
		//const bool mode_initializing = (mode == 0b00);
		const bool mode_aligning     = (mode == 0b01);
		const bool mode_tracking     = (mode == 0b10);
		//const bool mode_loss_gnss    = (mode == 0b11);


		// mode_initializing
		//  - pitch and roll are good (yaw is mag based)

		// mode_aligning.
		//  - attitude good
		//  - position and velocity are good
		//  - heading determined largely by mag and not by GPS
		//  - heading has high uncertainly (yaw is good for control)

		// mode_tracking
		//  - heading is good for control
		//

		// mode_loss_gnss
		//  - lost for > 45 seconds
		//  - attitude is good
		//  - treat as mode 0

		if (mode_aligning || mode_tracking) {
			// publish local_position

			const double lat = positionEstimatedLla.c[0];
			const double lon = positionEstimatedLla.c[1];
			const float alt = positionEstimatedLla.c[2];

			if (!_pos_ref.isInitialized()) {
				_pos_ref.initReference(lat, lon, time_now_us);
				_gps_alt_ref = alt;
			}

			const Vector2f pos_ned = _pos_ref.project(lat, lon);

			vehicle_local_position_s local_position{};
			local_position.timestamp_sample = time_now_us;

			local_position.xy_valid = true;
			local_position.z_valid = true;
			local_position.v_xy_valid = true;
			local_position.v_z_valid = true;

			local_position.x = pos_ned(0);
			local_position.y = pos_ned(1);
			local_position.z = -(alt - _gps_alt_ref);

			local_position.vx = velocityEstimatedNed.c[0];
			local_position.vy = velocityEstimatedNed.c[1];
			local_position.vz = velocityEstimatedNed.c[2];
			local_position.z_deriv = velocityEstimatedNed.c[2]; // TODO

			local_position.ax = accelerationLinearNed.c[0];
			local_position.ay = accelerationLinearNed.c[1];
			local_position.az = accelerationLinearNed.c[2];

			matrix::Quatf q{attitude.q};
			local_position.heading = matrix::Eulerf{q}.psi();
			local_position.heading_good_for_control = mode_tracking;

			if (_pos_ref.isInitialized()) {
				local_position.xy_global = true;
				local_position.z_global = true;
				local_position.ref_timestamp = _pos_ref.getProjectionReferenceTimestamp();
				local_position.ref_lat = _pos_ref.getProjectionReferenceLat();
				local_position.ref_lon = _pos_ref.getProjectionReferenceLon();
				local_position.ref_alt = _gps_alt_ref;
			}

			local_position.dist_bottom_valid = false;

			local_position.eph = positionUncertaintyEstimated;
			local_position.epv = positionUncertaintyEstimated;
			local_position.evh = velocityUncertaintyEstimated;
			local_position.evv = velocityUncertaintyEstimated;

			local_position.dead_reckoning = false;

			local_position.vxy_max = INFINITY;
			local_position.vz_max = INFINITY;
			local_position.hagl_min = INFINITY;
			local_position.hagl_max_z = INFINITY;
			local_position.hagl_max_xy = INFINITY;

			local_position.unaided_heading = NAN;
			local_position.timestamp = hrt_absolute_time();
			_local_position_pub.publish(local_position);
			perf_count(_local_position_pub_interval_perf);


			// publish global_position
			vehicle_global_position_s global_position{};
			global_position.timestamp_sample = time_now_us;
			global_position.lat = lat;
			global_position.lon = lon;
			global_position.alt = alt;
			global_position.alt = alt;

			global_position.eph = positionUncertaintyEstimated;
			global_position.epv = positionUncertaintyEstimated;

			global_position.timestamp = hrt_absolute_time();
			_global_position_pub.publish(global_position);
			perf_count(_global_position_pub_interval_perf);
		}

		// publish estimator_status (VN_MODE 1 only)
		if (_param_vn_mode.get() == 1) {

			estimator_status_s estimator_status{};
			estimator_status.timestamp_sample = time_now_us;

			float test_ratio = 0.f;

			if (mode_aligning) {
				test_ratio = 0.99f;

			} else if (mode_tracking) {
				// very good
				test_ratio = 0.1f;
			}

			estimator_status.hdg_test_ratio = test_ratio;
			estimator_status.vel_test_ratio = test_ratio;
			estimator_status.pos_test_ratio = test_ratio;
			estimator_status.hgt_test_ratio = test_ratio;

			estimator_status.accel_device_id = _px4_accel.get_device_id();
			estimator_status.gyro_device_id = _px4_gyro.get_device_id();

			estimator_status.timestamp = hrt_absolute_time();
			_estimator_status_pub.publish(estimator_status);

		}
	}

	// binary output 3
	if (VnUartPacket_isCompatible(packet,
				      COMMONGROUP_NONE,
				      TIMEGROUP_TIMESTARTUP,
				      IMUGROUP_NONE,
				      (GpsGroup)(GPSGROUP_UTC | GPSGROUP_NUMSATS | GPSGROUP_FIX | GPSGROUP_POSLLA | GPSGROUP_VELNED
						      | GPSGROUP_POSU | GPSGROUP_VELU | GPSGROUP_DOP),
				      ATTITUDEGROUP_NONE,
				      INSGROUP_NONE,
				      GPSGROUP_NONE)
	   ) {
		// TIMEGROUP_TIMESTARTUP
		const uint64_t time_startup = VnUartPacket_extractUint64(packet);
		(void)time_startup;

		// GPSGROUP_UTC
		TimeUtc timeUtc = VnUartPacket_extractTimeUtc(packet);
		(void)timeUtc;

		// GPSGROUP_NUMSATS
		const uint8_t numSats = VnUartPacket_extractUint8(packet);

		// GPSGROUP_FIX
		const uint8_t gpsFix = VnUartPacket_extractUint8(packet);

		// GPSGROUP_POSLLA
		const vec3d positionGpsLla = VnUartPacket_extractVec3d(packet);

		// GPSGROUP_VELNED
		const vec3f velocityGpsNed = VnUartPacket_extractVec3f(packet);

		// GPSGROUP_POSU
		const vec3f positionUncertaintyGpsNed = VnUartPacket_extractVec3f(packet);

		// GPSGROUP_VELU
		const float velocityUncertaintyGps = VnUartPacket_extractFloat(packet);

		// GPSGROUP_DOP
		const GpsDop dop = VnUartPacket_extractGpsDop(packet);


		// publish sensor_gnss
		if (gpsFix > 0) {
			sensor_gps_s sensor_gps{};
			sensor_gps.timestamp_sample = time_now_us;

			sensor_gps.device_id = 0; // TODO

			sensor_gps.time_utc_usec = 0;

			sensor_gps.satellites_used = numSats;

			sensor_gps.fix_type = gpsFix;

			sensor_gps.latitude_deg = positionGpsLla.c[0];
			sensor_gps.longitude_deg = positionGpsLla.c[1];
			sensor_gps.altitude_msl_m = positionGpsLla.c[2];
			sensor_gps.altitude_ellipsoid_m = sensor_gps.altitude_msl_m;

			sensor_gps.vel_m_s = matrix::Vector3f(velocityGpsNed.c).length();
			sensor_gps.vel_n_m_s = velocityGpsNed.c[0];
			sensor_gps.vel_e_m_s = velocityGpsNed.c[1];
			sensor_gps.vel_d_m_s = velocityGpsNed.c[2];
			sensor_gps.vel_ned_valid = true;

			sensor_gps.hdop = dop.hDOP;
			sensor_gps.vdop = dop.vDOP;

			sensor_gps.eph = sqrtf(sq(positionUncertaintyGpsNed.c[0]) + sq(positionUncertaintyGpsNed.c[1]));
			sensor_gps.epv = positionUncertaintyGpsNed.c[2];

			sensor_gps.s_variance_m_s = velocityUncertaintyGps;

			sensor_gps.timestamp = hrt_absolute_time();
			_sensor_gps_pub.publish(sensor_gps);
			perf_count(_gnss_pub_interval_perf);
		}
	}
}

bool VectorNav::init()
{
	// first try default baudrate
	const uint32_t DEFAULT_BAUDRATE = 115200;
	const uint32_t DESIRED_BAUDRATE = 921600;

	// first try default baudrate, if that fails try all other supported baudrates
	VnSensor_initialize(&_vs);

	if ((VnSensor_connect(&_vs, _port, DEFAULT_BAUDRATE) != E_NONE) || !VnSensor_verifySensorConnectivity(&_vs)) {

		VnSensor_disconnect(&_vs);

		static constexpr uint32_t BAUDRATES[] {9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600};

		for (auto &baudrate : BAUDRATES) {
			VnSensor_initialize(&_vs);

			if (VnSensor_connect(&_vs, _port, baudrate) == E_NONE && VnSensor_verifySensorConnectivity(&_vs)) {
				PX4_DEBUG("found baudrate %d", baudrate);
				break;
			}

			// disconnect before trying again
			VnSensor_disconnect(&_vs);
		}
	}

	if (!VnSensor_verifySensorConnectivity(&_vs)) {
		PX4_ERR("Error verifying sensor connectivity");
		VnSensor_disconnect(&_vs);
		return false;
	}

	VnError error = E_NONE;

	// change baudrate to max
	if ((error = VnSensor_changeBaudrate(&_vs, DESIRED_BAUDRATE)) != E_NONE) {
		PX4_ERR("Error changing baud rate failed: %d", error);
		VnSensor_disconnect(&_vs);
		return false;
	}

	// query the sensor's model number
	char model_number[30] {};

	if ((error = VnSensor_readModelNumber(&_vs, model_number, sizeof(model_number))) != E_NONE) {
		PX4_ERR("Error reading model number %d", error);
		VnSensor_disconnect(&_vs);
		return false;
	}

	// query the sensor's hardware revision
	uint32_t hardware_revision = 0;

	if ((error = VnSensor_readHardwareRevision(&_vs, &hardware_revision)) != E_NONE) {
		PX4_ERR("Error reading HW revision %d", error);
		VnSensor_disconnect(&_vs);
		return false;
	}

	// query the sensor's serial number
	uint32_t serial_number = 0;

	if ((error = VnSensor_readSerialNumber(&_vs, &serial_number)) != E_NONE) {
		PX4_ERR("Error reading serial number %d", error);
		VnSensor_disconnect(&_vs);
		return false;
	}

	// query the sensor's firmware version
	char firmware_version[30] {};

	if ((error = VnSensor_readFirmwareVersion(&_vs, firmware_version, sizeof(firmware_version))) != E_NONE) {
		PX4_ERR("Error reading firmware version %d", error);
		VnSensor_disconnect(&_vs);
		return false;
	}

	PX4_INFO("Model: %s, HW REV: %" PRIu32 ", SN: %" PRIu32 ", SW VER: %s", model_number, hardware_revision, serial_number,
		 firmware_version);

	return true;
}

bool VectorNav::configure()
{
	// disable all ASCII messages
	VnSensor_writeAsyncDataOutputType(&_vs, VNOFF, true);

	VnError error = E_NONE;

	/* For the registers that have more complex configuration options, it is
	 * convenient to read the current existing register configuration, change
	 * only the values of interest, and then write the configuration to the
	 * register. This allows preserving the current settings for the register's
	 * other fields. Below, we change the heading mode used by the sensor. */
	VpeBasicControlRegister vpeReg{};

	if (VnSensor_readVpeBasicControl(&_vs, &vpeReg) == E_NONE) {

		char strConversions[30] {};
		strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
		PX4_DEBUG("Old Heading Mode: %s\n", strConversions);

		vpeReg.enable = VNVPEENABLE_ENABLE;
		vpeReg.headingMode = VNHEADINGMODE_ABSOLUTE;

		if (VnSensor_writeVpeBasicControl(&_vs, vpeReg, true) == E_NONE) {

			if (VnSensor_readVpeBasicControl(&_vs, &vpeReg) == E_NONE) {
				strFromHeadingMode(strConversions, (VnHeadingMode)vpeReg.headingMode);
				PX4_DEBUG("New Heading Mode: %s\n", strConversions);
			}

		} else {
			PX4_ERR("Error writing VPE basic control");
		}

	} else {
		PX4_ERR("Error reading VPE basic control %d", error);
	}

	VnError VnSensor_readImuFilteringConfiguration(VnSensor * sensor, ImuFilteringConfigurationRegister * fields);
	// VnError VnSensor_writeImuFilteringConfiguration(VnSensor *sensor, ImuFilteringConfigurationRegister fields, bool waitForReply);

	// grab ImuFilteringConfigurationRegister, zero all modes and write



	// TODO:
	// sensor to antenna

	// VnError VnSensor_readGpsAntennaOffset(VnSensor *s, vec3f *position)
	// VnSensor_writeGpsAntennaOffset

	// A -> B
	//   GpsCompassBaselineRegister
	//    position and uncertainty
	// VnSensor_readGpsCompassBaseline


	// binary output 1: max rate IMU
	BinaryOutputRegister_initialize(
		&_binary_output_group_1,
		ASYNCMODE_PORT2,
		1, // divider
		COMMONGROUP_NONE,
		TIMEGROUP_TIMESTARTUP,
		(ImuGroup)(IMUGROUP_ACCEL | IMUGROUP_ANGULARRATE),
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
		GPSGROUP_NONE
	);

	if ((error = VnSensor_writeBinaryOutput1(&_vs, &_binary_output_group_1, true)) != E_NONE) {

		// char buffer[128]{};
		// strFromVnError((char*)buffer, error);
		// PX4_ERR("Error writing binary output 1: %s", buffer);
		PX4_ERR("Error writing binary output 1 %d", error);
		return false;
	}

	// binary output 2: medium rate AHRS, INS, baro, mag
	BinaryOutputRegister_initialize(
		&_binary_output_group_2,
		ASYNCMODE_PORT2,
		8, // divider
		COMMONGROUP_NONE,
		TIMEGROUP_TIMESTARTUP,
		(ImuGroup)(IMUGROUP_TEMP | IMUGROUP_PRES | IMUGROUP_MAG),
		GPSGROUP_NONE,
		(AttitudeGroup)(ATTITUDEGROUP_QUATERNION | ATTITUDEGROUP_LINEARACCELNED),
		(InsGroup)(INSGROUP_INSSTATUS | INSGROUP_POSLLA | INSGROUP_VELNED | INSGROUP_POSU | INSGROUP_VELU),
		GPSGROUP_NONE
	);

	if ((error = VnSensor_writeBinaryOutput2(&_vs, &_binary_output_group_2, true)) != E_NONE) {
		PX4_ERR("Error writing binary output 2 %d", error);
		return false;
	}

	// binary output 3: low rate GNSS
	BinaryOutputRegister_initialize(
		&_binary_output_group_3,
		ASYNCMODE_PORT2,
		80, // divider
		COMMONGROUP_NONE,
		TIMEGROUP_TIMESTARTUP,
		IMUGROUP_NONE,
		(GpsGroup)(GPSGROUP_UTC | GPSGROUP_NUMSATS | GPSGROUP_FIX | GPSGROUP_POSLLA | GPSGROUP_VELNED
			   | GPSGROUP_POSU | GPSGROUP_VELU | GPSGROUP_DOP),
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE,
		GPSGROUP_NONE
	);

	if ((error = VnSensor_writeBinaryOutput3(&_vs, &_binary_output_group_3, true)) != E_NONE) {
		PX4_ERR("Error writing binary output 3 %d", error);
		//return false;
	}

	VnSensor_registerAsyncPacketReceivedHandler(&_vs, VectorNav::binaryAsyncMessageReceived, this);
	VnSensor_registerErrorPacketReceivedHandler(&_vs, VectorNav::binaryAsyncMessageReceived, this);

	_time_configured_us.store(hrt_absolute_time());

	return true;
}

void VectorNav::Run()
{
	if (should_exit()) {
		VnSensor_unregisterAsyncPacketReceivedHandler(&_vs);
		VnSensor_disconnect(&_vs);
		exit_and_cleanup();
		return;

	} else if (!_initialized) {

		if (!_connected) {
			if (init()) {
				_connected = true;
			}
		}

		if (_connected) {
			if (!_configured) {
				if (configure()) {
					_configured = true;
				}
			}
		}

		if (_connected && _configured) {
			_initialized = true;

		} else {
			ScheduleDelayed(1_s);
			return;
		}
	}

	if (_connected && _configured && _initialized) {

		// check for timeout
		const hrt_abstime time_configured_us = _time_configured_us.load();
		const hrt_abstime time_last_valid_imu_us = _time_last_valid_imu_us.load();

		if (_param_vn_mode.get() == 1) {
			if ((time_last_valid_imu_us != 0) && (hrt_elapsed_time(&time_last_valid_imu_us) < 3_s))

				// update sensor_selection if configured in INS mode
				if ((_px4_accel.get_device_id() != 0) && (_px4_gyro.get_device_id() != 0)) {
					sensor_selection_s sensor_selection{};
					sensor_selection.accel_device_id = _px4_accel.get_device_id();
					sensor_selection.gyro_device_id = _px4_gyro.get_device_id();
					sensor_selection.timestamp = hrt_absolute_time();
					_sensor_selection_pub.publish(sensor_selection);
				}
		}

		if ((time_configured_us != 0) && (hrt_elapsed_time(&time_last_valid_imu_us) > 5_s)
		    && (time_last_valid_imu_us != 0) && (hrt_elapsed_time(&time_last_valid_imu_us) > 1_s)
		   ) {
			PX4_ERR("timeout, reinitializing");
			VnSensor_unregisterAsyncPacketReceivedHandler(&_vs);
			VnSensor_disconnect(&_vs);
			_connected = false;
			_configured = false;
			_initialized = false;
		}
	}

	ScheduleDelayed(100_ms);
}

int VectorNav::print_status()
{
	printf("Using port '%s'\n", _port);

	// if (_device[0] != '\0') {
	// 	PX4_INFO("UART device: %s", _device);
	// 	PX4_INFO("UART RX bytes: %"  PRIu32, _bytes_rx);
	// }

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	return 0;
}

int VectorNav::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
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
		return -1;
	}

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		VectorNav *instance = new VectorNav(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleNow();

		return PX4_OK;

	} else {
		if (device_name) {
			PX4_ERR("invalid device (-d) %s", device_name);

		} else {
			PX4_INFO("valid device required");
		}
	}

	return PX4_ERROR;
}

int VectorNav::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int VectorNav::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial bus driver for the VectorNav VN-100, VN-200, VN-300.

Most boards are configured to enable/start the driver on a specified UART using the SENS_VN_CFG parameter.

Setup/usage information: https://docs.px4.io/main/en/sensor/vectornav.html

### Examples

Attempt to start driver on a specified serial device.
$ vectornav start -d /dev/ttyS1
Stop driver
$ vectornav stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vectornav", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("ins");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");

	return PX4_OK;
}

extern "C" __EXPORT int vectornav_main(int argc, char *argv[])
{
	return VectorNav::main(argc, argv);
}
