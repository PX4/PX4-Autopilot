/****************************************************************************
 *
 *   Copyright (c) 2012-2025 PX4 Development Team. All rights reserved.
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
 * @file sbgecom.cpp
 * Driver for the SBG Systems products
 *
 * @author SBG Systems <contact@sbg-systems.com>
 */

#include "sbgecom.hpp"

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/getopt.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>

#define DEFAULT_DEVNAME "/dev/ttyS0"

#define SBG_MODE_SENSOR 0
#define SBG_MODE_GNSS 1
#define SBG_MODE_INS 2

#define SBG_ESTIMATOR_ATTITUDE			(1 << 0)	///< 0 - attitude estimate is good
#define SBG_ESTIMATOR_VELOCITY_HORIZ		(1 << 1)	///< 1 - horizontal velocity estimate is good
#define SBG_ESTIMATOR_VELOCITY_VERT		(1 << 2)	///< 2 - vertical velocity estimate is good
#define SBG_ESTIMATOR_POS_HORIZ_REL		(1 << 3)	///< 3 - horizontal position (relative) estimate is good
#define SBG_ESTIMATOR_POS_HORIZ_ABS		(1 << 4)	///< 4 - horizontal position (absolute) estimate is good
#define SBG_ESTIMATOR_POS_VERT_ABS		(1 << 5)	///< 5 - vertical position (absolute) estimate is good
#define SBG_ESTIMATOR_POS_VERT_AGL		(1 << 6)	///< 6 - vertical position (above ground) estimate is good
#define SBG_ESTIMATOR_CONST_POS_MODE		(1 << 7)	///< 7 - EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
#define SBG_ESTIMATOR_PRED_POS_HORIZ_REL	(1 << 8)	///< 8 - EKF has sufficient data to enter a mode that will provide a (relative) position estimate
#define SBG_ESTIMATOR_PRED_POS_HORIZ_ABS	(1 << 9)	///< 9 - EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
#define SBG_ESTIMATOR_GPS_GLITCH		(1 << 10)	///< 10 - EKF has detected a GPS glitch
#define SBG_ESTIMATOR_ACCEL_ERROR		(1 << 11)	///< 11 - EKF has detected bad accelerometer data

#define DEFAULT_CONFIG_PATH "/etc/extras/sbg_settings.json"
#define OVERRIDE_CONFIG_PATH CONFIG_BOARD_ROOT_PATH DEFAULT_CONFIG_PATH
#define NEED_REBOOT_STR "\"needReboot\":true"

using matrix::Vector2f;

SbgEcom::SbgEcom(const char *device_name, uint32_t baudrate, const char *config_file, const char *config_string):
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(device_name)),
	_baudrate(baudrate),
	_config_file(config_file),
	_config_string(config_string)
{
	if (device_name) {
		strncpy(_device_name, device_name, sizeof(_device_name) - 1);
		_device_name[sizeof(_device_name) - 1] = '\0';
	}

	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_SBG;

	set_device_id(device_id.devid);
	_px4_accel.set_device_id(device_id.devid);
	_px4_gyro.set_device_id(device_id.devid);
	_px4_mag.set_device_id(device_id.devid);
}

SbgEcom::~SbgEcom()
{
	sbgEComClose(&_com_handle);
	sbgInterfaceDestroy(&_sbg_interface);

	perf_free(_accel_pub_interval_perf);
	perf_free(_gyro_pub_interval_perf);
	perf_free(_mag_pub_interval_perf);
	perf_free(_gnss_pub_interval_perf);

	perf_free(_attitude_pub_interval_perf);
	perf_free(_local_position_pub_interval_perf);
	perf_free(_global_position_pub_interval_perf);
}

void SbgEcom::set_device_id(uint32_t device_id)
{
	_device_id = device_id;
}

uint32_t SbgEcom::get_device_id()
{
	return _device_id;
}

SbgErrorCode SbgEcom::getAndPrintProductInfo(SbgEComHandle *handle)
{
	SbgErrorCode error_code;
	SbgEComDeviceInfo device_info;

	assert(handle);

	error_code = sbgEComCmdGetInfo(handle, &device_info);

	if (error_code == SBG_NO_ERROR) {
		char calib_version_str[32];
		char hw_revision_str[32];
		char fmw_version_str[32];

		sbgVersionToStringEncoded(device_info.calibationRev, calib_version_str, sizeof(calib_version_str));
		sbgVersionToStringEncoded(device_info.hardwareRev, hw_revision_str, sizeof(hw_revision_str));
		sbgVersionToStringEncoded(device_info.firmwareRev, fmw_version_str, sizeof(fmw_version_str));

		PX4_INFO("      Serial Number: %09" PRIu32, device_info.serialNumber);
		PX4_INFO("       Product Code: %s", device_info.productCode);
		PX4_INFO("  Hardware Revision: %s", hw_revision_str);
		PX4_INFO("   Firmware Version: %s", fmw_version_str);
		PX4_INFO("     Calib. Version: %s", calib_version_str);

	} else {
		SBG_LOG_WARNING(error_code, "Unable to retrieve device information");
	}

	return error_code;
}

void SbgEcom::printLogCallBack(const char *file_name, const char *function_name, uint32_t line, const char *category,
			       SbgDebugLogType log_type, SbgErrorCode error_code, const char *message)
{
	const char *base_name;

	assert(file_name);
	assert(function_name);
	assert(category);
	assert(message);

	base_name = strrchr(file_name, '/');

	if (!base_name) {
		base_name = file_name;

	} else {
		base_name++;
	}

	switch (log_type) {
	case SBG_DEBUG_LOG_TYPE_DEBUG:
		PX4_DEBUG("%s:%" PRIu32 ": %s: %s", base_name, line, function_name, message);
		break;

	case SBG_DEBUG_LOG_TYPE_INFO:
		PX4_INFO("%s:%" PRIu32 ": %s: %s", base_name, line, function_name, message);
		break;

	case SBG_DEBUG_LOG_TYPE_WARNING:
		PX4_WARN("%s:%" PRIu32 ": %s: err:%s: %s", base_name, line, function_name, sbgErrorCodeToString(error_code), message);
		break;

	case SBG_DEBUG_LOG_TYPE_ERROR:
		PX4_ERR("%s:%" PRIu32 ": %s: err:%s: %s", base_name, line, function_name, sbgErrorCodeToString(error_code), message);
		break;
	}
}

void SbgEcom::handleLogImuShort(const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
	assert(ref_sbg_data);
	assert(user_arg);

	SbgEcom *instance = static_cast<SbgEcom *>(user_arg);

	const float temperature = sbgEComLogImuShortGetTemperature(&ref_sbg_data->imuShort);

	// publish sensor_accel
	instance->_px4_accel.update(hrt_absolute_time(),
				    sbgEComLogImuShortGetDeltaVelocity(&ref_sbg_data->imuShort, 0),
				    sbgEComLogImuShortGetDeltaVelocity(&ref_sbg_data->imuShort, 1),
				    sbgEComLogImuShortGetDeltaVelocity(&ref_sbg_data->imuShort, 2));
	instance->_px4_accel.set_error_count(perf_event_count(instance->_comms_errors));
	instance->_px4_accel.set_temperature(temperature);
	perf_count(instance->_accel_pub_interval_perf);

	// publish sensor_gyro
	instance->_px4_gyro.update(hrt_absolute_time(),
				   sbgEComLogImuShortGetDeltaAngle(&ref_sbg_data->imuShort, 0),
				   sbgEComLogImuShortGetDeltaAngle(&ref_sbg_data->imuShort, 1),
				   sbgEComLogImuShortGetDeltaAngle(&ref_sbg_data->imuShort, 2));
	instance->_px4_gyro.set_error_count(perf_event_count(instance->_comms_errors));
	instance->_px4_gyro.set_temperature(temperature);
	perf_count(instance->_gyro_pub_interval_perf);
}

void SbgEcom::handleLogMag(const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
	assert(ref_sbg_data);
	assert(user_arg);

	SbgEcom *instance = static_cast<SbgEcom *>(user_arg);

	// publish sensor_mag
	instance->_px4_mag.update(ref_sbg_data->magData.timeStamp,
				  (ref_sbg_data->magData.magnetometers[0]),
				  (ref_sbg_data->magData.magnetometers[1]),
				  (ref_sbg_data->magData.magnetometers[2]));
	instance->_px4_mag.set_error_count(perf_event_count(instance->_comms_errors));
	perf_count(instance->_mag_pub_interval_perf);
}

void SbgEcom::updateEstimatorStatus(uint32_t ekf_status, estimator_status_s *estimator_status)
{
	SbgEComSolutionMode ekf_nav_status = sbgEComLogEkfGetSolutionMode(ekf_status);

	estimator_status->solution_status_flags |= ((ekf_status & SBG_ECOM_SOL_ATTITUDE_VALID)
			&& (ekf_status & SBG_ECOM_SOL_HEADING_VALID)) ? SBG_ESTIMATOR_ATTITUDE : 0;
	estimator_status->solution_status_flags |= (ekf_status & SBG_ECOM_SOL_VELOCITY_VALID) ? SBG_ESTIMATOR_VELOCITY_HORIZ :
			0;
	estimator_status->solution_status_flags |= (ekf_status & SBG_ECOM_SOL_VELOCITY_VALID) ? SBG_ESTIMATOR_VELOCITY_VERT : 0;
	estimator_status->solution_status_flags |= (ekf_status & SBG_ECOM_SOL_POSITION_VALID) ? SBG_ESTIMATOR_POS_HORIZ_REL : 0;
	estimator_status->solution_status_flags |= (ekf_status & SBG_ECOM_SOL_POSITION_VALID) ? SBG_ESTIMATOR_POS_HORIZ_ABS : 0;
	estimator_status->solution_status_flags |= (ekf_status & SBG_ECOM_SOL_POSITION_VALID) ? SBG_ESTIMATOR_POS_VERT_ABS : 0;
	estimator_status->solution_status_flags |= (ekf_status & SBG_ECOM_SOL_POSITION_VALID) ? SBG_ESTIMATOR_POS_VERT_AGL : 0;
	estimator_status->solution_status_flags |= (ekf_status & SBG_ECOM_SOL_ZUPT_USED) ? SBG_ESTIMATOR_CONST_POS_MODE : 0;

	estimator_status->solution_status_flags |= (ekf_nav_status == SBG_ECOM_SOL_MODE_NAV_POSITION) ?
			SBG_ESTIMATOR_PRED_POS_HORIZ_REL : 0;
	estimator_status->solution_status_flags |= (ekf_nav_status == SBG_ECOM_SOL_MODE_NAV_POSITION) ?
			SBG_ESTIMATOR_PRED_POS_HORIZ_ABS : 0;
}

void SbgEcom::handleLogEkfQuat(const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
	const hrt_abstime time_now_us = hrt_absolute_time();

	assert(ref_sbg_data);
	assert(user_arg);

	SbgEcom *instance = static_cast<SbgEcom *>(user_arg);

	// publish estimator_status
	estimator_status_s estimator_status{};
	estimator_status.timestamp = time_now_us;
	estimator_status.timestamp_sample = ref_sbg_data->ekfQuatData.timeStamp;
	estimator_status.accel_device_id = instance->get_device_id();
	estimator_status.gyro_device_id = instance->get_device_id();
	estimator_status.mag_device_id = instance->get_device_id();

	instance->updateEstimatorStatus(ref_sbg_data->ekfQuatData.status, &estimator_status);

	instance->_estimator_status_pub.publish(estimator_status);

	// publish attitude
	vehicle_attitude_s attitude{};

	attitude.timestamp = time_now_us;
	attitude.timestamp_sample = ref_sbg_data->ekfQuatData.timeStamp;

	attitude.q[0] = ref_sbg_data->ekfQuatData.quaternion[0];
	attitude.q[1] = ref_sbg_data->ekfQuatData.quaternion[1];
	attitude.q[2] = ref_sbg_data->ekfQuatData.quaternion[2];
	attitude.q[3] = ref_sbg_data->ekfQuatData.quaternion[3];

	instance->_attitude_pub.publish(attitude);
	perf_count(instance->_attitude_pub_interval_perf);

	matrix::Quatf q{attitude.q};
	instance->_heading = matrix::Eulerf{q}.psi();
}

void SbgEcom::handleLogEkfNav(const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
	const hrt_abstime time_now_us = hrt_absolute_time();

	assert(ref_sbg_data);
	assert(user_arg);

	SbgEcom *instance = static_cast<SbgEcom *>(user_arg);

	// publish estimator_status
	estimator_status_s estimator_status{};
	estimator_status.timestamp = time_now_us;
	estimator_status.timestamp_sample = ref_sbg_data->ekfNavData.timeStamp;

	instance->updateEstimatorStatus(ref_sbg_data->ekfNavData.status, &estimator_status);

	instance->_estimator_status_pub.publish(estimator_status);

	SbgEComSolutionMode ekf_nav_status = sbgEComLogEkfGetSolutionMode(ref_sbg_data->ekfNavData.status);

	// don't publish local and global positions if not reliable
	if (ekf_nav_status != SBG_ECOM_SOL_MODE_NAV_POSITION) {
		return;
	}

	const double latitude = ref_sbg_data->ekfNavData.position[0];
	const double longitude = ref_sbg_data->ekfNavData.position[1];
	const double altitude = ref_sbg_data->ekfNavData.position[2];

	const double north_velocity = ref_sbg_data->ekfNavData.velocity[0];
	const double east_velocity = ref_sbg_data->ekfNavData.velocity[1];
	const double down_velocity = ref_sbg_data->ekfNavData.velocity[2];

	if (!instance->_pos_ref.isInitialized()) {
		instance->_pos_ref.initReference(latitude, longitude, time_now_us);
		instance->_gps_alt_ref = altitude;
	}

	const Vector2f pos_ned = instance->_pos_ref.project(latitude, longitude);

	// publish local_position
	vehicle_local_position_s local_position{};

	local_position.timestamp = time_now_us;
	local_position.timestamp_sample = ref_sbg_data->ekfNavData.timeStamp;

	local_position.xy_valid = math::isFinite(latitude) && math::isFinite(longitude);
	local_position.z_valid = math::isFinite(altitude);
	local_position.v_xy_valid = math::isFinite(north_velocity) && math::isFinite(east_velocity);
	local_position.v_z_valid = math::isFinite(down_velocity);

	local_position.x = pos_ned(0);
	local_position.y = pos_ned(1);
	local_position.z = -(altitude - instance->_gps_alt_ref);

	local_position.vx = north_velocity;
	local_position.vy = east_velocity;
	local_position.vz = down_velocity;

	local_position.heading = instance->_heading;
	local_position.heading_good_for_control = true;;

	if (instance->_pos_ref.isInitialized()) {
		local_position.xy_global = true;
		local_position.z_global = true;
		local_position.ref_timestamp = instance->_pos_ref.getProjectionReferenceTimestamp();
		local_position.ref_lat = instance->_pos_ref.getProjectionReferenceLat();
		local_position.ref_lon = instance->_pos_ref.getProjectionReferenceLon();
		local_position.ref_alt = instance->_gps_alt_ref;
	}

	local_position.dist_bottom_valid = false;

	local_position.eph = sqrt(pow(ref_sbg_data->ekfNavData.positionStdDev[0], 2) +
				  pow(ref_sbg_data->ekfNavData.positionStdDev[1], 2));
	local_position.epv = ref_sbg_data->ekfNavData.positionStdDev[2];
	local_position.evh = sqrt(pow(ref_sbg_data->ekfNavData.velocityStdDev[0], 2) +
				  pow(ref_sbg_data->ekfNavData.velocityStdDev[1], 2));
	local_position.evv = ref_sbg_data->ekfNavData.velocityStdDev[2];


	local_position.dead_reckoning = false;

	local_position.vxy_max = INFINITY;
	local_position.vz_max = INFINITY;
	local_position.hagl_min = INFINITY;
	local_position.hagl_max_xy = INFINITY;
	local_position.hagl_max_z = INFINITY;

	instance->_local_position_pub.publish(local_position);
	perf_count(instance->_local_position_pub_interval_perf);

	// publish global_position
	vehicle_global_position_s global_position{};

	global_position.timestamp = time_now_us;
	global_position.timestamp_sample = ref_sbg_data->ekfNavData.timeStamp;

	global_position.lat = latitude;
	global_position.lon = longitude;
	global_position.alt = altitude;
	global_position.alt_ellipsoid = ref_sbg_data->ekfNavData.undulation;

	global_position.lat_lon_valid = math::isFinite(latitude) && math::isFinite(longitude);
	global_position.alt_valid = math::isFinite(altitude);

	global_position.eph = sqrt(pow(ref_sbg_data->ekfNavData.positionStdDev[0], 2) +
				   pow(ref_sbg_data->ekfNavData.positionStdDev[1], 2));
	global_position.epv = ref_sbg_data->ekfNavData.positionStdDev[2];

	global_position.dead_reckoning = false;

	instance->_global_position_pub.publish(global_position);
	perf_count(instance->_global_position_pub_interval_perf);
}

void SbgEcom::handleLogGnssPosVelHdt(SbgEComMsgId msg, const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
	const hrt_abstime time_now_us = hrt_absolute_time();
	uint8_t type;
	uint8_t state;
	uint8_t spoofing;

	assert(msg);
	assert(ref_sbg_data);
	assert(user_arg);

	SbgEcom *instance = static_cast<SbgEcom *>(user_arg);
	GnssData *gnss_data = &instance->gnss_data;

	// Store the data based on its type
	switch (msg) {
	case SBG_ECOM_LOG_GPS1_POS:
		gnss_data->gps_pos = ref_sbg_data->gpsPosData;
		gnss_data->pos_received = true;
		gnss_data->pos_timestamp = time_now_us;
		break;

	case SBG_ECOM_LOG_GPS1_VEL:
		gnss_data->gps_vel = ref_sbg_data->gpsVelData;
		gnss_data->vel_received = true;
		gnss_data->vel_timestamp = time_now_us;
		break;

	case SBG_ECOM_LOG_GPS1_HDT:
		gnss_data->gps_hdt = ref_sbg_data->gpsHdtData;
		gnss_data->hdt_received = true;
		gnss_data->hdt_timestamp = time_now_us;
		break;
	}

	if (gnss_data->pos_received && gnss_data->vel_received && gnss_data->hdt_received) {
		// publish sensor_gps
		sensor_gps_s sensor_gps{};

		sensor_gps.timestamp = time_now_us;
		sensor_gps.timestamp_sample = gnss_data->gps_pos.timeStamp;

		sensor_gps.device_id = instance->get_device_id();

		sensor_gps.latitude_deg = gnss_data->gps_pos.latitude;
		sensor_gps.longitude_deg = gnss_data->gps_pos.longitude;
		sensor_gps.altitude_msl_m = gnss_data->gps_pos.altitude;
		sensor_gps.altitude_ellipsoid_m = gnss_data->gps_pos.undulation;

		sensor_gps.s_variance_m_s = sqrt(pow(gnss_data->gps_vel.velocityAcc[0], 2) +
						 pow(gnss_data->gps_vel.velocityAcc[1], 2) +
						 pow(gnss_data->gps_vel.velocityAcc[2], 2));
		sensor_gps.c_variance_rad = math::radians(gnss_data->gps_vel.courseAcc);

		type = sbgEComLogGnssPosGetType(&gnss_data->gps_pos);

		switch (type) {
		case SBG_ECOM_GNSS_POS_TYPE_NO_SOLUTION:
			sensor_gps.fix_type = 0;
			break;

		case SBG_ECOM_GNSS_POS_TYPE_PSRDIFF:
		case SBG_ECOM_GNSS_POS_TYPE_SBAS:
			sensor_gps.fix_type = 4;
			break;

		case SBG_ECOM_GNSS_POS_TYPE_RTK_FLOAT:
			sensor_gps.fix_type = 5;
			break;

		case SBG_ECOM_GNSS_POS_TYPE_RTK_INT:
			sensor_gps.fix_type = 6;
			break;

		case SBG_ECOM_GNSS_POS_TYPE_FIXED:
			sensor_gps.fix_type = 7;
			break;

		case SBG_ECOM_GNSS_POS_TYPE_PPP_FLOAT:
		case SBG_ECOM_GNSS_POS_TYPE_PPP_INT:
			sensor_gps.fix_type = 8;
			break;

		default:
			sensor_gps.fix_type = 3;
			break;
		}

		sensor_gps.eph = sqrt(pow(gnss_data->gps_pos.longitudeAccuracy, 2) +
				      pow(gnss_data->gps_pos.latitudeAccuracy, 2));
		sensor_gps.epv = gnss_data->gps_pos.altitudeAccuracy;

		state = sbgEComLogGnssPosGetIfmStatus(&gnss_data->gps_pos);

		switch (state) {
		case SBG_ECOM_GNSS_IFM_STATUS_UNKNOWN:
			sensor_gps.jamming_state = 0;
			break;

		case SBG_ECOM_GNSS_IFM_STATUS_CLEAN:
			sensor_gps.jamming_state = 1;
			break;

		case SBG_ECOM_GNSS_IFM_STATUS_MITIGATED:
			sensor_gps.jamming_state = 2;
			break;

		case SBG_ECOM_GNSS_IFM_STATUS_CRITICAL:
			sensor_gps.jamming_state = 3;
			break;
		}

		spoofing = sbgEComLogGnssPosGetSpoofingStatus(&gnss_data->gps_pos);

		switch (spoofing) {
		case SBG_ECOM_GNSS_SPOOFING_STATUS_UNKNOWN:
			sensor_gps.spoofing_state = 0;
			break;

		case SBG_ECOM_GNSS_SPOOFING_STATUS_CLEAN:
			sensor_gps.spoofing_state = 1;
			break;

		case SBG_ECOM_GNSS_SPOOFING_STATUS_SINGLE:
			sensor_gps.spoofing_state = 2;
			break;

		case SBG_ECOM_GNSS_SPOOFING_STATUS_MULTIPLE:
			sensor_gps.spoofing_state = 3;
			break;
		}

		sensor_gps.vel_m_s = sqrt(pow(gnss_data->gps_vel.velocity[0], 2) +
					  pow(gnss_data->gps_vel.velocity[1], 2) +
					  pow(gnss_data->gps_vel.velocity[2], 2));
		sensor_gps.vel_n_m_s = gnss_data->gps_vel.velocity[0];
		sensor_gps.vel_e_m_s = gnss_data->gps_vel.velocity[1];
		sensor_gps.vel_d_m_s = gnss_data->gps_vel.velocity[2];
		sensor_gps.vel_ned_valid = true;

		sensor_gps.cog_rad = math::radians(gnss_data->gps_vel.course);

		sensor_gps.timestamp_time_relative = sensor_gps.timestamp_sample - time_now_us;
		sensor_gps.time_utc_usec = 0;

		sensor_gps.satellites_used = gnss_data->gps_pos.numSvUsed;

		sensor_gps.heading = math::radians(gnss_data->gps_hdt.heading);
		sensor_gps.heading_offset = math::radians(gnss_data->gps_hdt.pitch);
		sensor_gps.heading_accuracy = math::radians(gnss_data->gps_hdt.headingAccuracy);

		// Check timestamp synchronization
		const hrt_abstime max_time_diff = 1000000; // Maximum allowed time difference in microseconds (e.g., 1 second)
		hrt_abstime pos_time = gnss_data->pos_timestamp;
		hrt_abstime vel_time = gnss_data->vel_timestamp;
		hrt_abstime hdt_time = gnss_data->hdt_timestamp;

		if (((time_now_us - pos_time) < max_time_diff) &&
		    ((time_now_us - vel_time) < max_time_diff) &&
		    ((time_now_us - hdt_time) < max_time_diff) &&
		    ((pos_time - vel_time) < max_time_diff) &&
		    ((pos_time - hdt_time) < max_time_diff) &&
		    ((vel_time - hdt_time) < max_time_diff)) {
			instance->_sensor_gps_pub.publish(sensor_gps);
			perf_count(instance->_gnss_pub_interval_perf);
		}

		// Reset the flags and timestamps
		gnss_data->pos_received = false;
		gnss_data->vel_received = false;
		gnss_data->hdt_received = false;

		gnss_data->pos_timestamp = 0;
		gnss_data->vel_timestamp = 0;
		gnss_data->hdt_timestamp = 0;
	}
}

SbgErrorCode SbgEcom::onLogReceived(SbgEComHandle *handle, SbgEComClass msg_class, SbgEComMsgId msg,
				    const SbgEComLogUnion *ref_sbg_data, void *user_arg)
{
	SBG_UNUSED_PARAMETER(handle);

	assert(msg_class);
	assert(msg);
	assert(ref_sbg_data);
	assert(user_arg);

	SbgEcom *instance = static_cast<SbgEcom *>(user_arg);

	if (msg_class == SBG_ECOM_CLASS_LOG_ECOM_0) {
		int32_t mode;
		int32_t ekf2_en;
		param_get(param_find("SBG_MODE"), &mode);
		param_get(param_find("EKF2_EN"), &ekf2_en);

		bool ekf_failure = (ekf2_en && mode == SBG_MODE_INS);

		if (!instance->_ekf_failure && ekf_failure) {
			PX4_WARN("Both SBG EKF and EKF2 are configured, this can lead to an unexpected behaviour");
			instance->_ekf_failure = true;

		} else if (instance->_ekf_failure && !ekf_failure) {
			PX4_INFO("EKF management is back to good");
			instance->_ekf_failure = false;
		}

		switch (msg) {
		case SBG_ECOM_LOG_IMU_SHORT:
			instance->handleLogImuShort(ref_sbg_data, user_arg);
			break;

		case SBG_ECOM_LOG_MAG:
			instance->handleLogMag(ref_sbg_data, user_arg);
			break;

		case SBG_ECOM_LOG_GPS1_POS:
		case SBG_ECOM_LOG_GPS1_VEL:
		case SBG_ECOM_LOG_GPS1_HDT:
			if (mode == SBG_MODE_GNSS || mode == SBG_MODE_INS) {
				instance->handleLogGnssPosVelHdt(msg, ref_sbg_data, user_arg);
			}

			break;

		case SBG_ECOM_LOG_EKF_QUAT:
			if (mode == SBG_MODE_INS) {
				instance->handleLogEkfQuat(ref_sbg_data, user_arg);
			}

			break;

		case SBG_ECOM_LOG_EKF_NAV:
			if (mode == SBG_MODE_INS) {
				instance->handleLogEkfNav(ref_sbg_data, user_arg);
			}

			break;

		default:
			PX4_DEBUG("Received unknown SBG message (class %u, id %u)", msg_class, msg);
			break;
		}

	} else {
		PX4_INFO("Received message from unsupported SBGEcom class %u", msg_class);
	}

	return SBG_NO_ERROR;
}

SbgErrorCode SbgEcom::handleOneLog(SbgEComHandle *handle)
{
	SbgErrorCode error_code;
	SbgEComProtocolPayload payload;
	uint8_t received_msg;
	uint8_t received_msg_class;

	assert(handle);

	sbgEComProtocolPayloadConstruct(&payload);

	perf_begin(_sample_perf);

	error_code = sbgEComProtocolReceive2(&handle->protocolHandle, &received_msg_class, &received_msg, &payload);

	if (error_code == SBG_NO_ERROR) {
		if (sbgEComMsgClassIsALog((SbgEComClass)received_msg_class)) {
			error_code = sbgEComLogParse((SbgEComClass)received_msg_class, (SbgEComMsgId)received_msg,
						     sbgEComProtocolPayloadGetBuffer(&payload), sbgEComProtocolPayloadGetSize(&payload), &_log_data);

			if (error_code == SBG_NO_ERROR) {
				if (handle->pReceiveLogCallback) {
					error_code = handle->pReceiveLogCallback(handle, (SbgEComClass)received_msg_class, (SbgEComMsgId)received_msg,
							&_log_data, handle->pUserArg);
				}

				sbgEComLogCleanup(&_log_data, (SbgEComClass)received_msg_class, (SbgEComMsgId)received_msg);

				perf_end(_sample_perf);

			} else {
				perf_count(_comms_errors);
			}

		} else {
			PX4_ERR("command received %d", error_code);
		}

	} else if (error_code != SBG_NOT_READY) {
		PX4_WARN("Invalid frame received %d", error_code);
		perf_count(_comms_errors);
	}

	sbgEComProtocolPayloadDestroy(&payload);

	return error_code;
}

SbgErrorCode SbgEcom::sendAirDataLog(SbgEComHandle *handle, SbgEcom *instance)
{
	SbgErrorCode error_code = SBG_NO_ERROR;
	SbgEComLogAirData air_data_log;
	uint8_t output_buffer[64];
	SbgStreamBuffer output_stream;

	assert(handle);
	assert(instance);

	vehicle_air_data_s air_data{};

	if (instance->_air_data_sub.update(&air_data)) {
		memset(&air_data_log, 0x00, sizeof(air_data_log));

		air_data_log.timeStamp = hrt_absolute_time() - air_data.timestamp;
		air_data_log.status |= SBG_ECOM_AIR_DATA_TIME_IS_DELAY;

		air_data_log.pressureAbs = air_data.baro_pressure_pa;
		air_data_log.status |= SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID;

		air_data_log.altitude = air_data.baro_alt_meter;
		air_data_log.status |= SBG_ECOM_AIR_DATA_ALTITUDE_VALID;

		air_data_log.airTemperature = air_data.ambient_temperature;
		air_data_log.status |= SBG_ECOM_AIR_DATA_TEMPERATURE_VALID;

		differential_pressure_s differential_pressure{};

		if (instance->_diff_pressure_sub.update(&differential_pressure)) {
			air_data_log.pressureDiff = differential_pressure.differential_pressure_pa;
			air_data_log.status |= SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID;
		}

		airspeed_s airspeed{};

		if (instance->_airspeed_sub.update(&airspeed)) {
			air_data_log.trueAirspeed = airspeed.true_airspeed_m_s;
			air_data_log.status |= SBG_ECOM_AIR_DATA_AIRPSEED_VALID;
		}

		sbgStreamBufferInitForWrite(&output_stream, output_buffer, sizeof(output_buffer));

		perf_begin(_write_perf);

		error_code = sbgEComLogAirDataWriteToStream(&air_data_log, &output_stream);

		if (error_code == SBG_NO_ERROR) {
			error_code = sbgEComProtocolSend(&handle->protocolHandle, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_AIR_DATA,
							 sbgStreamBufferGetLinkedBuffer(&output_stream), sbgStreamBufferGetLength(&output_stream));

			if (error_code != SBG_NO_ERROR) {
				PX4_ERR("Unable to send the AirData log %d", error_code);

			} else {
				perf_end(_write_perf);
			}

		} else {
			PX4_ERR("Unable to write the AirData payload. %d", error_code);
		}
	}

	return error_code;
}

SbgErrorCode SbgEcom::sendMagLog(SbgEComHandle *handle, SbgEcom *instance)
{
	SbgErrorCode error_code = SBG_NO_ERROR;
	SbgEComLogMag mag_log;
	uint8_t output_buffer[64];
	SbgStreamBuffer output_stream;

	assert(handle);
	assert(instance);

	vehicle_magnetometer_s mag{};

	if (instance->_mag_sub.update(&mag)) {
		memset(&mag_log, 0x00, sizeof(mag_log));

		mag_log.timeStamp = mag.timestamp_sample;
		// mag_log.status = 0; // STO: don't know how to set it

		mag_log.magnetometers[0] = mag.magnetometer_ga[0];
		mag_log.magnetometers[1] = mag.magnetometer_ga[1];
		mag_log.magnetometers[2] = mag.magnetometer_ga[2];

		sbgStreamBufferInitForWrite(&output_stream, output_buffer, sizeof(output_buffer));

		perf_begin(_write_perf);

		error_code = sbgEComLogMagWriteToStream(&mag_log, &output_stream);

		if (error_code == SBG_NO_ERROR) {
			error_code = sbgEComProtocolSend(&handle->protocolHandle, SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_MAG,
							 sbgStreamBufferGetLinkedBuffer(&output_stream), sbgStreamBufferGetLength(&output_stream));

			if (error_code != SBG_NO_ERROR) {
				PX4_ERR("Unable to send the Mag log %d", error_code);

			} else {
				perf_end(_write_perf);
			}

		} else {
			PX4_ERR("Unable to write the Mag payload. %d", error_code);
		}
	}

	return error_code;
}

void SbgEcom::send_config(SbgEComHandle *pHandle, const char *config)
{
	SbgEComCmdApiReply reply;

	assert(pHandle);
	assert(config);

	sbgEComCmdApiReplyConstruct(&reply);

	sbgEComCmdApiPost(pHandle, "/api/v1/settings", NULL, config, &reply);

	if (!sbgEComCmdApiReplySuccessful(&reply)) {
		PX4_ERR("Fail to apply SBG configuration: %s", reply.pContent);

	} else {
		bool need_reboot = (strstr(reply.pContent, NEED_REBOOT_STR) != NULL);
		sbgEComCmdApiPost(pHandle, "/api/v1/settings/save", NULL, NULL, &reply);

		if (need_reboot) {
			PX4_INFO("Reboot SBG device");
			sbgEComCmdApiPost(pHandle, "/api/v1/system/reboot", NULL, NULL, &reply);
		}
	}

	sbgEComCmdApiReplyDestroy(&reply);
}

void SbgEcom::send_config_file(SbgEComHandle *pHandle, const char *file_path)
{
	int fd;
	char *body = NULL;
	struct stat s;

	assert(pHandle);
	assert(file_path);

	fd = open(file_path, O_RDONLY);

	if (fd == -1) {
		PX4_ERR("Failed to open config");
		return;
	}

	fstat(fd, &s);
	body = (char *)malloc(s.st_size + 1);

	if (!body) {
		PX4_ERR("Failed to allocate memory (%ld) - %s", s.st_size + 1, strerror(get_errno()));
		close(fd);
		return;
	}

	read(fd, body, s.st_size);
	body[s.st_size] = '\0';

	send_config(pHandle, body);

	free(body);

	if (close(fd) == -1) {
		perror("Error closing the file");
		return;
	}
}

int SbgEcom::init()
{
	SbgErrorCode error_code;
	struct termios options;
	int *pSerialHandle;

	error_code = sbgInterfaceSerialCreate(&_sbg_interface, _device_name, _baudrate);

	if (error_code == SBG_NO_ERROR) {
		PX4_INFO("Serial interface created successfully on port: %s, baudrate: %ld", _device_name, _baudrate);
	}

	pSerialHandle = (int *)_sbg_interface.handle;

	if (tcgetattr((*pSerialHandle), &options) != -1) {
		// add custom options
		options.c_cflag &= CSIZE;
		options.c_iflag &= ~(IXON | IXOFF | IXANY);
		options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | ICRNL | INPCK);
		options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | ECHONL | IEXTEN);

		if (tcsetattr((*pSerialHandle), TCSANOW, &options) != -1) {
			error_code = sbgInterfaceFlush(&_sbg_interface, SBG_IF_FLUSH_ALL);

		} else {
			error_code = SBG_ERROR;
		}

	} else {
		error_code = SBG_ERROR;
	}

	error_code = sbgEComInit(&_com_handle, &_sbg_interface);
	// Increase sbgECom timeout for the initialization
	sbgEComSetCmdTrialsAndTimeOut(&_com_handle, 3, 5000);

	if (error_code == SBG_NO_ERROR) {
		int32_t ins_config_enable;
		param_get(param_find("SBG_CONFIGURE_EN"), &ins_config_enable);

		getAndPrintProductInfo(&_com_handle);

		if (ins_config_enable) {
			if (_config_string != nullptr) {
				PX4_INFO("Apply config string instead of config file");
				send_config(&_com_handle, _config_string);

			} else {
				send_config_file(&_com_handle, _config_file);
			}
		}

		// Reset sbgECom timeout to its defaut value
		sbgEComSetCmdTrialsAndTimeOut(&_com_handle, 3, SBG_ECOM_DEFAULT_CMD_TIME_OUT);
		sbgEComSetReceiveLogCallback(&_com_handle, onLogReceived, this);
		return PX4_OK;

	} else {
		PX4_ERR("sbgECom initialization failed (%d)", error_code);
		return PX4_ERROR;
	}
}

void SbgEcom::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (!_initialized) {
		init_result = init();
		_initialized = true;
	}

	if (init_result == PX4_OK) {
		SbgErrorCode error_code;

		error_code = handleOneLog(&_com_handle);

		if (error_code == SBG_NO_ERROR) {
			ScheduleDelayed(time_literals::operator ""_ms(0));

			if (failure) {
				assert(iteration_count >= 0);
				iteration_count--;
				failure = false;
			}

		} else if (error_code != SBG_NOT_READY) {
			PX4_ERR("Unable to process incoming sbgECom logs %d", error_code);
		}

		if (error_code != SBG_NO_ERROR) {
			ScheduleDelayed(time_literals::operator ""_ms(1));
			failure = true;
		}

		error_code = sendAirDataLog(&_com_handle, this);

		if (error_code != SBG_NO_ERROR) {
			PX4_WARN("Unable to send AirData log %d", error_code);
		}

		error_code = sendMagLog(&_com_handle, this);

		if (error_code != SBG_NO_ERROR) {
			PX4_WARN("Unable to send Mag log %d", error_code);
		}
	}
}

int SbgEcom::task_spawn(int argc, char **argv)
{
	sbgCommonLibSetLogCallback(printLogCallBack);

	bool error_flag = false;

	const char *myoptarg = nullptr;
	int myoptind = 1;
	int ch;

	int32_t baudrate;
	param_get(param_find("SBG_BAUDRATE"), &baudrate);
	const char *dev_name = DEFAULT_DEVNAME;
	const char *config_file = DEFAULT_CONFIG_PATH;

	/* INS settings can be overwritten from the SD card */
	if (access(OVERRIDE_CONFIG_PATH, F_OK) == 0) {
		config_file = OVERRIDE_CONFIG_PATH;

	} else {
		config_file = DEFAULT_CONFIG_PATH;
	}

	const char *config_string = nullptr;

	while ((ch = px4_getopt(argc, argv, "b:d:f:s:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			baudrate = atoi(myoptarg);
			break;

		case 'd':
			dev_name = myoptarg;
			break;

		case 'f':
			config_file = myoptarg;
			break;

		case 's':
			config_string = myoptarg;
			break;

		case '?':
			PX4_WARN("unrecognized flag ?");
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

	if (dev_name && (access(dev_name, R_OK | W_OK) == 0)) {
		SbgEcom *instance = new SbgEcom(dev_name, baudrate, config_file, config_string);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_task_id = task_id_is_work_queue;
		_object.store(instance);
		instance->ScheduleNow();
		return PX4_OK;

	} else {
		if (dev_name) {
			PX4_ERR("invalid device (-d) %s", dev_name);

		} else {
			PX4_ERR("valid device required");
		}
	}

	return PX4_ERROR;
}

int SbgEcom::custom_command(int argc, char **argv)
{
	return print_usage("unrecognized command");
}

int SbgEcom::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION("Description du module");

	PRINT_MODULE_USAGE_NAME("sbgecom", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("ins");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', DEFAULT_DEVNAME, nullptr, "Serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 921600, 9600, 921600, "Baudrate device", true);
	PRINT_MODULE_USAGE_PARAM_STRING('f', DEFAULT_CONFIG_PATH, nullptr, "Config JSON file path", true);
	PRINT_MODULE_USAGE_PARAM_STRING('s', nullptr, nullptr, "Config JSON string", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");

	return PX4_OK;
}

int SbgEcom::print_status()
{
	printf("Using port '%s'\n", _device_name);

	perf_print_counter(_sample_perf);
	perf_print_counter(_write_perf);
	perf_print_counter(_comms_errors);

	return 0;
}

extern "C" __EXPORT int sbgecom_main(int argc, char **argv)
{
	return SbgEcom::main(argc, argv);
}
