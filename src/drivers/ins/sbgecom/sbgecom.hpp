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
 * @file sbgecom.hpp
 * Driver for the SBG Systems products
 *
 * @author SBG Systems <contact@sbg-systems.com>
 */

#pragma once

#include <sbgEComLib.h>
#include <version/sbgVersion.h>

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_magnetometer.h>

class SbgEcom : public ModuleBase<SbgEcom>, public ModuleParams, public px4::ScheduledWorkItem
{
public:

	SbgEcom(const char *port, uint32_t baudrate, const char *config_file, const char *config_string);
	~SbgEcom() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char **argv);

	/** @see ModuleBase */
	static int custom_command(int argc, char **argv);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	/** @see ModuleBase::run() */
	void Run() override;

	int init();

private:

	/**
	* @brief Type for logging functions.
	*
	* @param file_name File name where the error occurred.
	* @param function_name Function name where the error occurred.
	* @param line Line number where the error occurred.
	* @param category Category for this log or "None" if no category has been specified.
	* @param log_type Define if we have an error, a warning, an info or a debug log.
	* @param error_code The error code associated with the message.
	* @param message The message to log.
	*/
	static void printLogCallBack(const char *file_name, const char *function_name, uint32_t line, const char *category,
				     SbgDebugLogType log_type, SbgErrorCode error_code, const char *message);

	/**
	* @brief Parse IMU (Inertial Measurement Unit) measurement logs.
	*
	* @param ref_sbg_data Contains the received log data as an union.
	* @param user_arg Optional user supplied argument.
	*/
	static void handleLogImuShort(const SbgEComLogUnion *ref_sbg_data, void *user_arg);

	/**
	* @brief Parse magnetic field measurements logs.
	*
	* @param ref_sbg_data Contains the received log data as an union.
	* @param user_arg Optional user supplied argument.
	*/
	static void handleLogMag(const SbgEComLogUnion *ref_sbg_data, void *user_arg);

	/**
	* @brief Parse EKF quaternion measurement logs.
	*
	* @param ref_sbg_data Contains the received log data as an union.
	* @param user_arg Optional user supplied argument.
	*/
	static void handleLogEkfQuat(const SbgEComLogUnion *ref_sbg_data, void *user_arg);

	/**
	* @brief Parse EKF navigation measurement logs.
	*
	* @param ref_sbg_data Contains the received log data as an union.
	* @param user_arg Optional user supplied argument.
	*/
	static void handleLogEkfNav(const SbgEComLogUnion *ref_sbg_data, void *user_arg);

	/**
	* @brief GNSS position, velocity and heading related logs.
	*
	* @param msg Message ID of the log received.
	* @param ref_sbg_data Contains the received log data as an union.
	* @param user_arg Optional user supplied argument.
	*/
	static void handleLogGnssPosVelHdt(SbgEComMsgId msg, const SbgEComLogUnion *ref_sbg_data, void *user_arg);

	/**
	* @brief Update estimator status message from EKF status flags.
	*
	* @param ekf_status EKF status flags.
	* @param estimator_status Estimator status message.
	*/
	static void updateEstimatorStatus(uint32_t ekf_status, estimator_status_s *estimator_status);

	/**
	* @brief Callback definition called each time a new log is received.
	*
	* @param handle Valid handle on the sbgECom instance that has called this callback.
	* @param msg_class Class of the message we have received
	* @param msg Message ID of the log received.
	* @param ref_sbg_data Contains the received log data as an union.
	* @param user_arg Optional user supplied argument.
	* @return SBG_NO_ERROR if the received log has been used successfully.
	*/
	static SbgErrorCode onLogReceived(SbgEComHandle *handle, SbgEComClass msg_class, SbgEComMsgId msg,
					  const SbgEComLogUnion *ref_sbg_data, void *user_arg);

	/**
	* @brief Send a config to the INS
	*
	* @param pHandle SbgECom instance.
	* @param config Config json string.
	*/
	static void send_config(SbgEComHandle *pHandle, const char *config);

	/**
	* @brief Send a config file to the INS
	*
	* @param pHandle SbgECom instance.
	* @param file_path Config file path.
	*/
	static void send_config_file(SbgEComHandle *pHandle, const char *file_path);

	/**
	* @brief Get and print product info.
	*
	* @param handle SbgECom instance.
	* @return SBG_NO_ERROR if successful.
	*/
	SbgErrorCode getAndPrintProductInfo(SbgEComHandle *handle);

	/**
	* @brief Try to parse one log from the input interface and then return.
	*
	* @param handle A valid sbgECom handle.
	* @return SBG_NO_ERROR if no error occurs during incoming log parsing.
	*/
	SbgErrorCode handleOneLog(SbgEComHandle *handle);

	/**
	* @brief Get air data and send it.
	*
	* @param handle A valid sbgECom handle.
	* @param instance An SbgEcom object.
	* @return SBG_NO_ERROR if no error occurs during incoming log parsing.
	*/
	SbgErrorCode sendAirDataLog(SbgEComHandle *handle, SbgEcom *instance);

	/**
	* @brief Get magnetometer data and send it.
	*
	* @param handle A valid sbgECom handle.
	* @param instance An SbgEcom object.
	* @return SBG_NO_ERROR if no error occurs during incoming log parsing.
	*/
	SbgErrorCode sendMagLog(SbgEComHandle *handle, SbgEcom *instance);

	void set_device_id(uint32_t device_id);
	uint32_t get_device_id(void);

	// SBG interface and state variables
	SbgInterface _sbg_interface;
	SbgEComHandle _com_handle;
	SbgEComLogUnion _log_data;

	uint32_t _baudrate;
	const char *_config_file;
	const char *_config_string;
	char _device_name[25];
	uint32_t _device_id{0};

	const int log_interval = 10;
	int iteration_count = log_interval;

	bool failure = false;
	bool _ekf_failure = false;

	bool _initialized = false;
	int init_result;

	MapProjection _pos_ref{};
	double _gps_alt_ref{NAN};

	struct GnssData {
		bool pos_received = false;
		bool vel_received = false;
		bool hdt_received = false;

		SbgEComLogGnssPos gps_pos;
		SbgEComLogGnssVel gps_vel;
		SbgEComLogGnssHdt gps_hdt;

		hrt_abstime pos_timestamp = 0;
		hrt_abstime vel_timestamp = 0;
		hrt_abstime hdt_timestamp = 0;
	};

	GnssData gnss_data;
	float _heading;

	px4::atomic<hrt_abstime> _time_last_valid_imu_us{false};

	// Sensors topics
	PX4Accelerometer _px4_accel{0};
	PX4Gyroscope	 _px4_gyro{0};
	PX4Magnetometer  _px4_mag{0};

	// Publications with topic dependent on multi-mode
	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};
	uORB::PublicationMulti<vehicle_attitude_s> _attitude_pub{ORB_ID(vehicle_attitude)};
	uORB::PublicationMulti<vehicle_local_position_s> _local_position_pub{ORB_ID(vehicle_local_position)};
	uORB::PublicationMulti<vehicle_global_position_s> _global_position_pub{ORB_ID(vehicle_global_position)};
	uORB::Publication<estimator_status_s> _estimator_status_pub{ORB_ID(estimator_status)};

	// Subscription for INS EKF aiding
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _airspeed_sub{ORB_ID(airspeed)};
	uORB::Subscription _diff_pressure_sub{ORB_ID(differential_pressure)};
	uORB::Subscription _mag_sub{ORB_ID(vehicle_magnetometer)};

	// Performance mounters for monitoring and debugging
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": errors")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": sample")};
	perf_counter_t _write_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": write")};

	perf_counter_t _accel_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": accel publish interval")};
	perf_counter_t _gyro_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": gyro publish interval")};
	perf_counter_t _mag_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": mag publish interval")};
	perf_counter_t _gnss_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": GNSS publish interval")};

	perf_counter_t _attitude_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": attitude publish interval")};
	perf_counter_t _local_position_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": local position publish interval")};
	perf_counter_t _global_position_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": global position publish interval")};
};
