/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#ifndef PX4_RDDRONE_H
#define PX4_RDDRONE_H

#include <termios.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/module.h>
#include <perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/sensor_uwb.h>
#include <uORB/topics/parameter_update.h>

#include <matrix/math.hpp>

using namespace time_literals;

typedef struct {
	uint8_t MAC[2];					// MAC address of UWB device
	uint8_t status;					// Status of Measurement
	uint16_t distance; 				// Distance in cm
	uint8_t nLos; 					// line of sight y/n
	int16_t aoa_azimuth;			// AOA of incoming msg for Azimuth antenna pairing
	int16_t aoa_elevation;			// AOA of incoming msg for Altitude antenna pairing
	int16_t aoa_dest_azimuth;		// AOA destination Azimuth
	int16_t aoa_dest_elevation; 	// AOA destination elevation
	uint8_t aoa_azimuth_FOM;		// AOA Azimuth FOM
	uint8_t aoa_elevation_FOM;		// AOA Elevation FOM
	uint8_t aoa_dest_azimuth_FOM;	// AOA Azimuth FOM
	uint8_t aoa_dest_elevation_FOM;	// AOA Elevation FOM
} __attribute__((packed)) UWB_range_meas_t;

typedef struct {
	uint8_t cmd;      			// Should be 0x8E for distance result message
	uint16_t len; 				// Should be 0x30 for distance result message
	uint32_t seq_ctr;			// Number of Ranges since last Start of Ranging
	uint32_t sessionId;			// Session ID of UWB session
	uint32_t range_interval;	// Time between ranging rounds
	uint8_t MAC[2];			// MAC address of UWB device
	UWB_range_meas_t measurements; //Raw anchor_distance distances in CM 2*9
	uint8_t stop; 		// Should be 0x1B
} __attribute__((packed)) distance_msg_t;


class UWB_SR150 : public ModuleBase<UWB_SR150>, public ModuleParams
{
public:
	UWB_SR150(const char *device_name, speed_t baudrate, bool uwb_pos_debug);

	~UWB_SR150();

	/**
	 * @see ModuleBase::custom_command
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage
	 */
	static int print_usage(const char *reason = nullptr);

	/**
	 * @see ModuleBase::Distance Result
	 */
	int collectData();

	/**
	 * @see ModuleBase::task_spawn
	 */
	static int task_spawn(int argc, char *argv[]);

	static UWB_SR150 *instantiate(int argc, char *argv[]);

	void run() override;

	// void stop();

private:
	static constexpr int64_t sq(int64_t x) { return x * x; }

	void parameters_update();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UWB_PORT_CFG>) 			_uwb_port_cfg,
		// (ParamInt<px4::params::UWB_DRIVER_EN>) 			_uwb_driver_en,
		(ParamFloat<px4::params::UWB_INIT_OFF_X>) 		_uwb_init_off_x,
		(ParamFloat<px4::params::UWB_INIT_OFF_Y>) 		_uwb_init_off_y,
		(ParamFloat<px4::params::UWB_INIT_OFF_Z>) 		_uwb_init_off_z,
		(ParamFloat<px4::params::UWB_INIT_YAW>) 		_uwb_init_off_yaw,
		(ParamFloat<px4::params::UWB_INIT_PITCH>) 		_uwb_init_off_pitch,
		(ParamInt<px4::params::UWB_SENS_ROT>) 			_uwb_sens_rot
	)


	uORB::SubscriptionInterval 						_parameter_update_sub{ORB_ID(parameter_update), 1_s};

	hrt_abstime param_timestamp{0};

	int _uart;
	fd_set _uart_set;
	struct timeval _uart_timeout {};
	// need to figure out debugging/error handling still...
	bool _uwb_pos_debug;

	uORB::Publication<sensor_uwb_s> _sensor_uwb_pub{ORB_ID(sensor_uwb)};
	sensor_uwb_s _sensor_uwb{};

	distance_msg_t _distance_result_msg{};
	matrix::Vector3d _rel_pos;

	matrix::Vector3d _uwb_init_offset;
	matrix::Vector3d _uwb_init_attitude;

	perf_counter_t _read_count_perf;
	perf_counter_t _read_err_perf;
};
#endif //PX4_RDDRONE_H
