/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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
#include <perf/perf_counter.h>
#include <lib/conversion/rotation.h>

#include <px4_platform_common/module_params.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/sensor_uwb.h>
#include <uORB/topics/parameter_update.h>

#include <matrix/math.hpp>

#define UWB_DEFAULT_PORT "/dev/ttyS1"

using namespace time_literals;

typedef struct {
	uint16_t MAC;					// MAC address of UWB device
	uint8_t status;					// Status of Measurement
	uint16_t distance; 				// Distance in cm
	uint8_t nLos; 					// line of sight y/n
	int16_t aoa_azimuth;				// AOA of incoming msg for Azimuth antenna pairing
	int16_t aoa_elevation;				// AOA of incoming msg for Altitude antenna pairing
	int16_t aoa_dest_azimuth;			// AOA destination Azimuth
	int16_t aoa_dest_elevation; 			// AOA destination elevation
	uint8_t aoa_azimuth_FOM;			// AOA Azimuth FOM
	uint8_t aoa_elevation_FOM;			// AOA Elevation FOM
	uint8_t aoa_dest_azimuth_FOM;			// AOA Azimuth FOM
	uint8_t aoa_dest_elevation_FOM;			// AOA Elevation FOM
} __attribute__((packed)) UWB_range_meas_t;

typedef struct {
	uint8_t cmd;      				// Should be 0x8E for distance result message
	uint16_t len; 					// Should be 0x30 for distance result message
	uint32_t seq_ctr;				// Number of Ranges since last Start of Ranging
	uint32_t sessionId;				// Session ID of UWB session
	uint32_t range_interval;			// Time between ranging rounds
	uint16_t MAC;					// MAC address of UWB device
	UWB_range_meas_t measurements; 			//Raw anchor_distance distances in CM 2*9
	uint8_t stop; 					// Should be 0x1B
} __attribute__((packed)) distance_msg_t;

class UWB_SR150 : public ModuleBase<UWB_SR150>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	UWB_SR150(const char *port);
	~UWB_SR150();

	/**
	 * @see ModuleBase::task_spawn
	 */
	static int task_spawn(int argc, char *argv[]);

	/**
	 * @see ModuleBase::custom_command
	 */
	static int custom_command(int argc, char *argv[]);

	/**
	 * @see ModuleBase::print_usage
	 */
	static int print_usage(const char *reason = nullptr);

	bool init();

	void start();

	void stop();

	int collectData();

private:

	void parameters_update();

	void Run() override;

	// Publications
	uORB::Publication<sensor_uwb_s> _sensor_uwb_pub{ORB_ID(sensor_uwb)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_uwb_sub{this, ORB_ID(sensor_uwb)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UWB_PORT_CFG>) 			_uwb_port_cfg,
		(ParamFloat<px4::params::UWB_INIT_OFF_X>) 		_offset_x,
		(ParamFloat<px4::params::UWB_INIT_OFF_Y>) 		_offset_y,
		(ParamFloat<px4::params::UWB_INIT_OFF_Z>) 		_offset_z,
		(ParamInt<px4::params::UWB_SENS_ROT>) 			_sensor_rot
	)
	// Performance (perf) counters
	perf_counter_t _read_count_perf;
	perf_counter_t _read_err_perf;

	sensor_uwb_s _sensor_uwb{};

	char _port[20] {};
	hrt_abstime param_timestamp{0};

	int _uart{-1};
	fd_set _uart_set;
	struct timeval _uart_timeout {};

	unsigned _consecutive_fail_count;
	int _interval{100000};

	distance_msg_t 		_distance_result_msg{};
};
#endif //PX4_RDDRONE_H
