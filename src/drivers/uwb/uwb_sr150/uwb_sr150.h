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
#include <uORB/topics/landing_target_pose.h>
#include <uORB/topics/uwb_grid.h>
#include <uORB/topics/uwb_distance.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/math.hpp>
#include <matrix/Matrix.hpp>

using namespace time_literals;

#define UWB_CMD  0x8e
#define UWB_CMD_START  0x01
#define UWB_CMD_STOP  0x00
#define UWB_CMD_RANGING  0x0A
#define STOP_B 0x0A

#define UWB_PRECNAV_APP   0x04
#define UWB_APP_START     0x10
#define UWB_APP_STOP      0x11
#define UWB_SESSION_START 0x22
#define UWB_SESSION_STOP  0x23
#define UWB_RANGING_START 0x01
#define UWB_RANGING_STOP  0x00
#define UWB_DRONE_CTL     0x0A

#define UWB_CMD_LEN  0x05
#define UWB_CMD_DISTANCE_LEN 0x21
#define UWB_MAC_MODE 2
#define MAX_ANCHORS 12
#define UWB_GRID_CONFIG "/fs/microsd/etc/uwb_grid_config.csv"

typedef struct {  //needs higher accuracy?
	float lat, lon, alt, yaw; //offset to true North
} gps_pos_t;

typedef struct {
	int16_t x, y, z; //axis in cm
} position_t; // Position of a device or target in 3D space

enum UWB_POS_ERROR_CODES {
	UWB_OK,
	UWB_ANC_BELOW_THREE,
	UWB_LIN_DEP_FOR_THREE,
	UWB_ANC_ON_ONE_LEVEL,
	UWB_LIN_DEP_FOR_FOUR,
	UWB_RANK_ZERO
};

typedef struct {
	uint8_t MAC[2];		// MAC Adress of UWB device
	uint8_t status;		// Status of Measurement
	uint16_t distance; 	// Distance in cm
	uint8_t nLos; 		// line of sight y/n
	uint16_t aoaFirst;	// Angle of Arrival of incoming msg
} __attribute__((packed)) UWB_range_meas_t;

typedef struct {
	uint32_t initator_time;  	//timestamp of init
	uint32_t sessionId;	// Session ID of UWB session
	uint8_t	num_anchors;	//number of anchors
	gps_pos_t target_gps; //GPS position of Landing Point
	uint8_t  mac_mode;	// MAC adress mode, either 2 Byte or 8 Byte
	uint8_t MAC[UWB_MAC_MODE][MAX_ANCHORS];
	position_t target_pos; //target position
	position_t anchor_pos[MAX_ANCHORS]; // Position of each anchor
	uint8_t stop; 		// Should be 27
} grid_msg_t;

typedef struct {
	uint8_t cmd;      	// Should be 0x8E for distance result message
	uint16_t len; 		// Should be 0x30 for distance result message
	uint32_t time_uwb_ms;	// Timestamp of UWB device in ms
	uint32_t seq_ctr;	// Number of Ranges since last Start of Ranging
	uint32_t sessionId;	// Session ID of UWB session
	uint32_t range_interval;	// Time between ranging rounds
	uint8_t  mac_mode;	// MAC adress mode, either 2 Byte or 8 Byte
	uint8_t  no_measurements;	// MAC adress mode, either 2 Byte or 8 Byte
	UWB_range_meas_t measurements[4]; //Raw anchor_distance distances in CM 2*9
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
	 * @see ModuleBase::Multilateration
	 */
	UWB_POS_ERROR_CODES localization();

	/**
	 * @see ModuleBase::Distance Result
	 */
	int distance();

	/**
	 * @see ModuleBase::task_spawn
	 */
	static int task_spawn(int argc, char *argv[]);

	static UWB_SR150 *instantiate(int argc, char *argv[]);

	void run() override;

private:
	void parameters_update();

	void grid_info_read(position_t *grid);

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::UWB_INIT_OFF_X>) _uwb_init_off_x,
		(ParamFloat<px4::params::UWB_INIT_OFF_Y>) _uwb_init_off_y,
		(ParamFloat<px4::params::UWB_INIT_OFF_Z>) _uwb_init_off_z,
		(ParamFloat<px4::params::UWB_INIT_OFF_YAW>) _uwb_init_off_yaw
	)

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	int _uart;
	fd_set _uart_set;
	struct timeval _uart_timeout {};
	bool _uwb_pos_debug;

	uORB::Publication<uwb_grid_s> _uwb_grid_pub{ORB_ID(uwb_grid)};
	uwb_grid_s _uwb_grid{};

	uORB::Publication<uwb_distance_s> _uwb_distance_pub{ORB_ID(uwb_distance)};
	uwb_distance_s _uwb_distance{};

	uORB::Publication<landing_target_pose_s> _landing_target_pub{ORB_ID(landing_target_pose)};
	landing_target_pose_s _landing_target{};

	grid_msg_t _uwb_grid_info{};
	distance_msg_t _distance_result_msg{};
	matrix::Vector3f _rel_pos;

	matrix::Dcmf _uwb_init_to_nwu;
	matrix::Dcmf _nwu_to_ned{matrix::Eulerf(M_PI_F, 0.0f, 0.0f)};
	matrix::Vector3f _current_position_uwb_init;
	matrix::Vector3f _current_position_ned;
	matrix::Vector3f _uwb_init_offset_v3f;

	perf_counter_t _read_count_perf;
	perf_counter_t _read_err_perf;
};

#endif //PX4_RDDRONE_H
