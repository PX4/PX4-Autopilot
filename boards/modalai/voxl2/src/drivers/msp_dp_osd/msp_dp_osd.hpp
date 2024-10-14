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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/home_position.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/log_message.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/mavlink_tunnel.h>

#include <uORB/topics/esc_status.h>
#include <uORB/topics/parameter_selector.h>
#include <uORB/topics/manual_control_setpoint.h>


#include "MspDPV1.hpp"
#include <drivers/osd/msp_osd/MessageDisplay/MessageDisplay.hpp>
#include <drivers/osd/msp_osd/uorb_to_msp.hpp>
#include "uorb_to_msp_dp.hpp"
#include "msp_osd_symbols.h"

using namespace time_literals;
using namespace msp_osd;

// location to "hide" unused display elements
#define LOCATION_HIDDEN 234;

struct PerformanceData {
	bool initialization_problems{false};
	long unsigned int successful_sends{0};
	long unsigned int unsuccessful_sends{0};
};

// mapping from symbol name to bit in the parameter bitmask
//  @TODO investigate params; it seems like this should be available directly?
enum DisplayOptionIndex : uint8_t {
	SHOW_HEADING_AS_CLOCK_DIR = 0
};

class MspDPOsd : public ModuleBase<MspDPOsd>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	MspDPOsd(const char *device);

	~MspDPOsd() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	static constexpr float    MSP_OSD_HEADING_RESET_THRESHOLD = 0.95f;
	static constexpr uint32_t MSP_OSD_AUX1 = 1;
	static constexpr uint32_t MSP_OSD_AUX2 = 2;

	void Run() override;

	// update a single display element in the display
	void Send(const unsigned int message_type, const void *payload, mspDirection_e direction = MSP_DIRECTION_REQUEST);

	// send full configuration to MSP (triggers the actual update)
	void SendConfig();
	void SendTelemetry();

	// perform actions required for local updates
	void parameters_update();

	MspDPV1 _msp{0};
	int _msp_fd{-1};

	msp_osd::MessageDisplay _display{};
	msp_osd::MessageDisplay _warning{};

	bool _is_initialized{false};

	// subscriptions to desired vehicle display information
	uORB::Subscription _airspeed_validated_sub{ORB_ID(airspeed_validated)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _home_position_sub{ORB_ID(home_position)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _log_message_sub{ORB_ID(log_message)};			// Comes from mavlink_log_{LEVEL}() , keeps track of warnings and error messages
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _esc_status_sub{ORB_ID(esc_status)};
	uORB::Subscription _parameter_selector_sub{ORB_ID(parameter_selector)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

	uORB::SubscriptionCallbackWorkItem _vehicle_mavlink_tunnel_sub{this, ORB_ID(mavlink_tunnel)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// local heartbeat
	bool _heartbeat{false};

	static const int MAX_REMOTE_OSD_FIELDS{2};
	static const int MAX_REMOTE_OSD_STRING_LEN{50};
	typedef struct {
		int row;
		int col;
		char string[MAX_REMOTE_OSD_STRING_LEN + 1];
	} msp_dp_remote_osd_t;
	msp_dp_remote_osd_t _remote_osd[MAX_REMOTE_OSD_FIELDS];

	typedef struct {
		int32_t		rssi_col;
		int32_t		rssi_row;
		int32_t		current_draw_col;
		int32_t		current_draw_row;
		int32_t		battery_col;
		int32_t		battery_row;
		int32_t		cell_battery_col;
		int32_t		cell_battery_row;
		int32_t		ready_col;
		int32_t		ready_row;
		int32_t		status_col;
		int32_t		status_row;
		int32_t		flight_mode_col;
		int32_t		flight_mode_row;
		int32_t 	latitude_col;
		int32_t 	latitude_row;
		int32_t 	longitude_col;
		int32_t		longitude_row;
		int32_t		to_home_col;
		int32_t		to_home_row;
		int32_t		crosshair_col;
		int32_t		crosshair_row;
		int32_t		heading_col;
		int32_t		heading_row;

		float osd_batt_low_v;
		int32_t osd_display_opts;
		int32_t osd_hdg_rst_chan;
	} msp_dp_osd_params_t;

	// Helpers from uOrb topics to MSP topics
	msp_battery_state_t construct_BATTERY_STATE(const battery_status_s& battery_status);

	// construct an MSP_NAME struct
	//  note: this is actually how we display _all_ string information
	msp_name_t construct_display_message(const struct vehicle_status_s& vehicle_status,
					const struct vehicle_attitude_s& vehicle_attitude,
					const struct log_message_s& log_message,
					const struct esc_status_s& esc_status,
					const struct parameter_selector_s& parameter_selector,
					const int log_level,
					msp_osd::MessageDisplay& display);

	msp_name_t construct_warning_message(const struct log_message_s& log_message,
					const int log_level,
					msp_osd::MessageDisplay& display);

	// Helper functions for constructing the display message
	void set_arm_status_string(const hrt_abstime& now, const struct vehicle_status_s& vehicle_status, msp_osd::MessageDisplay& display);
	void set_flight_mode_string(const hrt_abstime& now, const struct vehicle_status_s& vehicle_status, const struct esc_status_s& esc_status, const struct parameter_selector_s& parameter_selector, msp_osd::MessageDisplay& display);
	void set_warning_string(const hrt_abstime& now, const struct log_message_s& log_message, const int log_level, msp_osd::MessageDisplay& display);
	void set_heading_string(const hrt_abstime& now, const struct vehicle_attitude_s& vehicle_attitude, msp_osd::MessageDisplay& display);

	// parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::OSD_SYMBOLS>)     _param_osd_symbols,
		(ParamInt<px4::params::OSD_CH_HEIGHT>)   _param_osd_ch_height,
		(ParamInt<px4::params::OSD_SCROLL_RATE>) _param_osd_scroll_rate,
		(ParamInt<px4::params::OSD_DWELL_TIME>)  _param_osd_dwell_time,
		(ParamInt<px4::params::OSD_LOG_LEVEL>)   _param_osd_log_level,

		(ParamInt<px4::params::OSD_CH_COL>)  	 _param_osd_ch_col,
		(ParamInt<px4::params::OSD_CH_ROW>)   	 _param_osd_ch_row,


		(ParamInt<px4::params::OSD_HDG_COL>)  	 _param_osd_hdg_col,
		(ParamInt<px4::params::OSD_HDG_ROW>)   	 _param_osd_hdg_row

		// (ParamFloat<px4::params::OSD_BATT_LOW_V>) _param_batt_low_v,
		// (ParamInt<px4::params::OSD_DISPLAY_OPTS>) _param_osd_disp_opts,
		// (ParamInt<px4::params::OSD_HDG_RST_CHAN>) _param_osd_hdg_rst_chan
	)

	// metadata
	msp_dp_rc_sticks_t	_sticks{0};
	msp_dp_osd_params_t	_parameters{0};
	char _device[64] {};
	uint8_t fontType{0};
	resolutionType_e resolution{HD_5018};
	uint8_t row_max[4]{15,17,15,19};
	uint8_t column_max[4]{29,49,29,52};
	PerformanceData _performance_data{};

	// Default to 25mW power and channel to R1 (Band 5, Channel 1, freq 5658)
	uint8_t _power{1};
	uint8_t _band{5};
	uint8_t _channel{1};
	uint16_t _frequency{0x161A};

	bool _remote_enable{0};

	float _osd_heading_origin = {0.0f};
};
