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
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/events.h>
#include <systemlib/mavlink_log.h>


#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <drivers/device/device.h>
#include <lib/geo/geo.h>
#include <termios.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/transponder_report.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

#include "sg_sdk/sagetech_mxs.h"

using namespace time_literals;

class SagetechMXS : public ModuleBase<SagetechMXS>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SagetechMXS(const char *port);
	~SagetechMXS() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void print_info();

	bool init();

	int print_status() override;

private:
	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ADSB_SQUAWK>)		_adsb_squawk,
		(ParamInt<px4::params::ADSB_IDENT>)		_adsb_ident,
		(ParamInt<px4::params::ADSB_LIST_MAX>)		_adsb_list_max,
		(ParamInt<px4::params::ADSB_ICAO_ID>)		_adsb_icao,
		(ParamInt<px4::params::ADSB_LEN_WIDTH>)		_adsb_len_width,
		(ParamInt<px4::params::ADSB_EMIT_TYPE>)		_adsb_emit_type,
		(ParamInt<px4::params::ADSB_MAX_SPEED>)		_adsb_max_speed,
		(ParamInt<px4::params::ADSB_ICAO_SPECL>)	_adsb_icao_specl,
		(ParamInt<px4::params::ADSB_EMERGC>)		_adsb_emergc,
		(ParamInt<px4::params::MXS_OP_MODE>)		_mxs_op_mode,
		(ParamInt<px4::params::MXS_TARG_PORT>)		_mxs_targ_port,
		// (ParamInt<px4::params::MXS_COM0_BAUD>)		_mxs_com0_baud,
		// (ParamInt<px4::params::MXS_COM1_BAUD>)		_mxs_com1_baud,
		(ParamInt<px4::params::MXS_EXT_CFG>)		_mxs_ext_cfg,
		(ParamInt<px4::params::SER_MXS_BAUD>)		_ser_mxs_baud
	);

	// Serial Port Variables
	char _port[20] {};
	int  _fd{-1};

	// Publications
	uORB::Publication<transponder_report_s> _transponder_report_pub{ORB_ID(transponder_report)};
	orb_advert_t _mavlink_log_pub{nullptr};


	// Subscriptions
	uORB::Subscription                 _sensor_gps_sub{ORB_ID(sensor_gps)};
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s}; // subscription limited to 1 Hz updates
	uORB::Subscription                 _vehicle_status_sub{ORB_ID(vehicle_status)};          // regular subscription for additional data
	uORB::Subscription                 _transponder_report_sub{ORB_ID(transponder_report)};
	uORB::Subscription                 _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};


	// Performance (perf) counters
	perf_counter_t	_loop_count_perf{perf_alloc(PC_COUNT, MODULE_NAME": run_loop")};
	perf_counter_t	_loop_elapsed_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t  _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t  _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};

	// Constants
	static constexpr uint32_t UPDATE_INTERVAL_US{1000000 / 50}; 	// 20ms = 50 Hz
	static constexpr uint8_t FIVE_HZ_MOD{10};			// 0.2s = 5 Hz
	static constexpr uint8_t TWO_HZ_MOD{25};			// 0.5s = 2 Hz
	static constexpr uint8_t ONE_HZ_MOD{50};			// 1 Hz
	static constexpr uint16_t EIGHT_TWO_SEC_MOD{410};		// 8.2 seconds
	// static constexpr uint8_t  SG_MSG_START_BYTE{0xAA};
	static constexpr uint32_t PAYLOAD_MXS_MAX_SIZE{255};
	static constexpr float SAGETECH_SCALE_FEET_TO_M{0.3048F};
	static constexpr float SAGETECH_SCALE_KNOTS_TO_M_PER_SEC{0.514444F};
	static constexpr float SAGETECH_SCALE_M_PER_SEC_TO_KNOTS{1.94384F};
	static constexpr float SAGETECH_SCALE_FT_PER_MIN_TO_M_PER_SEC{0.00508F};
	static constexpr double SAGETECH_SCALE_M_TO_FT{3.28084};
	static constexpr float SAGETECH_SCALE_M_PER_SEC_TO_FT_PER_MIN{196.85F};
	static constexpr uint8_t ADSB_ALTITUDE_TYPE_PRESSURE_QNH{0};
	static constexpr uint8_t ADSB_ALTITUDE_TYPE_GEOMETRIC{1};
	static constexpr uint16_t MAX_VEHICLES_LIMIT{50};
	static constexpr float SAGETECH_HPL_UNKNOWN{38000.0F};
	static constexpr float CLIMB_RATE_LIMIT{16448};
	static constexpr uint16_t MXS_INIT_TIMEOUT_COUNT{1000};		// 1000 loop cycles = 20 seconds
	static constexpr uint8_t BASE_OCTAL{8};
	static constexpr uint8_t BASE_HEX{16};
	static constexpr uint8_t BASE_DEC{10};
	static constexpr uint16_t INVALID_SQUAWK{7777};
	static constexpr unsigned BAUD_460800{0010004};			// B460800 not defined in MacOS termios
	static constexpr unsigned BAUD_921600{0010007};			// B921600 not defined in MacOS termios

	// Stored variables
	uint64_t _loop_count;

	// Cached Subscription Data
	sensor_gps_s _gps;
	vehicle_land_detected_s _landed;

	enum class MsgType : uint8_t {
		Installation            = SG_MSG_TYPE_HOST_INSTALL,
		FlightID                = SG_MSG_TYPE_HOST_FLIGHT,
		Operating               = SG_MSG_TYPE_HOST_OPMSG,
		GPS_Data                = SG_MSG_TYPE_HOST_GPS,
		Data_Request            = SG_MSG_TYPE_HOST_DATAREQ,
		// RESERVED 0x06 - 0x0A
		Target_Request          = SG_MSG_TYPE_HOST_TARGETREQ,
		Mode                    = SG_MSG_TYPE_HOST_MODE,
		// RESERVED 0x0D - 0xC1
		ACK                     = SG_MSG_TYPE_XPNDR_ACK,
		Installation_Response   = SG_MSG_TYPE_XPNDR_INSTALL,
		FlightID_Response       = SG_MSG_TYPE_XPNDR_FLIGHT,
		Status_Response         = SG_MSG_TYPE_XPNDR_STATUS,
		RESERVED_0x84           = 0x84,
		RESERVED_0x85           = 0x85,
		Mode_Settings           = SG_MSG_TYPE_XPNDR_MODE,
		RESERVED_0x8D           = 0x8D,
		Version_Response        = SG_MSG_TYPE_XPNDR_VERSION,
		Serial_Number_Response  = SG_MSG_TYPE_XPNDR_SERIALNUM,
		Target_Summary_Report   = SG_MSG_TYPE_ADSB_TSUMMARY,

		ADSB_StateVector_Report = SG_MSG_TYPE_ADSB_SVR,
		ADSB_ModeStatus_Report  = SG_MSG_TYPE_ADSB_MSR,
		ADSB_Target_State_Report = SG_MSG_TYPE_ADSB_TSTATE,
		ADSB_Air_Ref_Vel_Report = SG_MSG_TYPE_ADSB_ARVR,
	};

	enum class ParseState {
		WaitingFor_Start,
		WaitingFor_MsgType,
		WaitingFor_MsgId,
		WaitingFor_PayloadLen,
		WaitingFor_PayloadContents,
		WaitingFor_Checksum,
	};

	struct __attribute__((packed)) Packet {
		const uint8_t   start = SG_MSG_START_BYTE;
		MsgType         type;
		uint8_t         id;
		uint8_t         payload_length;
		uint8_t         payload[PAYLOAD_MXS_MAX_SIZE];
	};

	struct {
		ParseState      state;
		uint8_t         index;
		Packet          packet;
		uint8_t         checksum;
	} _message_in;

	struct {
		bool initialized;
		bool init_failed;
		sg_operating_t op;
		sg_install_t inst;
		// sg_gps_t gps;
		sg_targetreq_t treq;
		sg_flightid_t fid;
		sg_ack_t ack;
	} mxs_state;

	struct {
		// cached variables to compare against params so we can send msg on param change.
		bool            failXpdr;
		bool            failSystem;
		struct {
			uint8_t     id;
			uint8_t     type;
		} msg;
	} last;

	// External Vehicle List
	transponder_report_s *vehicle_list = nullptr;
	uint16_t list_size_allocated;
	uint16_t vehicle_count = 0;
	uint16_t furthest_vehicle_index = 0;
	float furthest_vehicle_distance = 0;

	// Functions
	void Run() override;
	void handle_packet(const Packet &msg);
	int msg_write(const uint8_t *data, const uint16_t len) const;
	bool parse_byte(const uint8_t data);
	void handle_ack(const sg_ack_t ack);
	void handle_svr(const sg_svr_t svr);
	void handle_msr(sg_msr_t msr);
	bool get_vehicle_by_ICAO(const uint32_t icao, transponder_report_s &vehicle) const;
	bool find_index(const transponder_report_s &vehicle, uint16_t *index) const;
	void set_vehicle(const uint16_t index, const transponder_report_s &vehicle);
	void delete_vehicle(const uint16_t index);
	bool is_special_vehicle(uint32_t icao) const {return (_adsb_icao_specl.get() != 0) && (_adsb_icao_specl.get() == (int32_t) icao);}
	void handle_vehicle(const transponder_report_s &vehicle);
	void determine_furthest_aircraft();
	void send_data_req(const sg_datatype_t dataReqType);
	void send_install_msg();
	void send_flight_id_msg();
	void send_operating_msg();
	void send_gps_msg();
	void send_targetreq_msg();
	uint32_t convert_base_to_decimal(const uint8_t baseIn, uint32_t inputNumber);
	int open_serial_port();
	sg_emitter_t convert_emitter_type_to_sg(int emitType);
	int convert_sg_to_emitter_type(sg_emitter_t sg_emit);
	int handle_fid(const char *fid);
	int store_inst_resp();
	void auto_config_operating();
	void auto_config_installation();
	void auto_config_flightid();
	unsigned convert_to_px4_baud(int baudType);
	bool check_valid_squawk(int squawk);
};
