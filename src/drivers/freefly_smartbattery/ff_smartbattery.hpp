/****************************************************************************
 *
 *   Copyright (c) 2021 Auterion. All rights reserved.
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
 * @file ff_smartbattery.hpp
 * Driver for Freefly Smartbatteries based on Lazarus CAN protocol.
 *
 * @author Claudio Micheli <claudio@auterion.com>
 */

#pragma once



#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <hysteresis/hysteresis.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/battery_freefly.h>
#include <uORB/topics/battery_info.h>
#include <uORB/topics/battery_status.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <px4_platform_common/module_params.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_armed.h>


using namespace time_literals;


class FF_SmartBattery : public ModuleBase, public px4::ScheduledWorkItem, public ModuleParams
{
public:
	static Descriptor desc;

	FF_SmartBattery();

	virtual ~FF_SmartBattery();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	static constexpr int8_t MAX_READ_MSG_CNT_PER_LOOP{50};

	static constexpr uint32_t SAMPLE_FREQUENCY_HZ = 10;
	static constexpr uint32_t SAMPLE_INTERVAL_US{1_s / SAMPLE_FREQUENCY_HZ};
	static constexpr int8_t CELL_COUNT{6};
	static constexpr uint8_t BATTERIES_COUNT{2};

	static constexpr uint16_t FW_VERSION_MAJOR{1};
	static constexpr uint16_t FW_VERSION_MINOR{9};

	static constexpr uint8_t EXPECTED_BATTERY_MODEL{1};	//  [Model type]

	static constexpr float INCONSISTENCY_THRESHOLD{2.0f};	//  [V]

	enum SmartBatteryState {
		Reset = 0,
		Standby,
		Shutdown,
		CellHealthCheck,
		CriticalFault,
		IDSense,
		Protected,
		Hotswap = battery_freefly_s::BATTERY_MODE_HOTSWAP, // used
		Airmode = battery_freefly_s::BATTERY_MODE_AIRMODE, // used
		PreviewSoC, // probably doesn't power vehicle
		PreRun, // probably doesn't power vehicle
		StandBy // probably doesn't power vehicle
	};

	enum class AckMessage {
		Status = 0,
		Values,
		Cells1,
		Cells2,
		Count			// Keep it at the end
	};

	enum class FaultFlagMask {
		// Fault Flag 0 message
		Fault_Cell_OV_HW = 0,
		Fault_Cell_UV_HW,
		Fault_OCD,
		Fault_OCD_HW,
		Fault_SCD_HW,
		Fault_OCC,
		Fault_OT_Cells_Chg,
		Fault_OT_Cells_Dsg,
		//Fault Flag 1 message
		Fault_OT_PCB_Chg,
		Fault_OT_PCB_Dsg,
		Fault_UT_Cells_Chg,
		Fault_UT_Cells_Dsg,
		Fault_UT_PCB_Chg,
		Fault_UT_PCB_Dsg,
		Fault_Discharge_Cutoff,
		Fault_CellBalance,
		// Fault Flag 2 message
		Fault_PackMeasure,
		Fault_Cell_UV_Extreme,
		Fault_AFE_Comms,
		Fault_FuelGauge_Comms,
		Fault_FET_Short,
		Fault_OT_FET,
		Fault_Rsvd2,
		Fault_Rsvd3,
		Fault_Rsvd4,
		Count
	};

	enum class StatusFlagMask {
		Status_Mounted = 0,
		Status_OT_Cells_Warning,
		Status_OT_PCB_Warning,
		Count
	};

	static_assert(static_cast<uint32_t>(FaultFlagMask::Count) <= 32, "Maximum number of fault flags reached");

	const char *FAILURES_MSG[static_cast<uint32_t>(FaultFlagMask::Count)] = {
		"Cell Overvoltage Hardware Protection",
		"Cell Undervoltage Hardware Protection",
		"Short Circuit Current Hardware Protection",
		"Over Current Discharge Hardware Protection",
		"Over Current Charge",
		"Over Current Discharge",
		"Over Temperature Cells During Charge",
		"Over Temperature Cells During Discharge",
		"Over Temperature PCB During Charge",
		"Over Temperature PCB During Discharge",
		"Under Temperature Cells During Charge",
		"Under Temperature Cells During Discharge",
		"Under Temperature PCB During Charge",
		"Under Temperature PCB During Discharge",
		"Discharge Cutoff",
		"Cells out of Balance",
		"AFE and Fuel Gauge Voltage Mismatch",
		"Cells Extremely Low Voltage",
		"AFE IC I2C Communications Fault",
		"Fuel Gauge IC I2C Communications Fault",
		"Shorted Cutoff FET Fault"
	};

	/**
	 * Data Codes - used in the first byte of the payload to reflect the data carried.
	 */

	enum class MsgCode {
		RegisterCurVal = 0,
		RegisterWrite = 1,
		RegisterRead = 2,
		SetPublishRate = 3,
		CmdRsp = 4,
		Request = 5,
		Status = 6,
		Values = 7,
		Heartbeat = 8,
		DetailTemp = 9,
		Cell1to3_V = 10,
		Cell4to6_V = 11,
		Cell7to9_V = 12,
		Cell10to12_V = 13,
		DesignSpecs = 14,
		ChargeTerm = 15,
		DischargeCurRate = 16,
		ChargeCurRate = 17,
		Health = 18,
		LifeMaxMin = 19,
		UUID = 20,
		FW_Version = 21,
		FW_CommitID = 22,
		MfgData = 23,
		ModelNameString_0 = 24,
		ModelNameString_1 = 25,
		ModelNameString_2 = 26,
		ModelNameString_3 = 27,
		RequestStatCodes = 28,
		WriteMfgData = 29,
		FieldCount	// MUST ALWAYS BE LAST. Used for Array Indicies, etc
	};

	struct StatusMsg {
		uint8_t StatusFlags;
		uint8_t OutputStatus;
		uint8_t StateOfCharge;
		uint8_t Mode;
		uint8_t FaultFlags[3];
	} __attribute__((packed));

	struct ValuesMsg {
		int8_t Temperature;		// deg C
		uint16_t Voltage;		// mV
		int16_t Current;		// mA*10
		uint16_t Energy_consumed;	// mWh*100
	} __attribute__((packed));

	struct HealthDataMsg {
		uint8_t StateOfHealth;		// %, 0 to 100
		uint32_t CycleCount;		// # of cycles
		uint16_t Capacity;		// mA*10
	} __attribute__((packed));

	struct batteryInfo_s {
		uint16_t status_id;
		uint16_t cmd_id;
		uint8_t battery_id;
	};

	int 		_fd{-1};
	uint32_t 	_update_battery_status[BATTERIES_COUNT] {};
	bool		_was_armed{false};

	hrt_abstime	_last_battery_request{0};

	perf_counter_t	_cycle_perf;
	perf_counter_t  _comms_errors;

	void read_messages();
	void parse_data(struct can_msg_s &msg_p);
	int determineIndex(uint16_t batteryID);
	void update_battery_status(const uint8_t index, const StatusMsg *message);
	void update_battery_values(const uint8_t index, const ValuesMsg *message);
	void update_battery_cells(const uint8_t index, uint8_t CellIndex, const uint8_t *payload_data);
	void publish_messages();
	void update_params();
	void check_vehicle_arm_state();
	void change_mode(const uint8_t mode);
	void request_battery_info();

	// TODO: this is copied from Battery Lib, needs to be cleaned up
	float computeRemainingTime(float current_a, const uint8_t index);

	uint16_t checkFirmwareVersion(const uint8_t index, const uint8_t major, const uint8_t minor);
	uint16_t checkIncompatibleVoltage(const uint8_t index);
	uint16_t checkIncompatibleModel(const uint8_t index, const uint8_t model);


	uint8_t determineWarning(const float remaining_v);
	uint16_t determinePX4FaultFlags(const uint8_t index, const uint8_t FaultFlags[3], const uint8_t statusFlags,
					const uint32_t custom_faults);

	int read_can_data(uint8_t *buf, int len);
	int open_can_port();

	int send_can_msg(const struct can_msg_s &msg_p);



	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BAT_CRIT_THR>) _crit_thr,
		(ParamFloat<px4::params::BAT_LOW_THR>) _low_thr,
		(ParamFloat<px4::params::BAT_EMERGEN_THR>) _emergency_thr

	)

	systemlib::Hysteresis _battery_arm_hysteresis{true};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};

	uORB::PublicationMultiData<battery_freefly_s> _battery_freefly_pub[BATTERIES_COUNT] {ORB_ID(battery_freefly), ORB_ID(battery_freefly)};
	uORB::PublicationMultiData<battery_info_s> _battery_info_pub[BATTERIES_COUNT] {ORB_ID(battery_info), ORB_ID(battery_info)};
	uORB::PublicationMulti<battery_status_s> _battery_status_pub[BATTERIES_COUNT] {ORB_ID(battery_status), ORB_ID(battery_status)};

	battery_status_s _battery_status[BATTERIES_COUNT] {};


	// no copy, no assignment
	FF_SmartBattery(const FF_SmartBattery &) = delete;
	FF_SmartBattery operator=(const FF_SmartBattery &) = delete;

	const batteryInfo_s _battery_info[BATTERIES_COUNT] = {
		[0] = {				// Right battery (looking the vehicle from the front)
			.status_id = 0x403,
			.cmd_id = 0x40b,
			.battery_id = 1
		},
		[1] = {				// Left battery
			.status_id = 0x402,
			.cmd_id = 0x40a,
			.battery_id = 2
		}
	};

	AlphaFilter<float> _current_filter_a[BATTERIES_COUNT];
	AlphaFilter<float> _current_average_filter_a[BATTERIES_COUNT];
};
