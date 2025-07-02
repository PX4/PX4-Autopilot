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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/atomic.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/esc_status.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>

#include <lib/mixer_module/mixer_module.hpp>
#include <lib/cdev/CDev.hpp>

#include "led_control.h"

using namespace time_literals;

extern "C" __EXPORT int ff_escs_main(int argc, char *argv[]);

class FFescOutput : public ModuleBase, public OutputModuleInterface
{
public:
	static Descriptor desc;

	static constexpr int8_t MOTOR_COUNT = LedControl::MOTOR_COUNT; // only define motor count once
	FFescOutput();
	virtual ~FFescOutput();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	virtual int	init();
	bool updateOutputs(float outputs[MOTOR_COUNT], unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	static constexpr int8_t MAX_READ_MSG_CNT_PER_LOOP{50};
	static constexpr uint64_t MOTOR_ERROR_TIMEOUT{1_s};

	static constexpr float ESC_WARN_TEMP{95.f};
	static constexpr float ESC_MAX_TEMP{105.f};
	static constexpr float MOTOR_WARN_TEMP{120.f};
	static constexpr float MOTOR_MAX_TEMP{150.f};

	// THE FOLLOWING ARE THE DEFINES FROM THE ALTA ESC FOR REFERENCE ONLY !!!

	static constexpr unsigned char CAN_ADDR_ARM_SET{0x25};
	static constexpr unsigned char CAN_ADDR_DISARM_SET{0x23};

	static constexpr unsigned char CAN_ADDR_CONTROL_1{0x28};
	static constexpr unsigned char CAN_ADDR_CONTROL_2{0x29};

	// static constexpr unsigned char CAN_ADDR_BOOM_SET{0x21};
	// static constexpr unsigned char CAN_ADDR_BOOM_ACK{0x20};
	// static constexpr unsigned char CAN_ADDR_ARBITRARY{0x35};
	// static constexpr unsigned char CAN_ADDR_ARBITRARY_2{0x36};
	// static constexpr unsigned char CAN_ADDR_ARBITRARY_HIGH_PRIORITY{0x26};
	// static constexpr unsigned char CAN_ADDR_FLASH_ACK{0x34};
	// static constexpr unsigned char CAN_ADDR_TELEMETRY_REQUEST{0x2A};

	// static constexpr unsigned char CAN_ADDR_PARAMETER_READ{0x30};
	// static constexpr unsigned char CAN_ADDR_PARAMETER_READ_RESPONSE{0x31};
	// static constexpr unsigned char CAN_ADDR_PARAMETER_SET{0x32};
	// static constexpr unsigned char CAN_ADDR_PARAMETER_SET_RESPONSE{0x33};
	// static constexpr unsigned char CAN_ADDR_LED_SET{0x38};
	static constexpr unsigned char CAN_ADDR_REVERSE{0x3F};
	// static constexpr unsigned char CAN_ADDR_REVERSE_FAST{0x3E};
	// static constexpr unsigned char CAN_FLASH_ACK_MESSAGE{0x2};
	// static constexpr unsigned char CAN_UNLOCK_ACK_MESSAGE{0x1};
	// static constexpr unsigned char CAN_LOCK_ACK_MESSAGE{0x3};
	// static constexpr unsigned char CAN_ARB_UNLOCK{0x0};
	// static constexpr unsigned char CAN_ARB_FLASH{0x1};
	// static constexpr unsigned char CAN_ARB_LOCK{0x2};
	// static constexpr unsigned char CAN_ARB_BEEP{0x3};
	// static constexpr unsigned char CAN_ARB_ARM_LED{0x4};

	// static constexpr uint64_t CAN_VERIFICATION_BOOM_HR{0x50039871};
	// static constexpr uint64_t CAN_VERIFICATION_BOOM_LR{0x05956200};
	// static constexpr uint64_t CAN_VERIFICATION_BOOM_LR_MASK{0xFFFFFF00};
	// static constexpr uint64_t CAN_VERIFICATION_BOOM_ACK_HR{0x1e62125e};
	// static constexpr uint64_t CAN_VERIFICATION_BOOM_ACK_LR{0x2c7bdd00};//Transmit only, no mask
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_UNLOCK_HR{0x0a08b5bf};
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_UNLOCK_LR{0xd8160000};
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_UNLOCK_LR_MASK{0xFFFF0000};
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_UNLOCK_ACK_HR{0xd20db2dd};
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_UNLOCK_ACK_LR{0x96710000};//Transmit only, no mask
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_LOCK_ACK_HR{0xac433647};
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_LOCK_ACK_LR{0x6e020000};//Transmit only, no mask
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_HR{0x00572e7b};
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_LR{0xb8380000};
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_LR_MASK{0xFFFF0000};
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_ACK_HR{0x81d6863d};
	// static constexpr uint64_t CAN_VERIFICATION_FLASH_ACK_LR{0xf6390000}; //Transmit only, no mask
	// static constexpr uint64_t CAN_VERIFICATION_ARM_HR{0x91258e44};
	// static constexpr uint64_t CAN_VERIFICATION_ARM_LR{0x028d5300};
	// static constexpr uint64_t CAN_VERIFICATION_ARM_LR_MASK{0xFFFFFF00};
	// static constexpr uint64_t CAN_VERIFICATION_DISARM_HR{0xf1d1d688};
	// static constexpr uint64_t CAN_VERIFICATION_DISARM_LR{0x2b525c00};
	// static constexpr uint64_t CAN_VERIFICATION_DISARM_LR_MASK{0xFFFFFF00};
	static constexpr uint64_t CAN_VERIFICATION_REVERSE_HR{0x1df1f778};
	static constexpr uint64_t CAN_VERIFICATION_REVERSE_LR{0x6418aa00};
	// static constexpr uint64_t CAN_VERIFICATION_REVERSE_LR_MASK{0xFFFFFF00};
	// static constexpr uint64_t CAN_VERIFICATION_ARM_ACK_HR{0xc18b42c1};
	// static constexpr uint64_t CAN_VERIFICATION_ARM_ACK_LR{0xc4ae5f00};
	// static constexpr uint64_t CAN_VERIFICATION_DISARM_ACK_HR{0x70d0dc37};
	// static constexpr uint64_t CAN_VERIFICATION_DISARM_ACK_LR{0x2cee2100};
	// static constexpr uint64_t CAN_VERIFICATION_REVERSE_FAST_LR{0xa2370000};
	// static constexpr uint64_t CAN_VERIFICATION_REVERSE_FAST_LR_MASK{0xFFFF0000};
	// static constexpr uint64_t CAN_VERIFICATION_REVERSE_FAST_HR{0x0000007d};
	// static constexpr uint64_t CAN_VERIFICATION_REVERSE_FAST_HR_MASK{0x000000FF};

	static constexpr unsigned char CAN_ADDR_ARM_ACK{0x24};
	static constexpr unsigned char CAN_ADDR_DISARM_ACK{0x22};

	static constexpr unsigned char CAN_ADDR_TELEMETRY_RESPONSE{0x2B};
	static constexpr unsigned char CAN_ADDR_TELEMETRY_RESPONSE_2{0x2C};

	static constexpr unsigned char CAN_UID_RESPONSE{0x08};
	static constexpr unsigned char CAN_SET_ADDR_JUMP_ACK{0x41};
	static constexpr unsigned char CAN_EXECUTE_JUMP_ACK{0x4D};

	static constexpr uint32_t CAN_JUMP_BOOTLOADER_ADDRESS{0x08008000};
	static constexpr uint32_t CAN_OSD_ID{0x09};
	static constexpr uint32_t CAN_POWER_MODULE_ID{0x10};

	static constexpr hrt_abstime LED_BLINK_SPEED_SLOW{500_ms};
	static constexpr hrt_abstime LED_BLINK_SPEED_FAST{200_ms};

	enum EscFault {
		OverCurrent = 0,
		DeadPwm,
		HardwareFault,
		OverVoltage,
		CommandInconsistency,
		BootTimeCheckFail,
		MotorStopped,
		CANFailed
	};

	enum EscFault2 {
		DRV_OTW = 0,
		DRV_135C,
		DRV_125C,
		DRV_105C
	};

	// Esc telemetry 64 Bits
	// bits 0 to 3 = boom ID
	// bits 4 to 15 = voltage * 0.064
	// bits 16 to 23 = fault state
	// bits 24 to 31 = temperature + 100
	// bits 32 to 47 = rpm
	// bits 48 to 63 = current (Amps) - 32768 * 0.1
	struct EscTelemetry {
		uint8_t esc_id : 4;
		uint16_t voltage : 12;
		uint8_t fault_state;
		uint8_t motor_temperature;
		uint16_t rpm;
		uint16_t current;
	} __attribute__((packed));

	// ESC telemetry 2 bits
	// 8 bits boom id
	// 8 bits motor state
	// 8 bits MCU temp	// ESC temp, unsigned int -> -55 to 200, so this value represents temp+55
	// 8 bits flux (wb*100000)
	// 32 bits reserved
	struct EscTelemetry2 {
		uint8_t esc_id;
		uint8_t esc_control_state;
		uint8_t esc_temperature;
		uint8_t motor_flux;
		uint32_t reserved;
	} __attribute__((packed));

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::PublicationData<esc_status_s> _esc_status_pub{ORB_ID(esc_status)};

	int _file_descriptor{-1}; // of CAN device

	MixingOutput _mixing_output{"FF_ESC", MOTOR_COUNT, *this, MixingOutput::SchedulingPolicy::Auto, false, true};
	LedControl _led_control{this};

	bool _armed_motors{false}; ///< if any of the motors should spin and hence the ECSs need to be in the armed state
	uint8_t _armed_esc_mask{0}; ///< bitfield of what ESCs were sent an arm request and no disarm yet
	uint8_t _arming_state_acknowledged_esc_mask{0}; ///< bitfield of what ESCs acknowledged the current state of _armed_esc_mask
	uint8_t _arming_state_request_index{0}; ///< index of ESC we are awaiting an acknowledge from
	hrt_abstime _arming_state_requested_time{0}; ///< timestamp of command while waiting for an acknowledgement, 0 while not waiting

	hrt_abstime _time_at_boot{0}; ///< timestamp to prevent sending out telemetry when not yet initialized

	uint8_t 	_bootloader_ids[MOTOR_COUNT] {0};

	uint8_t		_telemetry_index{1};
	uint8_t		_last_telemetry_esc_id{0};
	bool		_publish_esc_status{false};

	hrt_abstime	_last_motor_overtemp_time[MOTOR_COUNT] {0};
	uint8_t		_last_motor_overtemp_fault[MOTOR_COUNT] {0};

	hrt_abstime	_last_esc_overtemp_time[MOTOR_COUNT] {0};
	uint16_t	_last_esc_overtemp_fault[MOTOR_COUNT] {0};

	bool 		_esc_answer_timeout{false};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t  _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": comms_errors")};

	uint16_t localPWMList[MOTOR_COUNT];

	actuator_outputs_s _actuator_outputs{};

	px4::atomic<const can_msg_s *> _new_command{nullptr};

	int 			initialize_driver();

	int 			send_can_msg(const struct can_msg_s &msg_p);
	int 			read_can_data(uint8_t *buf, int len);
	void 			read_messages();

	void 			parse_data(struct can_msg_s &msg_p);
	uint16_t 		convert_pwm_to_can(float pwmIn);

	int 			send_command_thread_safe(uint8_t motor_index);

	void 			request_esc_telemetry(uint8_t current_esc);

	void			enable_esc_output(const uint8_t index);
	void			disable_esc_output(const uint8_t index);

	uint8_t 		eval_telem1_failures(const uint8_t fault_state);
	uint16_t		eval_telem2_failures(const uint32_t fault_state);

	uint8_t 		check_online_escs(const esc_status_s &esc_status);
	uint8_t 		check_motor_temperature(const float motor_temperature, const uint8_t index);
	uint16_t		check_esc_temperature(const float esc_temperature, const uint8_t index);
};
