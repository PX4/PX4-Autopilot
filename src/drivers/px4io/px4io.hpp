/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file px4io.hpp
 * Driver for the PX4IO board.
 *
 * PX4IO is connected via DMA enabled high-speed UART.
 */

#include <px4_platform_common/events.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/sem.hpp>

#include <crc32.h>

#include <debug.h>

#include <lib/circuit_breaker/circuit_breaker.h>
#include <lib/button/ButtonPublisher.hpp>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/rc/dsm.h>
#include <lib/systemlib/mavlink_log.h>

#include <modules/px4iofirmware/protocol.h>

#include <uORB/topics/heater_status.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/px4io_status.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/vehicle_status.h>

#include "modules/dataman/dataman.h"

#include "px4io_driver.h"

#include "uploader.h"

#define PX4IO_DEVICE_PATH               "/dev/px4io"

#define PX4IO_SET_DEBUG                 _IOC(0xff00, 0)
#define PX4IO_REBOOT_BOOTLOADER         _IOC(0xff00, 1)
#define PX4IO_CHECK_CRC                 _IOC(0xff00, 2)

static constexpr unsigned MIN_TOPIC_UPDATE_INTERVAL = 2500; // 2.5 ms -> 400 Hz

using namespace time_literals;

/**
 * The PX4IO class.
 *
 * Encapsulates PX4FMU to PX4IO communications modeled as file operations.
 */
class PX4IO : public cdev::CDev, public ModuleBase<PX4IO>, public OutputModuleInterface
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	PX4IO() = delete;

	explicit PX4IO(device::Device *interface);

	~PX4IO() override;

	/**
	 * Initialize the PX4IO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 */
	int init() override;

	/**
	 * IO Control handler.
	 *
	 * Handle all IOCTL calls to the PX4IO file descriptor.
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] cmd the IOCTL command
	 * @param[in] the IOCTL command parameter (optional)
	 */
	int ioctl(file *filp, int cmd, unsigned long arg) override;

	/**
	 * Print IO status.
	 *
	 * Print all relevant IO status information
	 *
	 */
	int print_status();

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

	static int task_spawn(int argc, char *argv[]);

	/*
	 * To test what happens if IO stops receiving updates from FMU.
	 *
	 * @param is_fail       true for failure condition, false for normal operation.
	 */
	void test_fmu_fail(bool is_fail) { _test_fmu_fail = is_fail; };

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			   unsigned num_control_groups_updated) override;

private:
	void Run() override;

	/**
	 * Respond to a vehicle command with an ACK message
	 *
	 * @param cmd           The command that was executed or denied (inbound)
	 * @param result        The command result
	 */
	void answer_command(const vehicle_command_s &cmd, uint8_t result);

	/**
	 * Handle issuing dsm bind ioctl to px4io.
	 *
	 * @param dsmMode       DSM2_BIND_PULSES, DSMX_BIND_PULSES, DSMX8_BIND_PULSES
	 */
	int dsm_bind_ioctl(int dsmMode);

	/**
	 * Update IO's arming-related state
	 */
	int io_set_arming_state();

	/**
	 * Fetch status and alarms from IO
	 *
	 * Also publishes battery voltage/current.
	 */
	int io_get_status();

	/**
	 * Fetch RC inputs from IO.
	 *
	 * @param input_rc      Input structure to populate.
	 * @return              OK if data was returned.
	 */
	int io_publish_raw_rc();

	/**
	 * write register(s)
	 *
	 * @param page          Register page to write to.
	 * @param offset        Register offset to start writing at.
	 * @param values        Pointer to array of values to write.
	 * @param num_values    The number of values to write.
	 * @return              OK if all values were successfully written.
	 */
	int io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);

	/**
	 * write a register
	 *
	 * @param page          Register page to write to.
	 * @param offset        Register offset to write to.
	 * @param value         Value to write.
	 * @return              OK if the value was written successfully.
	 */
	int io_reg_set(uint8_t page, uint8_t offset, const uint16_t value);

	/**
	 * read register(s)
	 *
	 * @param page          Register page to read from.
	 * @param offset        Register offset to start reading from.
	 * @param values        Pointer to array where values should be stored.
	 * @param num_values    The number of values to read.
	 * @return              OK if all values were successfully read.
	 */
	int io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	/**
	 * read a register
	 *
	 * @param page          Register page to read from.
	 * @param offset        Register offset to start reading from.
	 * @return              Register value that was read, or _io_reg_get_error on error.
	 */
	uint32_t io_reg_get(uint8_t page, uint8_t offset);

	/**
	 * modify a register
	 *
	 * @param page          Register page to modify.
	 * @param offset        Register offset to modify.
	 * @param clearbits     Bits to clear in the register.
	 * @param setbits       Bits to set in the register.
	 */
	int io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits);

	void update_params();

	void updateTimerRateGroups();

	static int checkcrc(int argc, char *argv[]);

	static int bind(int argc, char *argv[]);

	static constexpr int PX4IO_MAX_ACTUATORS = 8;

	static const uint32_t   _io_reg_get_error = 0x80000000;

	device::Device *const _interface;

	ButtonPublisher _button_publisher;
	MixingOutput _mixing_output{"PWM_MAIN", PX4IO_MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, true};

	hrt_abstime _last_status_publish{0};
	hrt_abstime _poll_last{0};

	orb_advert_t _mavlink_log_pub{nullptr}; ///< mavlink log pub

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _interface_read_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": interface read")};
	perf_counter_t _interface_write_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": interface write")};

	/* advertised topics */
	uORB::PublicationMulti<input_rc_s> _to_input_rc{ORB_ID(input_rc)};
	uORB::Publication<px4io_status_s>  _px4io_status_pub{ORB_ID(px4io_status)};

	uORB::Subscription _actuator_armed_sub{ORB_ID(actuator_armed)};   ///< system armed control topic
	uORB::Subscription _heater_status_sub{ORB_ID(heater_status)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)}; ///< vehicle command topic
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};   ///< vehicle status topic

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	float _analog_rc_rssi_volt{-1.f}; ///< analog RSSI voltage

	int32_t _thermal_control{-1};   ///< thermal control state

	unsigned _hardware{0};          ///< Hardware revision
	unsigned _max_actuators{0};     ///< Maximum # of actuators supported by PX4IO
	unsigned _max_rc_input{0};      ///< Maximum receiver channels supported by PX4IO
	unsigned _max_transfer{16};     ///< Maximum number of I2C transfers supported by PX4IO

	uint32_t _group_channels[PX4IO_P_SETUP_PWM_RATE_GROUP3 - PX4IO_P_SETUP_PWM_RATE_GROUP0 + 1] {};

	/* cached IO state */
	uint16_t _alarms{0};                    ///< Various IO alarms
	uint16_t _last_written_arming_s{0};     ///< the last written arming state reg
	uint16_t _last_written_arming_c{0};     ///< the last written arming state reg
	uint16_t _rc_valid_update_count{0};
	uint16_t _setup_arming{0};              ///< last arming setup state
	uint16_t _status{0};                    ///< Various IO status flags

	bool _analog_rc_rssi_stable{false};     ///< true when analog RSSI input is stable

	bool _first_param_update{true};
	bool _param_update_force{true};         ///< force a parameter update
	bool _previous_safety_off{false};

	bool _test_fmu_fail{false};             ///< To test what happens if IO loses FMU
	bool _timer_rates_configured{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::RC_RSSI_PWM_CHAN>) _param_rc_rssi_pwm_chan,
		(ParamInt<px4::params::RC_RSSI_PWM_MAX>)  _param_rc_rssi_pwm_max,
		(ParamInt<px4::params::RC_RSSI_PWM_MIN>)  _param_rc_rssi_pwm_min,
		(ParamInt<px4::params::SENS_EN_THERMAL>)  _param_sens_en_themal,
		(ParamInt<px4::params::SYS_HITL>)         _param_sys_hitl,
		(ParamInt<px4::params::SYS_USE_IO>)       _param_sys_use_io
	)
};
