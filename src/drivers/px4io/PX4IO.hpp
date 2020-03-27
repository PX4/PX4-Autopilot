/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
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
 * @file PX4IO.hpp
 * Driver for the PX4IO board.
 *
 * PX4IO is connected via DMA enabled high-speed UART.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/sem.hpp>

#include <crc32.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_sbus.h>
#include <lib/circuit_breaker/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer/MixerGroup.hpp>
#include <lib/mixer/MultirotorMixer/MultirotorMixer.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <lib/rc/dsm.h>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/servorail_status.h>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>

#include <debug.h>

#include <modules/px4iofirmware/protocol.h>

#include "uploader.h"

#include "modules/dataman/dataman.h"

#include "px4io_driver.h"

#define PX4IO_SET_DEBUG			_IOC(0xff00, 0)
#define PX4IO_INAIR_RESTART_ENABLE	_IOC(0xff00, 1)
#define PX4IO_REBOOT_BOOTLOADER		_IOC(0xff00, 2)
#define PX4IO_CHECK_CRC			_IOC(0xff00, 3)

using namespace time_literals;

/**
 * The PX4IO class.
 *
 * Encapsulates PX4FMU to PX4IO communications modeled as file operations.
 */
class PX4IO : public cdev::CDev, public px4::ScheduledWorkItem
{
public:
	/**
	 * Constructor.
	 *
	 * Initialize all class variables.
	 */
	PX4IO() = delete;
	explicit PX4IO(device::Device *interface);

	/**
	 * Destructor.
	 *
	 * Wait for worker thread to terminate.
	 */
	~PX4IO() override;

	/**
	 * Initialize the PX4IO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 */
	int		init() override;

	/**
	 * Initialize the PX4IO class.
	 *
	 * Retrieve relevant initial system parameters. Initialize PX4IO registers.
	 *
	 * @param disable_rc_handling set to true to forbid override / RC handling on IO
	 * @param hitl_mode set to suppress publication of actuator_outputs - instead defer to pwm_out_sim
	 */
	int			init(bool disable_rc_handling, bool hitl_mode);

	/**
	 * Detect if a PX4IO is connected.
	 *
	 * Only validate if there is a PX4IO to talk to.
	 */
	virtual int		detect();

	/**
	 * IO Control handler.
	 *
	 * Handle all IOCTL calls to the PX4IO file descriptor.
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] cmd the IOCTL command
	 * @param[in] the IOCTL command parameter (optional)
	 */
	virtual int		ioctl(file *filp, int cmd, unsigned long arg);

	/**
	 * Disable RC input handling
	 */
	int			disable_rc_handling();

	/**
	 * Print IO status.
	 *
	 * Print all relevant IO status information
	 *
	 * @param extended_status Shows more verbose information (in particular RC config)
	 */
	void			print_status(bool extended_status);

	/**
	 * Fetch and print debug console output.
	 */
	int			print_debug();

	/*
	 * To test what happens if IO stops receiving updates from FMU.
	 *
	 * @param is_fail	true for failure condition, false for normal operation.
	 */
	void			test_fmu_fail(bool is_fail) { _test_fmu_fail = is_fail; };

	inline uint16_t		system_status() const { return _status; }

private:
	void Run() override;

	device::Device		*_interface{nullptr};

	unsigned		_hardware{0};		///< Hardware revision
	unsigned		_max_actuators{0};		///< Maximum # of actuators supported by PX4IO
	unsigned		_max_controls{0};		///< Maximum # of controls supported by PX4IO
	unsigned		_max_rc_input{0};		///< Maximum receiver channels supported by PX4IO
	unsigned		_max_relays{0};		///< Maximum relays supported by PX4IO
	unsigned		_max_transfer{16};		///< Maximum number of I2C transfers supported by PX4IO

	bool			_rc_handling_disabled{false};	///< If set, IO does not evaluate, but only forward the RC values
	unsigned		_rc_chan_count{0};		///< Internal copy of the last seen number of RC channels
	uint64_t		_rc_last_valid{0};		///< last valid timestamp

	volatile int		_task{-1};			///< worker task id
	volatile bool		_task_should_exit{false};	///< worker terminate flag

	hrt_abstime		_poll_last{0};
	hrt_abstime		_orb_check_last{0};

	orb_advert_t		_mavlink_log_pub{nullptr};	///< mavlink log pub

	perf_counter_t		_perf_update{perf_alloc(PC_ELAPSED, MODULE_NAME": update")};		///< local performance counter for status updates
	perf_counter_t		_perf_write{perf_alloc(PC_ELAPSED, MODULE_NAME": write")};		///< local performance counter for PWM control writes
	perf_counter_t		_perf_sample_latency{perf_alloc(PC_ELAPSED, MODULE_NAME": control latency")};	///< total system latency (based on passed-through timestamp)

	/* cached IO state */
	uint16_t		_status{0};		///< Various IO status flags
	uint16_t		_alarms{0};		///< Various IO alarms
	uint16_t		_last_written_arming_s{0};	///< the last written arming state reg
	uint16_t		_last_written_arming_c{0};	///< the last written arming state reg

	/* subscribed topics */
	uORB::SubscriptionCallbackWorkItem _t_actuator_controls_0{this, ORB_ID(actuator_controls_0)};

	uORB::Subscription	_t_actuator_controls_1{ORB_ID(actuator_controls_1)};	///< actuator controls group 1 topic
	uORB::Subscription	_t_actuator_controls_2{ORB_ID(actuator_controls_2)};;	///< actuator controls group 2 topic
	uORB::Subscription	_t_actuator_controls_3{ORB_ID(actuator_controls_3)};;	///< actuator controls group 3 topic
	uORB::Subscription	_t_actuator_armed{ORB_ID(actuator_armed)};		///< system armed control topic
	uORB::Subscription 	_t_vehicle_control_mode{ORB_ID(vehicle_control_mode)};	///< vehicle control mode topic
	uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};	///< parameter update topic
	uORB::Subscription	_t_vehicle_command{ORB_ID(vehicle_command)};		///< vehicle command topic

	bool			_param_update_force{true};	///< force a parameter update

	/* advertised topics */
	uORB::PublicationMulti<input_rc_s>			_to_input_rc{ORB_ID(input_rc)};
	uORB::PublicationMulti<actuator_outputs_s>		_to_outputs{ORB_ID(actuator_outputs)};
	uORB::PublicationMulti<multirotor_motor_limits_s>	_to_mixer_status{ORB_ID(multirotor_motor_limits)};
	uORB::Publication<servorail_status_s>			_to_servorail{ORB_ID(servorail_status)};
	uORB::Publication<safety_s>				_to_safety{ORB_ID(safety)};

	bool			_primary_pwm_device{false};	///< true if we are the default PWM output
	bool			_lockdown_override{false};	///< allow to override the safety lockdown
	bool			_armed{false};			///< wether the system is armed
	bool			_override_available{false};	///< true if manual reversion mode is enabled

	bool			_cb_flighttermination{true};	///< true if the flight termination circuit breaker is enabled
	bool 			_in_esc_calibration_mode{false};	///< do not send control outputs to IO (used for esc calibration)

	int32_t			_rssi_pwm_chan{0}; ///< RSSI PWM input channel
	int32_t			_rssi_pwm_max{0}; ///< max RSSI input on PWM channel
	int32_t			_rssi_pwm_min{0}; ///< min RSSI input on PWM channel
	int32_t			_thermal_control{-1}; ///< thermal control state
	bool			_analog_rc_rssi_stable{false}; ///< true when analog RSSI input is stable
	float			_analog_rc_rssi_volt{-1.f}; ///< analog RSSI voltage

	bool			_test_fmu_fail{false}; ///< To test what happens if IO loses FMU

	struct MotorTest {
		uORB::Subscription test_motor_sub{ORB_ID(test_motor)};
		bool in_test_mode{false};
		hrt_abstime timeout{0};
	};
	MotorTest _motor_test;
	bool                    _hitl_mode{false};     ///< Hardware-in-the-loop simulation mode - don't publish actuator_outputs

	/**
	 * Send controls for one group to IO
	 */
	int			io_set_control_state(unsigned group);

	/**
	 * Send all controls to IO
	 */
	int			io_set_control_groups();

	/**
	 * Update IO's arming-related state
	 */
	int			io_set_arming_state();

	/**
	 * Push RC channel configuration to IO.
	 */
	int			io_set_rc_config();

	/**
	 * Fetch status and alarms from IO
	 *
	 * Also publishes battery voltage/current.
	 */
	int			io_get_status();

	/**
	 * Disable RC input handling
	 */
	int			io_disable_rc_handling();

	/**
	 * Fetch RC inputs from IO.
	 *
	 * @param input_rc	Input structure to populate.
	 * @return		OK if data was returned.
	 */
	int			io_get_raw_rc_input(input_rc_s &input_rc);

	/**
	 * Fetch and publish raw RC input data.
	 */
	int			io_publish_raw_rc();

	/**
	 * Fetch and publish the PWM servo outputs.
	 */
	int			io_publish_pwm_outputs();

	/**
	 * write register(s)
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to start writing at.
	 * @param values	Pointer to array of values to write.
	 * @param num_values	The number of values to write.
	 * @return		OK if all values were successfully written.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);

	/**
	 * write a register
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to write to.
	 * @param value		Value to write.
	 * @return		OK if the value was written successfully.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t value);

	/**
	 * read register(s)
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @param values	Pointer to array where values should be stored.
	 * @param num_values	The number of values to read.
	 * @return		OK if all values were successfully read.
	 */
	int			io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	/**
	 * read a register
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @return		Register value that was read, or _io_reg_get_error on error.
	 */
	uint32_t		io_reg_get(uint8_t page, uint8_t offset);
	static const uint32_t	_io_reg_get_error = 0x80000000;

	/**
	 * modify a register
	 *
	 * @param page		Register page to modify.
	 * @param offset	Register offset to modify.
	 * @param clearbits	Bits to clear in the register.
	 * @param setbits	Bits to set in the register.
	 */
	int			io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits);

	/**
	 * Send mixer definition text to IO
	 */
	int			mixer_send(const char *buf, unsigned buflen, unsigned retries = 3);

	/**
	 * Handle a status update from IO.
	 *
	 * Publish IO status information if necessary.
	 *
	 * @param status	The status register
	 */
	int			io_handle_status(uint16_t status);

	/**
	 * Handle an alarm update from IO.
	 *
	 * Publish IO alarm information if necessary.
	 *
	 * @param alarm		The status register
	 */
	int			io_handle_alarms(uint16_t alarms);

	/**
	 * Handle issuing dsm bind ioctl to px4io.
	 *
	 * @param dsmMode	0:dsm2, 1:dsmx
	 */
	void			dsm_bind_ioctl(int dsmMode);

	/**
	 * Handle a servorail update from IO.
	 *
	 * Publish servo rail information if necessary.
	 *
	 * @param vservo	vservo register
	 * @param vrssi 	vrssi register
	 */
	void			io_handle_vservo(uint16_t vservo, uint16_t vrssi);

	/**
	 * check and handle test_motor topic updates
	 */
	void handle_motor_test();
};
