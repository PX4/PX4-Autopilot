/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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

#include "TAP_ESC.hpp"

TAP_ESC::TAP_ESC(char const *const device, uint8_t channels_count):
	CDev(TAP_ESC_DEVICE_PATH),
	OutputModuleInterface(MODULE_NAME, px4::serial_port_to_wq(device)),
	_mixing_output{"TAP_ESC", channels_count, *this, MixingOutput::SchedulingPolicy::Auto, true},
	_channels_count(channels_count)
{
	strncpy(_device, device, sizeof(_device) - 1);
	_device[sizeof(_device) - 1] = '\0';  // Fix in case of overflow

	_mixing_output.setAllMinValues(RPMMIN);
	_mixing_output.setAllMaxValues(RPMMAX);
	_mixing_output.setAllDisarmedValues(RPMSTOPPED);
	_mixing_output.setAllFailsafeValues(RPMSTOPPED);
}

TAP_ESC::~TAP_ESC()
{
	tap_esc_common::deinitialise_uart(_uart_fd);
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

int TAP_ESC::init()
{
	int ret = tap_esc_common::initialise_uart(_device, _uart_fd);

	if (ret != 0) {
		PX4_ERR("failed to initialise UART.");
		return ret;
	}

	/* Respect boot time required by the ESC FW */
	hrt_abstime uptime_us = hrt_absolute_time();

	if (uptime_us < MAX_BOOT_TIME_MS * 1000) {
		usleep((MAX_BOOT_TIME_MS * 1000) - uptime_us);
	}

	/* Issue Basic Config */
	EscPacket packet{PACKET_HEAD, sizeof(ConfigInfoBasicRequest), ESCBUS_MSG_ID_CONFIG_BASIC};
	ConfigInfoBasicRequest &config = packet.d.reqConfigInfoBasic;
	memset(&config, 0, sizeof(ConfigInfoBasicRequest));
	config.maxChannelInUse = _channels_count;
	/* Enable closed-loop control if supported by the board */
	config.controlMode = BOARD_TAP_ESC_MODE;

	/* Asign the id's to the ESCs to match the mux */
	for (uint8_t phy_chan_index = 0; phy_chan_index < _channels_count; phy_chan_index++) {
		config.channelMapTable[phy_chan_index] = _device_mux_map[phy_chan_index] & ESC_MASK_MAP_CHANNEL;
		config.channelMapTable[phy_chan_index] |= (_device_dir_map[phy_chan_index] << 4) & ESC_MASK_MAP_RUNNING_DIRECTION;
	}

	config.maxChannelValue = RPMMAX;
	config.minChannelValue = RPMMIN;

	ret = tap_esc_common::send_packet(_uart_fd, packet, 0);

	if (ret < 0) {
		return ret;
	}

	/* set wait time for tap esc configurate and write flash (0.02696s measure by Saleae logic Analyzer) */
	usleep(30000);

	/* Verify All ESC got the config */
	for (uint8_t cid = 0; cid < _channels_count; cid++) {

		/* Send the InfoRequest querying  CONFIG_BASIC */
		EscPacket packet_info = {PACKET_HEAD, sizeof(InfoRequest), ESCBUS_MSG_ID_REQUEST_INFO};
		InfoRequest &info_req = packet_info.d.reqInfo;
		info_req.channelID = _device_mux_map[cid];;
		info_req.requestInfoType = REQUEST_INFO_BASIC;

		ret = tap_esc_common::send_packet(_uart_fd, packet_info, cid);

		if (ret < 0) {
			return ret;
		}

		/* Get a response */
		int retries = 10;
		bool valid = false;

		while (retries--) {
			tap_esc_common::read_data_from_uart(_uart_fd, &_uartbuf);

			if (tap_esc_common::parse_tap_esc_feedback(&_uartbuf, &_packet) == 0) {
				valid = (_packet.msg_id == ESCBUS_MSG_ID_CONFIG_INFO_BASIC
					 && _packet.d.rspConfigInfoBasic.channelID == _device_mux_map[cid]);
				break;

			} else {
				/* Give it time to come in */
				usleep(1000);
			}
		}

		if (!valid) {
			PX4_ERR("Verification of the configuration failed, ESC number: %d", cid);
			return -EIO;
		}
	}

	/* To Unlock the ESC from the Power up state we need to issue 10
	 * ESCBUS_MSG_ID_RUN request with all the values 0;
	 */
	EscPacket unlock_packet{PACKET_HEAD, _channels_count, ESCBUS_MSG_ID_RUN};
	unlock_packet.len *= sizeof(unlock_packet.d.reqRun.rpm_flags[0]);
	memset(unlock_packet.d.bytes, 0, sizeof(unlock_packet.d.bytes));

	int unlock_times = 10;

	while (unlock_times--) {
		tap_esc_common::send_packet(_uart_fd, unlock_packet, -1);

		/* Min Packet to Packet time is 1 Ms so use 2 */
		usleep(2000);
	}

	/* do regular cdev init */
	return CDev::init();
}

void TAP_ESC::send_esc_outputs(const uint16_t *pwm, const uint8_t motor_cnt)
{
	uint16_t rpm[TAP_ESC_MAX_MOTOR_NUM] {};
	_led_controller.update(_led_control_data);

	for (uint8_t i = 0; i < motor_cnt; i++) {
		rpm[i] = pwm[i];

		if ((rpm[i] & RUN_CHANNEL_VALUE_MASK) > RPMMAX) {
			rpm[i] = (rpm[i] & ~RUN_CHANNEL_VALUE_MASK) | RPMMAX;

		} else if ((rpm[i] & RUN_CHANNEL_VALUE_MASK) < RPMSTOPPED) {
			rpm[i] = (rpm[i] & ~RUN_CHANNEL_VALUE_MASK) | RPMSTOPPED;
		}

		// apply the led color
		if (i < BOARD_MAX_LEDS) {
			switch (_led_control_data.leds[i].color) {
			case led_control_s::COLOR_RED:
				rpm[i] |= RUN_RED_LED_ON_MASK;
				break;

			case led_control_s::COLOR_GREEN:
				rpm[i] |= RUN_GREEN_LED_ON_MASK;
				break;

			case led_control_s::COLOR_BLUE:
				rpm[i] |= RUN_BLUE_LED_ON_MASK;
				break;

			case led_control_s::COLOR_AMBER: //make it the same as yellow
			case led_control_s::COLOR_YELLOW:
				rpm[i] |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK;
				break;

			case led_control_s::COLOR_PURPLE:
				rpm[i] |= RUN_RED_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
				break;

			case led_control_s::COLOR_CYAN:
				rpm[i] |= RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
				break;

			case led_control_s::COLOR_WHITE:
				rpm[i] |= RUN_RED_LED_ON_MASK | RUN_GREEN_LED_ON_MASK | RUN_BLUE_LED_ON_MASK;
				break;

			default: // led_control_s::COLOR_OFF
				break;
			}
		}
	}

	rpm[_responding_esc] |= RUN_FEEDBACK_ENABLE_MASK;

	EscPacket packet{PACKET_HEAD, _channels_count, ESCBUS_MSG_ID_RUN};
	packet.len *= sizeof(packet.d.reqRun.rpm_flags[0]);

	for (uint8_t i = 0; i < _channels_count; i++) {
		packet.d.reqRun.rpm_flags[i] = rpm[i];
	}

	int ret = tap_esc_common::send_packet(_uart_fd, packet, _responding_esc);

	if (++_responding_esc >= _channels_count) {
		_responding_esc = 0;
	}

	if (ret < 1) {
		PX4_WARN("TX ERROR: ret: %d, errno: %d", ret, errno);
	}
}

void TAP_ESC::send_tune_packet(EscbusTunePacket &tune_packet)
{
	PX4_DEBUG("send_tune_packet: Frequency: %d Hz Duration %d ms, strength: %d", tune_packet.frequency,
		  tune_packet.duration_ms, tune_packet.strength);

	EscPacket buzzer_packet{PACKET_HEAD, sizeof(EscbusTunePacket), ESCBUS_MSG_ID_TUNE};
	buzzer_packet.d.tunePacket = tune_packet;
	tap_esc_common::send_packet(_uart_fd, buzzer_packet, -1);
}

bool TAP_ESC::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS], unsigned num_outputs,
			    unsigned num_control_groups_updated)
{
	if (_initialized) {
		uint16_t motor_out[TAP_ESC_MAX_MOTOR_NUM] {};

		// We need to remap from the system default to what PX4's normal scheme is
		switch (num_outputs) {
		case 4:
			motor_out[0] = outputs[2];
			motor_out[1] = outputs[1];
			motor_out[2] = outputs[3];
			motor_out[3] = outputs[0];
			break;

		case 6:
			motor_out[0] = outputs[3];
			motor_out[1] = outputs[0];
			motor_out[2] = outputs[4];
			motor_out[3] = outputs[2];
			motor_out[4] = outputs[1];
			motor_out[5] = outputs[5];
			break;

		default:

			// Use the system defaults
			for (uint8_t i = 0; i < num_outputs; ++i) {
				motor_out[i] = outputs[i];
			}

			break;
		}

		// Set remaining motors to RPMSTOPPED to be on the safe side
		for (uint8_t i = num_outputs; i < TAP_ESC_MAX_MOTOR_NUM; ++i) {
			motor_out[i] = RPMSTOPPED;
		}

		send_esc_outputs(motor_out, num_outputs);

		tap_esc_common::read_data_from_uart(_uart_fd, &_uartbuf);

		if (!tap_esc_common::parse_tap_esc_feedback(&_uartbuf, &_packet)) {
			if (_packet.msg_id == ESCBUS_MSG_ID_RUN_INFO) {
				RunInfoRepsonse &feed_back_data = _packet.d.rspRunInfo;

				if (feed_back_data.channelID < esc_status_s::CONNECTED_ESC_MAX) {
					_esc_feedback.esc[feed_back_data.channelID].timestamp = hrt_absolute_time();
					_esc_feedback.esc[feed_back_data.channelID].esc_errorcount = 0;
					_esc_feedback.esc[feed_back_data.channelID].esc_rpm = feed_back_data.speed;
#if defined(ESC_HAVE_VOLTAGE_SENSOR)
					_esc_feedback.esc[feed_back_data.channelID].esc_voltage = feed_back_data.voltage;
#endif // ESC_HAVE_VOLTAGE_SENSOR
#if defined(ESC_HAVE_CURRENT_SENSOR)
					_esc_feedback.esc[feed_back_data.channelID].esc_current = feed_back_data.current;
#endif // ESC_HAVE_CURRENT_SENSOR
#if defined(ESC_HAVE_TEMPERATURE_SENSOR)
					_esc_feedback.esc[feed_back_data.channelID].esc_temperature = static_cast<float>(feed_back_data.temperature);
#endif // ESC_HAVE_TEMPERATURE_SENSOR
					_esc_feedback.esc[feed_back_data.channelID].esc_state = feed_back_data.ESCStatus;
					_esc_feedback.esc[feed_back_data.channelID].failures = 0;
					//_esc_feedback.esc[feed_back_data.channelID].esc_setpoint_raw = motor_out[feed_back_data.channelID];
					//_esc_feedback.esc[feed_back_data.channelID].esc_setpoint = (float)motor_out[feed_back_data.channelID] * 15.13f - 16171.4f;

					_esc_feedback.counter++;
					_esc_feedback.esc_count = num_outputs;
					_esc_feedback.esc_connectiontype = esc_status_s::ESC_CONNECTION_TYPE_SERIAL;

					_esc_feedback.esc_online_flags |= 1 << feed_back_data.channelID;
					_esc_feedback.esc_armed_flags  |= 1 << feed_back_data.channelID;

					_esc_feedback.timestamp = hrt_absolute_time();
					_esc_feedback_pub.publish(_esc_feedback);
				}
			}
		}

		return true;
	}

	return false;
}

void TAP_ESC::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		exit_and_cleanup();
		return;
	}

	// push backup schedule
	ScheduleDelayed(20_ms);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	if (!_initialized) {
		if (init() == PX4_OK) {
			_initialized = true;

		} else {
			PX4_ERR("init failed");
			exit_and_cleanup();
		}

	} else {
		_mixing_output.update();

		// Handle tunes
		if (_tune_control_sub.updated()) {
			tune_control_s tune;

			if (_tune_control_sub.copy(&tune)) {
				if (tune.timestamp > 0) {
					Tunes::ControlResult result = _tunes.set_control(tune);
					PX4_DEBUG("new tune id: %d, result: %d, duration: %lu", tune.tune_id, (int)result, tune.duration);
				}
			}
		}

		const hrt_abstime timestamp_now = hrt_absolute_time();

		if ((timestamp_now - _interval_timestamp > _duration)) {
			_interval_timestamp = timestamp_now;

			if (_silence_length > 0) {
				_duration = _silence_length;
				_silence_length = 0;

			} else {
				uint8_t strength = 0;
				Tunes::Status parse_ret_val = _tunes.get_next_note(_frequency, _duration, _silence_length, strength);

				if (parse_ret_val == Tunes::Status::Continue) {
					// Continue playing.
					if (_frequency > 0) {
						// Start playing the note.
						EscbusTunePacket esc_tune_packet{};
						esc_tune_packet.frequency = _frequency;
						esc_tune_packet.duration_ms = (uint16_t)(_duration / 1000); // convert to ms
						esc_tune_packet.strength = strength;
						send_tune_packet(esc_tune_packet);
					}

				} else {
					_silence_length = 0;
				}
			}
		}
	}

	// check at end of cycle (updateSubscriptions() can potentially change to a different WorkQueue thread)
	_mixing_output.updateSubscriptions(true, true);

	perf_end(_cycle_perf);
}

int TAP_ESC::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	int ret = OK;

	lock();

	switch (cmd) {
	case MIXERIOCRESET:
		_mixing_output.resetMixerThreadSafe();
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			unsigned buflen = strlen(buf);
			ret = _mixing_output.loadMixerThreadSafe(buf, buflen);
			break;
		}


	default:
		ret = -ENOTTY;
		break;
	}

	unlock();

	return ret;
}

int TAP_ESC::task_spawn(int argc, char *argv[])
{
	/* Parse arguments */
	const char *device = nullptr;
	uint8_t channels_count = 0;

	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	if (argc < 2) {
		print_usage("not enough arguments");
		return -1;
	}

	while ((ch = px4_getopt(argc, argv, "d:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'n':
			channels_count = atoi(myoptarg);
			break;
		}
	}

	// Sanity check on arguments
	if (channels_count == 0) {
		print_usage("Channel count is invalid (0)");
		return -1;
	}

	if (device == nullptr || strlen(device) == 0) {
		print_usage("no device specified");
		return -1;
	}

	TAP_ESC *instance = new TAP_ESC(device, channels_count);

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;
	instance->ScheduleNow();
	return 0;
}

int TAP_ESC::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TAP_ESC::print_status()
{
	_mixing_output.printStatus();
	return 0;
}

int TAP_ESC::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module controls the TAP_ESC hardware via UART. It listens on the
actuator_controls topics, does the mixing and writes the PWM outputs.

### Implementation
Currently the module is implementd as a threaded version only, meaning that it
runs in its own thread instead of on the work queue.

### Example
The module is typically started with:
tap_esc start -d /dev/ttyS2 -n <1-8>

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("tap_esc", "driver");

	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the task");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "Device used to talk to ESCs", true);
	PRINT_MODULE_USAGE_PARAM_INT('n', 4, 0, 8, "Number of ESCs", true);
	return PX4_OK;
}

extern "C" __EXPORT int tap_esc_main(int argc, char *argv[])
{
	return TAP_ESC::main(argc, argv);
}
