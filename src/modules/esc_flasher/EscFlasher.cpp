#include "EscFlasher.hpp"

/*
 * The ESC Flasher module will allow configuring of some ESC features, over DSHOT.
 * It will enable ESC reflashing for AM32 using bit-banging UART 19200 on the signal line.
*/

#include "AM32Firmware.h"
#include <uORB/topics/led_control.h>

using namespace time_literals;

#define ESC_FLASHER_RET_OK    (0)
#define ESC_FLASHER_RET_NOK   (-1)

static const uint32_t SECONDS_TO_MICROSECONDS = 1000000;

ESC_Flasher::ESC_Flasher() : ModuleParams(nullptr), _loop_perf(perf_alloc(PC_ELAPSED, "esc_flasher")) {
	// New instance created, set all initial values
	subscribe_orb_messages();

	run_calls = 0;
	print_flag = false;
	time_now = 0;
	time_1s = 0;
	time_flash_start = 0;
	time_delayed_start = 0;

	start_flash = 0;
	motor_flags = 0;
	cancel_flash = 0;

	// Get ESC type, needs to be set via param ESC_TYPE
	_esc_type = (ESCType)_param_esc_type.get();
	set_esc_type(true);

	// Get ESC update param at boot, check if it is pending
	_esc_update = (ESCUpdate)_param_esc_update.get();
	if (_esc_update != ESCUpdate::Disabled) {
		// Don't allow update at boot, must be set some time after startup
		_param_esc_update.reset();
		_esc_update = ESCUpdate::Disabled;
		PX4_ERR("Cannot update ESCs at boot, try again after startup");
	}

	esc_update_state = 0;
	prev_esc_update_state = 0;
	flashing_result = 0;
	memset(esc_update_motors, 0, sizeof(esc_update_motors));
	memset(&esc_flashing_state, 0, sizeof(esc_flashing_state));

	memset(&esc_versions, 0, sizeof(esc_versions));
	memset(&esc_gpios, 0, sizeof(esc_gpios));
	gpio_high = 0;
	time_gpio_set = 0;

	update_params();

	msg_id = 0;

	load_firmware_source();

	// Set to run at 1 Hz
	run_delay = run_delay_max;
	wake = false;
}

ESC_Flasher::~ESC_Flasher() {
	// Instance deleted, clean up
	unsubscribe_orb_messages();
	perf_free(_loop_perf);
}

void ESC_Flasher::load_firmware_source(void) {
	esc_flasher_status_s status{0};
	// Default to mem_array
	_am32_fw_source = AM32_FIRMWARE_SOURCE::MEM_ARRAY;
	_am32_fw_ver_major = 0;
	_am32_fw_ver_minor = 0;

	// TODO option for changing source via parameter, looking for firmware from that source

	switch (_am32_fw_source) {
		case AM32_FIRMWARE_SOURCE::MEM_ARRAY:
#ifdef INCLUDE_AM32_FIRMWARE
			_am32_fw_ver_major = am32_fw_version_major;
			_am32_fw_ver_minor = am32_fw_version_minor;

			// This is temporary so compile includes the array
			if (sizeof(am32_firmware) > 1234) {
				PX4_DEBUG("Sample byte from am32_firmware[1234] %d", (int)am32_firmware[1234]);
			}

			// Publish status with included firmware version
			status.esc_flashing_in_progress = false;
			status.version_major = _am32_fw_ver_major;
			status.version_minor = _am32_fw_ver_minor;
			status.timestamp = hrt_absolute_time();
			_esc_flasher_status_pub.publish(status);
#endif
			break;
		case AM32_FIRMWARE_SOURCE::SD_CARD:
			_am32_fw_ver_major = 0;
			_am32_fw_ver_minor = 0;
			break;
		case AM32_FIRMWARE_SOURCE::STREAM_HEAP:
			_am32_fw_ver_major = 0;
			_am32_fw_ver_minor = 0;
			break;
	}
}

void ESC_Flasher::set_esc_type(bool print) {
	if (_esc_type == ESCType::Unknown) {
		if (print) PX4_INFO("ESC Flasher starting, ESC Type is DEFAULT");
	}
	else if (_esc_type == ESCType::AM32) {
		if (print) PX4_INFO("ESC Flasher starting, ESC Type is AM32");
	}
	else if (_esc_type == ESCType::BLHELI32) {
		if (print) PX4_INFO("ESC Flasher starting, ESC Type is BLHeli32");
	}
	else if (_esc_type == ESCType::AM32) {
		if (print) PX4_INFO("ESC Flasher starting, ESC Type is AM32_Old");
	}
	else if (_esc_type == ESCType::BlueJay) {
		if (print) PX4_INFO("ESC Flasher starting, ESC Type is BlueJay");
	}
	else {
		_esc_type = ESCType::Unknown;
		_param_esc_type.reset();
		PX4_ERR("Invalid ESC Type, parameter ESC_TYPE has been reset");
	}
}

void ESC_Flasher::update_params(void) {
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();

		// Get ESC type, needs to be set via param ESC_TYPE
		_esc_type = (ESCType)_param_esc_type.get();
		set_esc_type(false);

		ESCUpdate esc_update = (ESCUpdate)_param_esc_update.get();
		if (esc_update != ESCUpdate::Disabled && esc_update != ESCUpdate::Recover2) {
			if (_esc_type == ESCType::AM32) {
				// Start the update process
				_esc_update = esc_update;
				// Flag all motors for update
				memset(esc_update_motors, 1, sizeof(esc_update_motors));
				// Clear param ESC_UPDATE
				_param_esc_update.reset();

				// Start flashing state machine
				esc_update_state = 1;
				prev_esc_update_state = 0;

				// Set a 2 second delay before bit-banging
				time_delayed_start = hrt_absolute_time() + 2_s;

				if (esc_update == ESCUpdate::Recover) {
					PX4_INFO("Starting recovery on motors 1 2 3 4");
				}
				else {
					PX4_INFO("Starting flash on motors 1 2 3 4");
				}
			}
			else {
				// ESC Type not supported
				_esc_update = ESCUpdate::Disabled;
				_param_esc_update.reset();

				PX4_ERR("ESC type %d not supported by esc_flasher", (int)_esc_type);
			}
		}
		else if (esc_update == ESCUpdate::Recover2) {
			if (_esc_type == ESCType::AM32) {
				if (_esc_update == ESCUpdate::Recover) {
					// Proceed with the recovery, battery should be in by now
					_esc_update = esc_update;
					esc_update_state = 31;
					PX4_INFO("Proceeding with recovery step 2");
				}
			}
			else {
				// ESC Type not supported
				_esc_update = ESCUpdate::Disabled;
				_param_esc_update.reset();

				PX4_ERR("ESC type %d not supported by esc_flasher", (int)_esc_type);
			}
		}
	}
}

void ESC_Flasher::play_tune(int tune) {
	tune_control_s tune_control{0};
	if (tune == 1) {
		tune_control.tune_id = tune_control_s::TUNE_ID_PROG_PX4IO; // PROG_PX4IO
	}
	else if (tune == 2) {
		tune_control.tune_id = tune_control_s::TUNE_ID_PROG_PX4IO_OK; // PROG_PX4IO_OK
	}
	else if (tune == 3) {
		tune_control.tune_id = tune_control_s::TUNE_ID_PROG_PX4IO_ERR; // PROG_PX4IO_ERR
	}
	tune_control.volume = tune_control_s::VOLUME_LEVEL_MAX;
	tune_control.tune_override = true;
	tune_control.timestamp = hrt_absolute_time();
	_tune_control_pub.publish(tune_control);
}

void ESC_Flasher::stop_tune(void) {
	tune_control_s tune_control{0};
	tune_control.tune_override = true;
	tune_control.timestamp = hrt_absolute_time();
	_tune_control_pub.publish(tune_control);
}

void ESC_Flasher::subscribe_orb_messages(void) {
	_sensor_gps_sub.subscribe();
	_vehicle_status_sub.subscribe();
}

void ESC_Flasher::unsubscribe_orb_messages(void) {
	_sensor_gps_sub.unsubscribe();
	_vehicle_status_sub.unsubscribe();
}

void ESC_Flasher::run() {
	while (!should_exit()) {
		perf_begin(_loop_perf);

		update_params();
		RunLoop();

		perf_end(_loop_perf);

		// Sleep so our task can yield to other tasks
		//px4_usleep(run_delay);
		delay_loops = run_delay / run_delay_min;
		while (!wake && delay_loops--) {
			px4_usleep(run_delay_min);
		}
		wake = false;
	}
}

// ESC Flasher work loop
void ESC_Flasher::RunLoop() {
	static esc_flasher_request_s request;
	static esc_flasher_request_ack_s update;
	static esc_flasher_status_s status;
	static uint32_t gpio;
	int retval = 0;

	run_calls++;

	time_now = hrt_absolute_time();

	// Get vehicle armed state
	vehicle_armed = get_current_arming_state();

	// Check update flags from command 'esc_flasher flash'
	if (start_flash) {
		if (_esc_type == ESCType::AM32) {
			if (_esc_update == ESCUpdate::Disabled) {
				if (motor_flags) {
					_esc_update = ESCUpdate::ForceUpdate;
					esc_update_state = 1;
					prev_esc_update_state = 0;
					// Start immediately, no need to wait for param response
					time_delayed_start = time_now;
					// Set motor update flags
					memset(esc_update_motors, 0, sizeof(esc_update_motors));
					char motors_string[16] = "1 2 3 4";
					int motors_index = 0;
					for (uint32_t i = 0; i < ESC_COUNT; i++) {
						esc_update_motors[i] = (motor_flags & (1 << i));
						if (esc_update_motors[i]) {
							motors_string[motors_index * 2] = (0x30 + i + 1);
							motors_index++;
						}
					}
					// Terminate string
					motors_string[(motors_index * 2) - 1] = 0;

					if (motors_index == 1) PX4_INFO("Starting flash on motor %s", motors_string);
					else PX4_INFO("Starting flash on motors %s", motors_string);
				}
				else {
					PX4_ERR("Invalid motor flags for flashing");
				}
			}
			else {
				PX4_ERR("ESC flashing already in progress");
			}
		}
		else {
			// ESC Type not supported
			_esc_update = ESCUpdate::Disabled;

			PX4_ERR("ESC type %d not supported by esc_flasher", (int)_esc_type);
		}

		start_flash = 0;
		motor_flags = 0;
	}

	if (cancel_flash) {
		// Send request to DShot to cancel any pending requests
		_esc_update = ESCUpdate::Cancel;
		esc_update_state = 96;
		prev_esc_update_state = 0;

		cancel_flash = 0;
	}

	// Check update parameter
	if (_esc_update != ESCUpdate::Disabled) {
		if (_esc_update != ESCUpdate::Cancel && vehicle_armed != vehicle_status_s::ARMING_STATE_DISARMED) {
			PX4_ERR("Cannot update ESCs while vehicle is armed!");
			_esc_update = ESCUpdate::Disabled;
		}
		else {
			switch (esc_update_state) {
			case 0:
				// Idle state

				break;
			case 1:
				// Set loop to run fast 200 Hz
				run_delay = run_delay_min;

				if (time_now >= time_delayed_start) {
					PX4_INFO("ESC Flasher starting update");
					esc_update_state = 2;
					time_flash_start = time_now;
					flashing_result = 0;

					// Play start flashing tune
					esc_flasher_tune = 1;

					// RECOVERY CASE
					if (_esc_update == ESCUpdate::Recover) {
						// Skip to state 4 disable DShot
						esc_update_state = 4;
					}
				}

				break;
			case 2:
				// Send ESC_INFO request to DShot
				// First, read any pending ack
				_esc_flasher_request_ack_sub.copy(&update);

				// Send ESC_INFO request to DShot driver
				memset(&request, 0, sizeof(request));
				request.timestamp = hrt_absolute_time();
				request.msg_id = ++msg_id;
				request.request = esc_flasher_request_s::REQUEST_ESC_INFO;
				for (uint32_t i = 0; i < ESC_COUNT; i++) {
					if (esc_update_motors[i]) request.motor_flags |= (1 << i);
				}
				_esc_flasher_request_pub.publish(request);

				esc_info_timeout = 0;
				esc_update_state = 3;

				break;
			case 3:
				// Wait for response from DShot driver to REQUEST_ESC_INFO request
				if (_esc_flasher_request_ack_sub.updated()) {
					esc_flasher_request_ack_s response;
					_esc_flasher_request_ack_sub.copy(&response);

					// Save/print response
					if (response.msg_id == msg_id && response.request == esc_flasher_request_ack_s::REQUEST_ESC_INFO &&
						response.result == esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_ACCEPTED) {
						for (uint32_t i = 0; i < ESC_COUNT; i++) {
							if (response.fw_flags & (1 << i)) {
								esc_versions[i].major = response.fw_major[i];
								esc_versions[i].minor = response.fw_minor[i];
								PX4_INFO("ESC_INFO request filled, ESC %d firmware version: %d.%d",
									(int)i, (int)esc_versions[i].major, (int)esc_versions[i].minor);
							}
						}
					}
					else if (response.msg_id == msg_id && response.request == esc_flasher_request_ack_s::REQUEST_ESC_INFO &&
						response.result == esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_UNSUPPORTED) {
						// Unsupported ESC detected
						PX4_ERR("Unsupported ESC detected, cancelling ESC flasher");
						prev_esc_update_state = esc_update_state;
						esc_update_state = 99;
						flashing_result = 9;
					}
					else {
						// Mismatch somewhere
						PX4_ERR("Response from DShot mismatch! Response: %d, msg_id: %d, response msg_id: %d",
							(int)response.result, (int)msg_id, (int)response.msg_id);
						prev_esc_update_state = esc_update_state;
						esc_update_state = 99;
						flashing_result = 10;

						break;
					}

					// If we are in mode ESCUpdate::Update, only flash motors that aren't on the correct fw version
					// Mode ESCUpdate:ForceUpdate should flash the new firmware always
					if (_esc_update == ESCUpdate::Update) {
						// If firmware versions match, clear the update flag for that motor
						for (uint32_t i = 0; i < ESC_COUNT; i++) {
							if (esc_update_motors[i] && (esc_versions[i].major == _am32_fw_ver_major) &&
								(esc_versions[i].minor == _am32_fw_ver_minor)) {
									esc_update_motors[i] = 0;
							}
						}
					}

					// Check that we still have ESCs to flash, if not then cancel state machine
					bool flash_required = false;
					for (uint32_t i = 0; i < ESC_COUNT; i++) {
						if (esc_update_motors[i]) {
							flash_required = true;
							break;
						}
					}
					if (!flash_required) {
						// No ESCs need flashing, go to completion state
						prev_esc_update_state = esc_update_state;
						esc_update_state = 20;
						flashing_result = 1;

						break;
					}

					esc_update_state = 4;
				}
				else {
					esc_info_timeout++;
					if (esc_info_timeout > (1_s / run_delay) * 5) {
						// Wait 5 seconds for response, then timeout
						PX4_ERR("Timeout waiting for response from DShot driver");
						prev_esc_update_state = esc_update_state;
						esc_update_state = 99;
						flashing_result = 11;

						break;
					}
				}

				break;
			case 4:
				// Received fw versions, next ask DShot for flashing
				// DShot should disable its outputs until we send another request
				// First, read any pending ack
				_esc_flasher_request_ack_sub.copy(&update);

				// Send REQUEST_FLASHING request to DShot driver
				memset(&request, 0, sizeof(request));
				request.timestamp = hrt_absolute_time();
				request.msg_id = ++msg_id;
				request.request = esc_flasher_request_s::REQUEST_FLASHING;
				for (uint32_t i = 0; i < ESC_COUNT; i++) {
					if (esc_update_motors[i]) request.motor_flags |= (1 << i);
				}
				_esc_flasher_request_pub.publish(request);

				esc_info_timeout = 0;
				esc_update_state = 5;

				break;
			case 5:
				// Wait for response from DShot driver to REQUEST_FLASHING request
				if (_esc_flasher_request_ack_sub.updated()) {
					esc_flasher_request_ack_s response;
					_esc_flasher_request_ack_sub.copy(&response);

					// Save/print response
					if (response.msg_id == msg_id && response.request == esc_flasher_request_ack_s::REQUEST_FLASHING &&
						response.result == esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_ACCEPTED) {
						for (uint32_t i = 0; i < ESC_COUNT; i++) {
							if (response.gpio_flags & (1 << i)) {
								esc_gpios[i] = response.gpio_pins[i];
								PX4_INFO("ESC_INFO request filled, ESC %d GPIO %d",
									(int)i, (int)esc_gpios[i]);
							}
						}
						PX4_INFO("REQUEST_FLASHING response received");
					}
					else {
						// Mismatch somewhere
						PX4_ERR("Response from DShot mismatch! Response: %d, msg_id: %d, response msg_id: %d",
							(int)response.result, (int)msg_id, (int)response.msg_id);
						prev_esc_update_state = esc_update_state;
						esc_update_state = 99;
						flashing_result = 12;

						break;
					}

					esc_update_state = 6;
				}
				else {
					esc_info_timeout++;
					if (esc_info_timeout > (1_s / run_delay) * 5) {
						// Wait 5 seconds for response, then timeout
						PX4_ERR("Timeout waiting for response from DShot driver");
						prev_esc_update_state = esc_update_state;
						esc_update_state = 99;
						flashing_result = 13;

						break;
					}
				}

				break;
			case 6:
				// Start bit-banging!
				// First set GPIOs as outputs and set high '1'
				for (uint32_t i = 0; i < ESC_COUNT; i++) {
					gpio = esc_gpios[i];
					if (gpio) {
						gpio &= ~(GPIO_MODE_MASK);
						gpio |= GPIO_OUTPUT;
						gpio |= GPIO_OUTPUT_SET;
						px4_arch_configgpio(gpio);

						// If we have set any GPIO to high, set a flag that can be checked during Cancel request
						gpio_high = 1;
					}
				}

				PX4_INFO("Set motor GPIOs to output '1'");

				time_gpio_set = time_now;
				esc_update_state = 7;

				if (_esc_update == ESCUpdate::Recover) {
					// Special state for holding until Recover2 is sent
					esc_update_state = 30;
					PX4_INFO("Insert battery, then set param ESC_UPDATE to 4 to continue");
				}

				break;
			case 30:
				// Wait for ESC_UPDATE 4

				break;
			case 31:
				// Recovery2 has been sent, battery should be in
				esc_update_motor_index = 0;
				time_gpio_set = time_now;
				esc_update_state = 7;

				break;
			case 7:
				// Wait > 0.5 seconds for ESCs to reboot to bootloader
				if (time_now - time_gpio_set > 600_ms) {
					esc_update_motor_index = 0;
					esc_update_state = 8;
				}

				break;
			case 8:
				// Send AM32 bootloader init packet
				if (esc_update_motor_index >= ESC_COUNT) {
					// How did we get here?
					// We are done, for now
					esc_update_state = 20;
					flashing_result = 2;

					break;
				}
				if (esc_update_motors[esc_update_motor_index]) {
					//int bitbang_send_packet(uint32_t gpio, uint8_t* packet, uint16_t length, uint8_t* response, uint8_t response_length);
					retval = bitbang_send_packet(esc_gpios[esc_update_motor_index], (uint8_t*)am32_boot_init, sizeof(am32_boot_init),
						response_data, am32_boot_init_resp_len);
					if (retval == am32_boot_init_resp_len) {
						// Yay!
						PX4_INFO("Response received from AM32 Bootloader %d: %d %d %d %d %d %d %d %d %d", (int)(esc_update_motor_index + 1),
							(int)response_data[0], (int)response_data[1], (int)response_data[2], (int)response_data[3], (int)response_data[4],
							(int)response_data[5], (int)response_data[6], (int)response_data[7], (int)response_data[8]);

						// Move on to next motor
						esc_update_motor_index++;
						if (esc_update_motor_index >= ESC_COUNT) {
							// Begin the flashing process
							memset(&esc_flashing_state, 0, sizeof(esc_flashing_state));
							esc_update_state = 10;
						}
					}
					else {
						// Oh no
						PX4_ERR("Wrong or no response received from AM32 Bootloader");
						prev_esc_update_state = esc_update_state;
						esc_update_state = 99;
						flashing_result = 14;
					}
				}
				else {
					esc_update_motor_index++;
				}

				break;
			case 9:
				// Unused state

				break;
			case 10:
				// Flashing order is:
				// Setup next write locally - Get address, size of next chunk
				// Set address of the next write chunk (CMD 0xFF), check ACK
				// Set buffer size of the next chunk (CMD 0xFE)
				// Send buffer (no CMD, just data), check ACK
				// Send write command (CMD 0x01), check ACK
				// Go back to Set address and loop until write all firmware bytes
				// Send run app command (CMD 0x00 0x00 0x00 0x00)
				// Success!

				// Use esc_flashing_state struct to flash 1 ESC at a time
				// Get first motor
				for (uint32_t i = esc_flashing_state.current_motor; i < ESC_COUNT; i++) {
					if (esc_update_motors[i]) {
						esc_flashing_state.current_motor = i;
						esc_flashing_state.current_gpio = esc_gpios[i];
						break;
					}
				}

				if (esc_flashing_state.current_motor >= ESC_COUNT) {
					// All motors complete, move on to finishing state
					PX4_INFO("All motors flashed successfully!");
					esc_update_state = 16;
					break;
				}

				esc_update_state = 11;

				break;
			case 11:
				// Setup firmware flash next part (am32_firmware or am32_firmware_tag)
				if (esc_flashing_state.fw_part == 0) {
					esc_flashing_state.fw_length = sizeof(am32_firmware);
					esc_flashing_state.fw_bytes_sent = 0;
					esc_flashing_state.fw_bytes_left = esc_flashing_state.fw_length;
					esc_flashing_state.base_address = AM32_FIRMWARE_ADDR & (0xFFFF);
				}
				else if (esc_flashing_state.fw_part == 1) {
					esc_flashing_state.fw_length = sizeof(am32_firmware_tag);
					esc_flashing_state.fw_bytes_sent = 0;
					esc_flashing_state.fw_bytes_left = esc_flashing_state.fw_length;
					esc_flashing_state.base_address = AM32_FIRMWARE_TAG_ADDR & (0xFFFF);
				}

				esc_update_state = 12;

				break;
			case 12:
				// Copy next chunk to tx_buffer
				esc_flashing_state.tx_length = esc_flashing_state.fw_bytes_left;
				if (esc_flashing_state.tx_length > max_chunk_length) {
					esc_flashing_state.tx_length = max_chunk_length;
				}

				// Copy data and set next address
				if (esc_flashing_state.fw_part == 0) {
					memcpy(esc_flashing_state.tx_buffer, &am32_firmware[esc_flashing_state.fw_bytes_sent], esc_flashing_state.tx_length);
						esc_flashing_state.next_address = esc_flashing_state.base_address + esc_flashing_state.fw_bytes_sent;
				}
				else {
					memcpy(esc_flashing_state.tx_buffer, &am32_firmware_tag[esc_flashing_state.fw_bytes_sent], esc_flashing_state.tx_length);
						esc_flashing_state.next_address = esc_flashing_state.base_address + esc_flashing_state.fw_bytes_sent;
				}

				// Make CRC, append it on to the tx packet
				esc_flashing_state.crc = make_crc(esc_flashing_state.tx_buffer, esc_flashing_state.tx_length);
				esc_flashing_state.tx_buffer[esc_flashing_state.tx_length] = (uint8_t)(esc_flashing_state.crc & 0x00FF);
				esc_flashing_state.tx_buffer[esc_flashing_state.tx_length + 1] = (uint8_t)(((esc_flashing_state.crc & 0xFF00) >> 8));

				// Send set address command
				am32_set_addr[0] = 0xFF;
				am32_set_addr[1] = 0x00;
				am32_set_addr[2] = (esc_flashing_state.next_address & 0xFF00) >> 8;
				am32_set_addr[3] = (esc_flashing_state.next_address & 0x00FF);
				esc_flashing_state.crc = make_crc(am32_set_addr, 4);
				am32_set_addr[4] = (uint8_t)(esc_flashing_state.crc & 0x00FF);
				am32_set_addr[5] = (uint8_t)((esc_flashing_state.crc & 0xFF00) >> 8);

				retval = bitbang_send_packet(esc_flashing_state.current_gpio, am32_set_addr, sizeof(am32_set_addr),
					response_data, am32_set_addr_resp_len);
				if (retval == am32_set_addr_resp_len && response_data[0] == 0x30) {
					// Received ACK, continue to next step
					esc_update_state = 13;
				}
				else {
					// Something went wrong
					PX4_ERR("Error in Set Address command for motor %d", (int)esc_flashing_state.current_motor + 1);
					prev_esc_update_state = esc_update_state;
					esc_update_state = 99;
					flashing_result = 15;

					break;
				}

				break;
			case 13:
				// Sent set buffer size command
				am32_set_buff_size[0] = 0xFE;
				am32_set_buff_size[1] = 0x00;
				if (esc_flashing_state.tx_length == max_chunk_length) {
					am32_set_buff_size[2] = 1;
					am32_set_buff_size[3] = 0;
				}
				else {
					am32_set_buff_size[2] = 0;
					am32_set_buff_size[3] = esc_flashing_state.tx_length;
				}
				esc_flashing_state.crc = make_crc(am32_set_buff_size, 4);
				am32_set_buff_size[4] = (uint8_t)(esc_flashing_state.crc & 0x00FF);
				am32_set_buff_size[5] = (uint8_t)((esc_flashing_state.crc & 0xFF00) >> 8);

				retval = bitbang_send_packet(esc_flashing_state.current_gpio, am32_set_buff_size, sizeof(am32_set_buff_size),
					response_data, am32_set_buff_size_resp_len);

				// There is no expected response to this command, so proceed to send the buffer
				retval = bitbang_send_packet(esc_flashing_state.current_gpio, esc_flashing_state.tx_buffer, esc_flashing_state.tx_length + 2,
					response_data, am32_send_buff_resp_len);
				if (retval == am32_send_buff_resp_len && response_data[0] == 0x30) {
					// Received ACK, continue to next step
					esc_update_state = 14;
				}
				else {
					// Something went wrong
					PX4_ERR("Error in Send Buffer command for motor %d", (int)esc_flashing_state.current_motor + 1);
					prev_esc_update_state = esc_update_state;
					esc_update_state = 99;
					flashing_result = 16;

					break;
				}

				break;
			case 14:
				// Send Write command
				am32_write[0] = 0x01;
				am32_write[1] = 0;
				esc_flashing_state.crc = make_crc(am32_write, 2);
				am32_write[2] = (uint8_t)(esc_flashing_state.crc & 0x00FF);
				am32_write[3] = (uint8_t)((esc_flashing_state.crc & 0xFF00) >> 8);

				retval = bitbang_send_packet(esc_flashing_state.current_gpio, am32_write, sizeof(am32_write),
					response_data, am32_write_resp_len);
				if (retval == am32_write_resp_len && response_data[0] == 0x30) {
					// Received ACK, continue to next step
					if (esc_flashing_state.fw_part == 0) {
						PX4_INFO("Sent firmware packet %d of %d to motor %d", (int)(esc_flashing_state.fw_bytes_sent / max_chunk_length) + 1,
							(int)(esc_flashing_state.fw_length / max_chunk_length) + 1, (int)esc_flashing_state.current_motor + 1);
					}
					else {
						PX4_INFO("Sent firmware tag section to motor %d", (int)esc_flashing_state.current_motor + 1);
					}
					// Set byte counts after latest write
					esc_flashing_state.fw_bytes_sent += esc_flashing_state.tx_length;
					esc_flashing_state.fw_bytes_left -= esc_flashing_state.tx_length;

					if (esc_flashing_state.fw_bytes_left) {
						// Go back to setup next chunk
						esc_update_state = 12;
					}
					else {
						// Done with firmware part, check states
						if (esc_flashing_state.fw_part == 0) {
							esc_flashing_state.fw_part = 1;
							esc_update_state = 11;
						}
						else {
							// Flashed all parts, run app and move to next motor
							PX4_INFO("Firmware update complete on motor %d", (int)esc_flashing_state.current_motor + 1);
							esc_update_state = 15;
						}
					}
				}
				else {
					// Something went wrong
					PX4_ERR("Error in Write command for motor %d", (int)esc_flashing_state.current_motor);
					prev_esc_update_state = esc_update_state;
					esc_update_state = 99;
					flashing_result = 17;

					break;
				}

				break;
			case 15:
				// Send Run App command and move on to next motor
				retval = bitbang_send_packet(esc_flashing_state.current_gpio, (uint8_t*)am32_run_app, sizeof(am32_run_app),
					response_data, am32_run_app_resp_len);

				esc_flashing_state.current_motor++;
				esc_flashing_state.fw_part = 0;
				esc_update_state = 10;

				break;
			case 16:
				// All motors should be flashed by this state, notify DShot so it can re-enable
				memset(&request, 0, sizeof(request));
				request.timestamp = hrt_absolute_time();
				request.msg_id = ++msg_id;
				request.request = esc_flasher_request_s::REQUEST_FLASHING_COMPLETE;
				_esc_flasher_request_pub.publish(request);

				esc_info_timeout = 0;
				esc_update_state = 17;

				break;
			case 17:
				// Wait for response from DShot driver to REQUEST_FLASHING_COMPLETE request
				if (_esc_flasher_request_ack_sub.updated()) {
					esc_flasher_request_ack_s response;
					_esc_flasher_request_ack_sub.copy(&response);

					// Save/print response
					if (response.msg_id == msg_id && response.request == esc_flasher_request_ack_s::REQUEST_FLASHING_COMPLETE &&
						response.result == esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_ACCEPTED) {
						PX4_INFO("REQUEST_FLASHING_COMPLETE response received");
					}
					else {
						// Mismatch somewhere
						PX4_ERR("Response from DShot mismatch! Response: %d, msg_id: %d, response msg_id: %d",
							(int)response.result, (int)msg_id, (int)response.msg_id);
						prev_esc_update_state = esc_update_state;
						esc_update_state = 99;
						flashing_result = 18;

						break;
					}

					esc_update_state = 20;
					flashing_result = 2;
				}
				else {
					esc_info_timeout++;
					if (esc_info_timeout > (1_s / run_delay) * 5) {
						// Wait 5 seconds for response, then timeout
						PX4_ERR("Timeout waiting for response from DShot driver");
						prev_esc_update_state = esc_update_state;
						esc_update_state = 99;
						flashing_result = 19;

						break;
					}
				}

				break;
			case 20:
				// Complete
				if (flashing_result == 1) {
					//PX4_ERR("No ESCs need flashing, returning to idle state");
					mavlink_log_critical(&_mavlink_log_pub, "ESCs are already up-to-date, %d.%d\t",
						(int)am32_fw_version_major, (int)am32_fw_version_minor);
				}
				else {
					//PX4_INFO("ESC Flasher finished in %d seconds! Returning to idle state", (int)((time_now - time_flash_start) / 1_s));
					mavlink_log_critical(&_mavlink_log_pub, "ESC flash finished in %d seconds, %d.%d\t",
						(int)((time_now - time_flash_start) / 1_s), (int)am32_fw_version_major, (int)am32_fw_version_minor);
				}

				_esc_update = ESCUpdate::Disabled;
				esc_update_state = 0;
				time_flash_start = 0;
				// Clear motor update flags
				memset(esc_update_motors, 0, sizeof(esc_update_motors));

				// Play success tune
				esc_flasher_tune = 2;

				// Set loop to run slow 1 Hz
				run_delay = run_delay_max;

				break;
			case 96:
				// CANCEL command, reset any GPIOs, wait > 20 ms, send Cancel request to DShot
				if (gpio_high) {
					for (uint32_t i = 0; i < ESC_COUNT; i++) {
						gpio = esc_gpios[i];
						if (gpio) {
							gpio &= ~(GPIO_MODE_MASK);
							gpio |= GPIO_OUTPUT;
							//gpio |= GPIO_OUTPUT_SET;
							px4_arch_configgpio(gpio);
						}
					}
					gpio_high = 0;
					time_gpio_set = time_now;
				}
				else {
					time_gpio_set = 0;
				}
				esc_update_state = 97;

				// Set loop to run fast 200 Hz
				run_delay = run_delay_min;

				break;
			case 97:
				// Send REQUEST_CANCEL request to DShot driver
				// All state variables will be reset in error state
				if (time_now - time_gpio_set > 30_ms) {
					memset(&request, 0, sizeof(request));
					request.timestamp = hrt_absolute_time();
					request.msg_id = ++msg_id;
					request.request = esc_flasher_request_s::REQUEST_CANCEL;
					_esc_flasher_request_pub.publish(request);

					esc_info_timeout = 0;
					esc_update_state = 98;
				}

				break;
			case 98:
				// Wait for response from DShot driver to REQUEST_CANCEL request
				if (_esc_flasher_request_ack_sub.updated()) {
					esc_flasher_request_ack_s response;
					_esc_flasher_request_ack_sub.copy(&response);

					// Save/print response
					if (response.msg_id == msg_id && response.request == esc_flasher_request_ack_s::REQUEST_CANCEL &&
						response.result == esc_flasher_request_ack_s::ESC_FLASHER_CMD_RESULT_ACCEPTED) {
						PX4_INFO("REQUEST_CANCEL response received");
					}
					else {
						// Mismatch somewhere
						PX4_ERR("Response from DShot mismatch! Response: %d, msg_id: %d, response msg_id: %d",
							(int)response.result, (int)msg_id, (int)response.msg_id);
						prev_esc_update_state = esc_update_state;
						esc_update_state = 99;
						flashing_result = 20;

						break;
					}

					esc_update_state = 99;
				}
				else {
					esc_info_timeout++;
					if (esc_info_timeout > (1_s / run_delay) * 5) {
						// Wait 5 seconds for response, then timeout
						PX4_ERR("Timeout waiting for response from DShot driver");
						prev_esc_update_state = esc_update_state;
						esc_update_state = 99;
						flashing_result = 21;

						break;
					}
				}

				break;
			case 99:
				// Error/reset case
				//PX4_ERR("ESC Flasher error state, returning to idle state");
				mavlink_log_critical(&_mavlink_log_pub, "ESC Flash failed, state %d, result %d\t",
					(int)prev_esc_update_state, (int)flashing_result);

				// Play error tune
				esc_flasher_tune = 3;

				// SHOULD WE SEND CANCEL COMMAND TO DSHOT HERE???

				_esc_update = ESCUpdate::Disabled;
				esc_update_state = 0;
				prev_esc_update_state = 0;
				flashing_result = 0;
				time_flash_start = 0;
				// Clear motor update flags
				memset(esc_update_motors, 0, sizeof(esc_update_motors));

				// Set loop to run slow 1 Hz
				run_delay = run_delay_max;

				break;
			}
		}
	}

	// Always publish esc_flasher_status while flashing
	{
		flashing_in_progress = (_esc_update == ESCUpdate::Disabled) ? false : true;
		if (!flashing_in_progress) {
			if (flashing_in_progress_last) {
				// Send ESC Flasher status uORB message, try to send it only once
				memset(&status, 0, sizeof(status));
				status.esc_flashing_in_progress = false;
				status.version_major = _am32_fw_ver_major;
				status.version_minor = _am32_fw_ver_minor;
				status.timestamp = hrt_absolute_time();
				_esc_flasher_status_pub.publish(status);
				// Send vehicle command for LEDs, so Commander will update LEDs with correct blink color
				send_vehicle_command_leds();
				// Reset parameter ESC_UPDATE again, this might update Commander also
				_param_esc_update.reset();
			}
			flashing_in_progress_last = flashing_in_progress;
		}
		else {
			// Send in progress message and LED color at 4 Hz during flashing
			if (flashing_in_progress_last != flashing_in_progress) {
				// Force to send message right now
				time_250ms = 0;
				last_color = 0;
			}
			if (time_now - time_250ms >= 250_ms) {
				time_250ms = time_now;
				memset(&status, 0, sizeof(status));

				if (last_color != led_control_s::COLOR_GREEN) {
					last_color = led_control_s::COLOR_GREEN;
				}
				else {
					last_color = led_control_s::COLOR_BLUE;
				}

				status.esc_flashing_in_progress = true;
				status.timestamp = hrt_absolute_time();
				_esc_flasher_status_pub.publish(status);

				set_physical_led_outputs(last_color);
			}
			flashing_in_progress_last = flashing_in_progress;
		}
	}

	if (time_now - time_1s > 1_s) {
		// 1 second loop
		time_1s = time_now;

		if (print_flag) {
			PX4_INFO("ESC Flasher status:");
			PX4_INFO("     Time: %llu, run calls: %lu", time_now, run_calls);
			PX4_INFO("     Loop rate: %d", (int)(1_s / run_delay));
			PX4_INFO("     ESC Type: %s", esc_types_strings[(uint8_t)_esc_type]);
			PX4_INFO("     ESC Update: %d", (int)_esc_update);
		}

		if (esc_flasher_stop_tune) {
			stop_tune();
			esc_flasher_stop_tune = 0;
		}
		if (esc_flasher_tune) {
			if (esc_flasher_tune == 1) {
				// This will STOP tune from playing during flashing
				//esc_flasher_tune = 0;
			}
			play_tune(esc_flasher_tune);
			//if (esc_flasher_tune == 2 || esc_flasher_tune == 3) {
			if (esc_flasher_tune) {
				// Stop any tune in the next second loop
				esc_flasher_stop_tune = 1;
			}
			esc_flasher_tune = 0;
		}
	}
}

int ESC_Flasher::bitbang_send_packet(uint32_t gpio, uint8_t* packet, uint16_t length, uint8_t* response, uint8_t response_length) {
	static const uint32_t bit_time = 52;
	static const uint32_t half_bit_time = bit_time / 2;
	static uint8_t read_data[16];

	uint16_t write_index = 0;
	uint8_t read_index = 0;
	uint8_t bit_count = 0;
	uint8_t read_bits = 0;

	// Configure GPIO pin as output, set high '1'
	gpio &= ~(GPIO_MODE_MASK);
	gpio |= GPIO_OUTPUT;
	gpio |= GPIO_OUTPUT_SET;
	gpio &= ~(GPIO_PUPD_MASK);
	gpio |= GPIO_PULLUP;
	px4_arch_configgpio(gpio);

	// Enter critical section, disable interrupts
	static irqstate_t irq_state;
	irq_state = px4_enter_critical_section();

	uint8_t nextb;
	uint64_t start;
	uint64_t now;

	for (uint16_t k = 0; k < length; k++) {
		nextb = packet[write_index++];

		px4_arch_gpiowrite(gpio, 0);
		start = hrt_absolute_time();
		while (true) {
			now = hrt_absolute_time();
			if (now - start >= bit_time) {
				break;
			}
		}

		for (uint32_t i = 0; i < 8; i++) {
			if (nextb & 1) px4_arch_gpiowrite(gpio, 1);
			else px4_arch_gpiowrite(gpio, 0);
			nextb >>= 1;

			start = hrt_absolute_time();
			while (true) {
				now = hrt_absolute_time();
				if (now - start >= bit_time) {
					break;
				}
			}
		}

		px4_arch_gpiowrite(gpio, 1);
		start = hrt_absolute_time();
		while (true) {
			now = hrt_absolute_time();
			if (now - start >= bit_time) {
				break;
			}
		}
	}

	// Set GPIO to input
	gpio &= ~(GPIO_MODE_MASK);
	gpio |= GPIO_INPUT;
	gpio &= ~(GPIO_PUPD_MASK);
	gpio |= GPIO_PULLUP;
	px4_arch_configgpio(gpio);

	// Read response bytes, if any
	read_index = 0;
	if (response_length && response) {
		memset(read_data, 0, sizeof(read_data));

		for (uint32_t i = 0; i < response_length; i++) {
			start = hrt_absolute_time();
			while (px4_arch_gpioread(gpio)) {
				// Wait for pin to go low
				now = hrt_absolute_time();
				if (now - start > 100000) {
					// Timeout 100ms waiting for start bit

					// Leave critical section, re-enable interrupts
					px4_leave_critical_section(irq_state);
					return -2;
				}
			}

			// Wait 1/2 bit-time for start bit
			start = hrt_absolute_time();
			while (true) {
				now = hrt_absolute_time();
				if (now - start >= half_bit_time) {
					break;
				}
			}

			// Read 8 bits
			bit_count = 0;
			read_bits = 0;
			while (bit_count < 8) {
				// Wait 1 bit-time to read next bit
				start = hrt_absolute_time();
				while (true) {
					now = hrt_absolute_time();
					if (now - start >= bit_time) {
						break;
					}
				}
				read_bits |= (px4_arch_gpioread(gpio) << (bit_count++));
			}

			read_data[read_index++] = read_bits;

			// Now wait 1 more bit-time until we get to stop bit
			start = hrt_absolute_time();
			while (true) {
				now = hrt_absolute_time();
				if (now - start >= bit_time) {
					break;
				}
			}
		}

		// Copy data to out buffer
		memcpy(response, read_data, read_index);
	}

	// Put pin back into output mode, set high
	gpio &= ~(GPIO_MODE_MASK);
	gpio |= GPIO_OUTPUT;
	gpio |= GPIO_OUTPUT_SET;
	px4_arch_configgpio(gpio);

	// Delay here to give some time before next command
	start = hrt_absolute_time();
	while (true) {
		now = hrt_absolute_time();
		// Delay 1 bit time, for 8 bits
		if (now - start >= bit_time * 8) {
			break;
		}
	}

	// Leave critical section, re-enable interrupts
	px4_leave_critical_section(irq_state);

	return read_index;
}

// CRC struct
typedef union {
	uint8_t bytes[2];
	uint16_t word;
} uint8_16_u;

uint16_t ESC_Flasher::make_crc(uint8_t* buffer, uint16_t length) {
	static uint8_16_u CRC_16;
	CRC_16.word=0;

	for(int i = 0; i < length; i++) {
		uint8_t xb = buffer[i];
		for (uint8_t j = 0; j < 8; j++) {
			if (((xb & 0x01) ^ (CRC_16.word & 0x0001)) !=0 ) {
				CRC_16.word = CRC_16.word >> 1;
				CRC_16.word = CRC_16.word ^ 0xA001;
			}
			else {
				CRC_16.word = CRC_16.word >> 1;
			}
			xb = xb >> 1;
		}
	}

	return CRC_16.word;
}

uint8_t ESC_Flasher::get_current_arming_state(void) {
	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&_vehicle_status);
	}

	return _vehicle_status.arming_state;
}

void ESC_Flasher::set_physical_led_outputs(uint8_t color)
{
#ifdef FIREFLY_ESC
	// Control I2C LEDs
	led_control_s led_control_msg{0};
	led_control_msg.mode = led_control_s::MODE_MANUAL_RGB;
	led_control_msg.color = led_control_s::COLOR_MANUAL;
	led_control_msg.priority = led_control_s::MAX_PRIORITY;
	if (color == 2) {
		led_control_msg.rgb_manual[0] = 0;
		led_control_msg.rgb_manual[1] = 255;
		led_control_msg.rgb_manual[2] = 0;
	}
	else if (color == 3) {
		led_control_msg.rgb_manual[0] = 0;
		led_control_msg.rgb_manual[1] = 0;
		led_control_msg.rgb_manual[2] = 255;
	}
	led_control_msg.timestamp = hrt_absolute_time();
	_led_control_pub.publish(led_control_msg);
#else
	// Control I2C LEDs
	led_control_s led_control_msg{0};
	led_control_msg.mode = led_control_s::MODE_BREATHE;
	led_control_msg.color = led_control_s::COLOR_CYAN;
	led_control_msg.priority = led_control_s::MAX_PRIORITY;
	led_control_msg.timestamp = hrt_absolute_time();
	_led_control_pub.publish(led_control_msg);
#endif
}

void ESC_Flasher::send_vehicle_command_leds(void) {
	vehicle_command_s vcmd{0};

	// Turn LEDs off when done flashing (-1.f)
	vcmd.param1 = -1.f;
	vcmd.param2 = -1.f;
	vcmd.param3 = -1.f;
	vcmd.target_system = 1;
	vcmd.target_component = 1;
	vcmd.source_system = 1;
	vcmd.source_component = 1;
	vcmd.confirmation = 0;
	vcmd.from_external = false;
	vcmd.timestamp = hrt_absolute_time();
	_vehicle_command_pub.publish(vcmd);
}

ESC_Flasher *ESC_Flasher::instantiate(int argc, char *argv[]) {
	ESC_Flasher *instance = new ESC_Flasher();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int ESC_Flasher::task_spawn(int argc, char *argv[]) {
	_task_id = px4_task_spawn_cmd("esc_flasher",
		SCHED_DEFAULT,
		SCHED_PRIORITY_FAST_DRIVER,
		PX4_STACK_ADJUSTED(2048),
		(px4_main_t)&run_trampoline,
		(char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int ESC_Flasher::print_status()
{
	PX4_INFO("ESC Flasher status:");
	PX4_INFO("     Time: %llu, run calls: %lu", time_now, run_calls);
	PX4_INFO("     Loop rate: %d", (int)(1_s / run_delay));
	PX4_INFO("     ESC Type: %s", esc_types_strings[(uint8_t)_esc_type]);
	PX4_INFO("     ESC Update: %d", (int)_esc_update);

	perf_print_counter(_loop_perf);

	return 0;
}

int ESC_Flasher::print_usage(const char *reason) {
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ESC Flasher module allows configuration and reflashing
of supported ESCs
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("esc_flasher", "system");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the background task");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int ESC_Flasher::custom_command(int argc, char *argv[]) {
	if (argc < 1) {
		PX4_INFO("Usage: esc_flasher <command>");
		return ESC_FLASHER_RET_NOK;
	}

	// Get our current object
	ESC_Flasher* mESC_Flasher = NULL;
	(void)mESC_Flasher;
	if (is_running()) {
		mESC_Flasher = (ESC_Flasher*)_object.load();
	}
	else {
		return ESC_FLASHER_RET_NOK;
	}

	int ret = ESC_FLASHER_RET_OK;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// Get motor index
	int motor_index = -1;
	uint16_t motor_flags = 0;
	bool found_m = false;
	bool found_a = false;
	while ((ch = px4_getopt(argc, argv, "am:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'a':
			if (found_m) {
				return print_usage("Conflicting flags, use only -m <1 2 3 4> or -a");
			}
			// Flash all 4 motors
			motor_flags = 0b0000000000001111;
			found_a = true;
			break;
		case 'm':
			if (found_a) {
				return print_usage("Conflicting flags, use only -m <1 2 3 4> or -a");
			}
			motor_index = strtol(myoptarg, nullptr, 10);
			if (motor_index < 1 || motor_index > 4) {
				return print_usage("Motor index out of range. Acceptable values are 1-4");
			}
			motor_flags |= (1 << (motor_index - 1));
			found_m = true;
			break;
		default:
			return print_usage("unrecognized flag");
		}
	}

	const char *command = argv[0];

	// Handle various commands issued via the command line
	if (strcmp(command, "print") == 0) {
		// Print status every second
		if (argc < 2) {
			PX4_ERR("Usage: esc_flasher print <0 for off, 1 for on>");
			ret = ESC_FLASHER_RET_NOK;
		}

		int enable = strtol(argv[1], NULL, 10);
		mESC_Flasher->print_flag = (bool)enable;

		return ret;
	}
	else if (strcmp(command, "flash") == 0) {
		// Flash a specific motor
		if (motor_flags == 0) {
			return print_usage("Usage: esc_flasher flash -m <1 2 3 4>");
		}

		mESC_Flasher->start_flash = 1;
		mESC_Flasher->motor_flags = motor_flags;
		// Wake it from the sleep-loop, this helps if we are running at 1 Hz
		mESC_Flasher->wake = true;

		return ret;
	}
	else if (strcmp(command, "cancel") == 0) {
		// Cancel any current request, return control to DShot
		mESC_Flasher->cancel_flash = 1;
		// Wake it from the sleep-loop, this helps if we are running at 1 Hz
		mESC_Flasher->wake = true;

		return ret;
	}

	return print_usage("Unknown command");
}

extern "C" __EXPORT int esc_flasher_main(int argc, char *argv[]) {
	return ESC_Flasher::main(argc, argv);
}
