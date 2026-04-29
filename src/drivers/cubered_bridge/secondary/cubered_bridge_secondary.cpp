/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file cubered_bridge_secondary.cpp
 *
 * CubeRed IO Module - PX4IO simulation for CubeRed Secondary
 *
 * This module simulates PX4IO functionality on the CubeRed Secondary
 * by providing low-latency serial communication on ttyS4.
 */

#include "cubered_bridge_secondary.h"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

// Include PX4IO protocol definitions
#include <modules/px4iofirmware/protocol.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

using namespace time_literals;

ModuleBase::Descriptor CuberedBridgeSecondary::desc {CuberedBridgeSecondary::task_spawn, CuberedBridgeSecondary::custom_command, CuberedBridgeSecondary::print_usage};

#undef PX4_DEBUG
#define PX4_DEBUG PX4_INFO

// Configuration page data (simulating PX4IO)
static const uint16_t config_page[] = {
	PX4IO_PROTOCOL_VERSION,		// PX4IO_P_CONFIG_PROTOCOL_VERSION
	2,				// PX4IO_P_CONFIG_HARDWARE_VERSION (Simulate PX4IOv2)
	5,				// PX4IO_P_CONFIG_BOOTLOADER_VERSION
	PX4IO_MAX_TRANSFER_LEN,		// PX4IO_P_CONFIG_MAX_TRANSFER
	PX4IO_PROTOCOL_MAX_CONTROL_COUNT,// PX4IO_P_CONFIG_CONTROL_COUNT
	8,				// PX4IO_P_CONFIG_ACTUATOR_COUNT
	18,				// PX4IO_P_CONFIG_RC_INPUT_COUNT
	3,				// PX4IO_P_CONFIG_ADC_INPUT_COUNT
};

// Status page data
static uint16_t status_page[8] = {
	0,				// PX4IO_P_STATUS_FREEMEM
	0,				// PX4IO_P_STATUS_CPULOAD
	PX4IO_P_STATUS_FLAGS_INIT_OK | PX4IO_P_STATUS_FLAGS_ARM_SYNC,	// PX4IO_P_STATUS_FLAGS
	0,				// PX4IO_P_STATUS_ALARMS
	0,				// PX4IO_P_STATUS_VSERVO
	0,				// PX4IO_P_STATUS_VRSSI
	0,				// PX4IO_P_STATUS_PRSSI
	0,				// PX4IO_P_STATUS_MIXER
};

// Servo PWM values (microseconds)
static uint16_t servos_page[8] = {0};

// Direct PWM values (microseconds)
static uint16_t direct_pwm_page[8] = {0};

// Raw RC input values
static uint16_t raw_rc_input_page[] = {
	0,				// PX4IO_P_RAW_RC_COUNT
	0,				// PX4IO_P_RAW_RC_FLAGS
	0,				// PX4IO_P_RAW_RC_NRSSI
	0,				// PX4IO_P_RAW_RC_DATA
	0,				// PX4IO_P_RAW_FRAME_COUNT
	0,				// PX4IO_P_RAW_LOST_FRAME_COUNT
	// PX4IO_P_RAW_RC_BASE + channels will be added dynamically
};

// Raw ADC input values
static uint16_t raw_adc_input_page[3] = {0}; // Support up to 3 ADC inputs

// PWM rate group information
static uint16_t pwm_info_page[8] = {0};

// PWM disarmed values that are active even when SAFETY_SAFE
static uint16_t disarmed_pwm_page[8] = {0};

// PWM failsafe values - zero disables the output
static uint16_t failsafe_pwm_page[8] = {0};

// Setup page data (simulating PX4IO)
static uint16_t setup_page[] = {
	PX4IO_P_SETUP_FEATURES_ADC_RSSI,	// PX4IO_P_SETUP_FEATURES
	0,					// PX4IO_P_SETUP_ARMING
	0,					// PX4IO_P_SETUP_PWM_RATES
	50,					// PX4IO_P_SETUP_PWM_DEFAULTRATE
	200,					// PX4IO_P_SETUP_PWM_ALTRATE
	72,					// PX4IO_P_SETUP_SBUS_RATE
	10000,					// PX4IO_P_SETUP_VSERVO_SCALE
	0,					// PX4IO_P_SETUP_SET_DEBUG
	0,					// PX4IO_P_SETUP_REBOOT_BL
	0,					// PX4IO_P_SETUP_CRC
	0,					// PX4IO_P_SETUP_CRC + 1
	PX4IO_THERMAL_IGNORE,		// PX4IO_P_SETUP_THERMAL
	0,					// PX4IO_P_SETUP_ENABLE_FLIGHTTERMINATION
	0,					// PX4IO_P_SETUP_PWM_RATE_GROUP0
	0,					// PX4IO_P_SETUP_PWM_RATE_GROUP1
	0,					// PX4IO_P_SETUP_PWM_RATE_GROUP2
	0,					// PX4IO_P_SETUP_PWM_RATE_GROUP3
	0,					// PX4IO_P_SETUP_SAFETY_BUTTON_ACK
	0,					// PX4IO_P_SETUP_SAFETY_OFF
};

CuberedBridgeSecondary::CuberedBridgeSecondary()
{
}

CuberedBridgeSecondary::~CuberedBridgeSecondary()
{
	if (_serial_fd >= 0) {
		::close(_serial_fd);
	}

	// Clean up PWM outputs
	if (_pwm_initialized) {
		up_pwm_servo_deinit(_pwm_mask);
	}

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

int CuberedBridgeSecondary::task_spawn(int argc, char *argv[])
{
	int task_id = px4_task_spawn_cmd("cubered_bridge_secondary",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5, // High priority for low latency
					 2048,
					 (px4_main_t) &run_trampoline,
					 (char *const *)argv);

	if (task_id < 0) {
		return -errno;
	}

	desc.task_id = task_id;
	return 0;
}

int CuberedBridgeSecondary::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return CuberedBridgeSecondary::instantiate(ac, av);
	}, argc, argv);
}

CuberedBridgeSecondary *CuberedBridgeSecondary::instantiate(int argc, char *argv[])
{
	CuberedBridgeSecondary *instance = new CuberedBridgeSecondary();

	if (instance && !instance->init()) {
		delete instance;
		return nullptr;
	}

	return instance;
}

bool CuberedBridgeSecondary::init()
{
	if (init_serial() != PX4_OK) {
		PX4_ERR("Failed to initialize serial port");
		return false;
	}

	if (init_pwm() != PX4_OK) {
		PX4_ERR("Failed to initialize PWM outputs");
		return false;
	}

	PX4_INFO("CuberedBridgeSecondary initialized on %s with PWM outputs", DEVICE_NAME);
	return true;
}

int CuberedBridgeSecondary::init_serial()
{
	// Open serial port
	_serial_fd = ::open(DEVICE_NAME, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_serial_fd < 0) {
		PX4_ERR("Failed to open %s: %s", DEVICE_NAME, strerror(errno));
		return PX4_ERROR;
	}

	// Configure serial port
	termios uart_config;

	if (tcgetattr(_serial_fd, &uart_config) < 0) {
		PX4_ERR("Failed to get termios config: %s", strerror(errno));
		::close(_serial_fd);
		_serial_fd = -1;
		return PX4_ERROR;
	}

	// Set baud rate
	if (cfsetispeed(&uart_config, BAUDRATE) < 0 || cfsetospeed(&uart_config, BAUDRATE) < 0) {
		PX4_ERR("Failed to set baudrate: %s", strerror(errno));
		::close(_serial_fd);
		_serial_fd = -1;
		return PX4_ERROR;
	}

	// Configure for raw mode
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config.c_oflag &= ~OPOST;
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	uart_config.c_cflag &= ~(CSIZE | PARENB);
	uart_config.c_cflag |= CS8;

	// Set timeouts for non-blocking read
	uart_config.c_cc[VMIN] = 6;
	uart_config.c_cc[VTIME] = 1;

	if (tcsetattr(_serial_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("Failed to set termios config: %s", strerror(errno));
		::close(_serial_fd);
		_serial_fd = -1;
		return PX4_ERROR;
	}

	// Flush any existing data
	tcflush(_serial_fd, TCIOFLUSH);

	return PX4_OK;
}

int CuberedBridgeSecondary::init_pwm()
{
	// Initialize PWM channels (similar to pwm_out module)
	_pwm_mask = ((1u << DIRECT_PWM_OUTPUT_CHANNELS) - 1);

	// Initialize timer rates
	for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		_timer_rates[timer] = -1;

		uint32_t channels = io_timer_get_group(timer);

		if (channels == 0) {
			continue;
		}

		// Set default PWM rate (400Hz like PX4IO)
		_timer_rates[timer] = 400;
	}

	// Initialize PWM hardware
	printf("CuberedBridgeSecondary: Attempting PWM init with mask 0x%02" PRIx32 "\n", _pwm_mask);
	int ret = up_pwm_servo_init(_pwm_mask);

	if (ret < 0) {
		printf("CuberedBridgeSecondary: up_pwm_servo_init failed (%i)\n", ret);
		return PX4_ERROR;
	}

	printf("CuberedBridgeSecondary: up_pwm_servo_init returned mask 0x%02x\n", ret);
	_pwm_mask = ret;

	// Set the timer rates
	for (int timer = 0; timer < MAX_IO_TIMERS; ++timer) {
		uint32_t channels = _pwm_mask & up_pwm_servo_get_rate_group(timer);

		if (channels == 0) {
			printf("CuberedBridgeSecondary: Timer %d has no channels\n", timer);
			continue;
		}

		printf("CuberedBridgeSecondary: Setting timer %d rate to %d Hz for channels 0x%02" PRIx32 "\n", timer, _timer_rates[timer],
		       channels);
		ret = up_pwm_servo_set_rate_group_update(timer, _timer_rates[timer]);

		if (ret != 0) {
			printf("CuberedBridgeSecondary: up_pwm_servo_set_rate_group_update failed for timer %i, rate %i (%i)\n", timer, _timer_rates[timer],
			       ret);
			_timer_rates[timer] = -1;
			_pwm_mask &= ~channels;

		} else {
			printf("CuberedBridgeSecondary: Timer %d configured successfully\n", timer);
		}
	}

	_pwm_initialized = true;

	// Always arm PWM hardware (like PWMOut does with pwm_on = true)
	// The armed/disarmed state will control values, not hardware state
	up_pwm_servo_arm(true, _pwm_mask);

	// Debug: Read back GPIO states to verify direction control
	bool bidir_state = px4_arch_gpioread(GPIO_BIDIR_DISABLED);
	bool unidir_state = px4_arch_gpioread(GPIO_UNIDIR_ENABLED);
	printf("CuberedBridgeSecondary: PWM direction control - BIDIR_DISABLED=%s, UNIDIR_ENABLED=%s\n",
	       bidir_state ? "HIGH" : "LOW", unidir_state ? "HIGH" : "LOW");

	PX4_INFO("PWM initialized with mask 0x%02" PRIx32, _pwm_mask);
	return PX4_OK;
}

void CuberedBridgeSecondary::run()
{
	PX4_INFO("CuberedBridgeSecondary task started - simulating PX4IO on CubeRed Secondary");

	while (!should_exit()) {
		perf_begin(_loop_perf);
		perf_count(_loop_interval_perf);

		poll_and_process();

		perf_end(_loop_perf);
	}

	PX4_INFO("CuberedBridgeSecondary task exiting");
}

void CuberedBridgeSecondary::poll_and_process()
{
	const size_t min_packet_size = 6; // Minimum PX4IO packet size

	// Always start fresh with clean buffer
	IOPacket packet {};
	size_t bytes_read = 0;
	size_t expected_packet_size = min_packet_size;

	while (bytes_read < expected_packet_size) {
		pollfd fds[1];
		fds[0].fd = _serial_fd;
		fds[0].events = POLLIN;

		int ret = poll(fds, 1, POLL_TIMEOUT_MS);

		if (ret > 0) {
			if (fds[0].revents & POLLIN) {
				// Data available to read from CubeRed Primary
				int read_ret = ::read(_serial_fd, (reinterpret_cast<uint8_t *>(&packet)) + bytes_read, sizeof(IOPacket) - bytes_read);

				if (read_ret > 0) {
					bytes_read += read_ret;

					// Once we have at least min packet, try to determine actual size
					if (bytes_read >= min_packet_size && expected_packet_size == min_packet_size) {
						expected_packet_size = PKT_SIZE(packet);
					}

					// Check CRC when we have the complete packet
					if (bytes_read >= expected_packet_size) {

						if (validate_crc(packet)) {
							//printf("CRC OK at %zu bytes\n", bytes_read);
							process_received_data(packet);
							return;

						} else {
							printf("CRC failed at %zu bytes - restarting\n", bytes_read);
							// CRC failed - start fresh
							bytes_read = 0;
							expected_packet_size = min_packet_size;
							packet = {};
							continue;
						}
					}

				} else if (read_ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
					PX4_ERR("Read error: %s", strerror(errno));
					return;
				}
			}

		} else if (ret < 0 && errno != EINTR) {
			PX4_ERR("Poll error: %s", strerror(errno));
			return;

		} else {
			// Timeout, start fresh with clean buffer
			bytes_read = 0;
			expected_packet_size = min_packet_size;
			packet = {};
			return;
		}
	}
}

void CuberedBridgeSecondary::process_received_data(IOPacket &packet)
{
	uint8_t code = PKT_CODE(packet);
	//uint8_t count = PKT_COUNT(packet);

	if (code == PKT_CODE_READ) {
		handle_read_request(packet);

	} else if (code == PKT_CODE_WRITE) {
		handle_write_request(packet);

	} else {
		PX4_DEBUG("Unknown packet code: 0x%02x", code);
		send_error_response();
	}

	//PX4_DEBUG("Received packet: page=%u, offset=%u, code=0x%02x, count=%u",
	//	 packet.page, packet.offset, code, count);

}

void CuberedBridgeSecondary::send_response(IOPacket &packet)
{
	if (_serial_fd >= 0) {
		size_t packet_size = PKT_SIZE(packet);
		ssize_t bytes_written = ::write(_serial_fd, &packet, packet_size);

		if (bytes_written != (ssize_t)packet_size) {
			if (bytes_written < 0) {
				PX4_ERR("Write error: %s", strerror(errno));

			} else {
				PX4_WARN("Partial write: %zd of %zu bytes", bytes_written, packet_size);
			}
		}
	}
}

int CuberedBridgeSecondary::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
CubeRed IO Module - PX4IO simulation for CubeRed Secondary.

This module simulates PX4IO functionality on the CubeRed Secondary by providing
low-latency serial communication on ttyS4 with the CubeRed Primary.

The module runs as a dedicated task (not in work queue) to provide low-latency
polling and response on the serial port, mimicking the behavior of a real PX4IO
coprocessor.

### Implementation
The module runs in its own high-priority task and uses poll() with a short timeout
to achieve low latency communication with the CubeRed Primary. It processes
incoming commands and responds according to the PX4IO protocol.

### Examples
Start the cubered-io module:
$ cubered_bridge_secondary start

Stop the cubered-io module:
$ cubered_bridge_secondary stop

Check status:
$ cubered_bridge_secondary status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("cubered_bridge_secondary", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

bool CuberedBridgeSecondary::validate_crc(IOPacket &packet)
{
	uint8_t expected_crc = packet.crc;
	packet.crc = 0;
	uint8_t calculated_crc = calculate_crc(packet);
	packet.crc = expected_crc;

	return expected_crc == calculated_crc;
}

uint8_t CuberedBridgeSecondary::calculate_crc(IOPacket &packet)
{
	return crc_packet(&packet);
}

void CuberedBridgeSecondary::handle_read_request(IOPacket &packet)
{
	//PX4_INFO("Read request: page=%u, offset=%u, count=%u", packet.page, packet.offset, PKT_COUNT(packet));

	uint8_t offset = packet.offset;
	uint8_t count = PKT_COUNT(packet);

	// Prepare response packet
	IOPacket response;
	memset(&response, 0, sizeof(response));
	response.count_code = count | PKT_CODE_SUCCESS;
	response.page = packet.page;
	response.offset = packet.offset;

	// Handle different pages
	switch (packet.page) {
	case PX4IO_PAGE_CONFIG:
		// Copy requested config data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(config_page)/sizeof(config_page[0]); i++) {
			response.regs[i] = config_page[offset + i];
		}
		break;

	case PX4IO_PAGE_STATUS:
		//PX4_INFO("status_page size: %zu", sizeof(status_page)/sizeof(status_page[0]));
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(status_page)/sizeof(status_page[0]); i++) {
			response.regs[i] = status_page[offset + i];
			//PX4_INFO("  status_page[%u] = %u (0x%04x)", offset + i, status_page[offset + i], status_page[offset + i]);
		}
		//printf("responding with count %u\n", count);
		break;

	case PX4IO_PAGE_DIRECT_PWM:
		// Copy requested direct PWM data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(direct_pwm_page)/sizeof(direct_pwm_page[0]); i++) {
			response.regs[i] = direct_pwm_page[offset + i];
		}
		break;

	case PX4IO_PAGE_SERVOS:
		// Copy requested servo data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(servos_page)/sizeof(servos_page[0]); i++) {
			response.regs[i] = servos_page[offset + i];
		}
		break;

	case PX4IO_PAGE_RAW_RC_INPUT:
		// Copy requested RC input data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(raw_rc_input_page)/sizeof(raw_rc_input_page[0]); i++) {
			response.regs[i] = raw_rc_input_page[offset + i];
		}
		break;

	case PX4IO_PAGE_RAW_ADC_INPUT:
		// Copy requested ADC input data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(raw_adc_input_page)/sizeof(raw_adc_input_page[0]); i++) {
			response.regs[i] = raw_adc_input_page[offset + i];
		}
		break;

	case PX4IO_PAGE_PWM_INFO:
		// Copy requested PWM info data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(pwm_info_page)/sizeof(pwm_info_page[0]); i++) {
			response.regs[i] = pwm_info_page[offset + i];
		}
		break;

	case PX4IO_PAGE_SETUP:
		// Copy requested setup data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(setup_page)/sizeof(setup_page[0]); i++) {
			response.regs[i] = setup_page[offset + i];
		}
		break;

	case PX4IO_PAGE_DISARMED_PWM:
		// Copy requested disarmed PWM data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(disarmed_pwm_page)/sizeof(disarmed_pwm_page[0]); i++) {
			response.regs[i] = disarmed_pwm_page[offset + i];
		}
		break;

	case PX4IO_PAGE_FAILSAFE_PWM:
		// Copy requested failsafe PWM data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(failsafe_pwm_page)/sizeof(failsafe_pwm_page[0]); i++) {
			response.regs[i] = failsafe_pwm_page[offset + i];
		}
		break;

	default:
		// Unsupported page
		PX4_DEBUG("Unsupported page: %u", packet.page);
		send_error_response();
		return;
	}

	send_packet(response);
}

void CuberedBridgeSecondary::handle_write_request(IOPacket &packet)
{
	//PX4_DEBUG("Write request: page=%u, offset=%u, count=%u",
	//	 packet.page, packet.offset, PKT_COUNT(packet));

	// Only print unsupported pages
	if (packet.page != PX4IO_PAGE_SETUP &&
	    packet.page != PX4IO_PAGE_DIRECT_PWM &&
	    packet.page != PX4IO_PAGE_FAILSAFE_PWM &&
	    packet.page != PX4IO_PAGE_DISARMED_PWM) {
	}

	uint8_t offset = packet.offset;
	uint8_t count = PKT_COUNT(packet);

	// Handle different pages
	switch (packet.page) {
	case PX4IO_PAGE_STATUS:
		// Update status data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(status_page)/sizeof(status_page[0]); i++) {
			status_page[offset + i] = packet.regs[i];
		}
		// Check if arming status changed
		if (offset <= PX4IO_P_STATUS_FLAGS && (offset + count) > PX4IO_P_STATUS_FLAGS) {
			bool armed = (status_page[PX4IO_P_STATUS_FLAGS] & PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED) != 0;
			set_pwm_armed(armed);
		}
		break;

	case PX4IO_PAGE_DIRECT_PWM:
		{
			static unsigned debug_counter = 0;
			
			// Update direct PWM data
			for (uint8_t i = 0; i < count && (offset + i) < sizeof(direct_pwm_page)/sizeof(direct_pwm_page[0]); i++) {
				direct_pwm_page[offset + i] = packet.regs[i];
			}
			
			// Debug print every 400th call
			if (++debug_counter >= 400) {
				debug_counter = 0;
				printf("CuberedBridgeSecondary PWM in (offset=%u, count=%u, %s): ", offset, count, _pwm_armed ? "ARMED" : "DISARMED");
				for (uint8_t i = 0; i < 8 && i < sizeof(direct_pwm_page)/sizeof(direct_pwm_page[0]); i++) {
					printf("%u ", direct_pwm_page[i]);
				}
				printf("\n");
			}
			
			// Drive actual PWM outputs
			update_pwm_outputs();
		}
		break;

	case PX4IO_PAGE_SERVOS:
		// Update servo data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(servos_page)/sizeof(servos_page[0]); i++) {
			servos_page[offset + i] = packet.regs[i];
		}
		break;

	case PX4IO_PAGE_RAW_RC_INPUT:
		// Update RC input data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(raw_rc_input_page)/sizeof(raw_rc_input_page[0]); i++) {
			raw_rc_input_page[offset + i] = packet.regs[i];
		}
		break;

	case PX4IO_PAGE_RAW_ADC_INPUT:
		// Update ADC input data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(raw_adc_input_page)/sizeof(raw_adc_input_page[0]); i++) {
			raw_adc_input_page[offset + i] = packet.regs[i];
		}
		break;

	case PX4IO_PAGE_PWM_INFO:
		// Update PWM info data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(pwm_info_page)/sizeof(pwm_info_page[0]); i++) {
			pwm_info_page[offset + i] = packet.regs[i];
		}
		break;

	case PX4IO_PAGE_SETUP:
		// Update setup data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(setup_page)/sizeof(setup_page[0]); i++) {
			setup_page[offset + i] = packet.regs[i];
		}
		break;

	case PX4IO_PAGE_DISARMED_PWM:
		// Update disarmed PWM data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(disarmed_pwm_page)/sizeof(disarmed_pwm_page[0]); i++) {
			disarmed_pwm_page[offset + i] = packet.regs[i];
		}
		break;

	case PX4IO_PAGE_FAILSAFE_PWM:
		// Update failsafe PWM data
		for (uint8_t i = 0; i < count && (offset + i) < sizeof(failsafe_pwm_page)/sizeof(failsafe_pwm_page[0]); i++) {
			failsafe_pwm_page[offset + i] = packet.regs[i];
		}
		break;

	default:
		// For now, just acknowledge other write requests
		// TODO: Implement actual register writes as needed
		break;
	}

	// Send success response
	IOPacket response;
	memset(&response, 0, sizeof(response));
	response.count_code = 0 | PKT_CODE_SUCCESS;
	response.page = packet.page;
	response.offset = packet.offset;

	send_packet(response);
}

void CuberedBridgeSecondary::send_corrupt_response()
{
	IOPacket response;
	memset(&response, 0, sizeof(response));
	response.count_code = 0 | PKT_CODE_CORRUPT;
	send_packet(response);
}

void CuberedBridgeSecondary::send_error_response()
{
	IOPacket response;
	memset(&response, 0, sizeof(response));
	response.count_code = 0 | PKT_CODE_ERROR;
	send_packet(response);
}

void CuberedBridgeSecondary::send_packet(IOPacket &packet)
{
	// Calculate packet size
	//size_t packet_size = 4 + (PKT_COUNT(packet) * 2); // header + registers

	// Calculate and set CRC
	packet.crc = 0; // Clear CRC before calculation
	packet.crc = crc_packet(&packet);

	send_response(packet);
}

int CuberedBridgeSecondary::print_status()
{
	PX4_INFO("CuberedBridgeSecondary Status:");
	PX4_INFO("  Setup Page (50):");
	PX4_INFO("    Features: 0x%04x", setup_page[PX4IO_P_SETUP_FEATURES]);
	PX4_INFO("    Arming: 0x%04x", setup_page[PX4IO_P_SETUP_ARMING]);
	PX4_INFO("    PWM Rates: 0x%04x", setup_page[PX4IO_P_SETUP_PWM_RATES]);
	PX4_INFO("    Default Rate: %u Hz", setup_page[PX4IO_P_SETUP_PWM_DEFAULTRATE]);
	PX4_INFO("    Alt Rate: %u Hz", setup_page[PX4IO_P_SETUP_PWM_ALTRATE]);
	PX4_INFO("    SBUS Rate: %u Hz", setup_page[PX4IO_P_SETUP_SBUS_RATE]);
	PX4_INFO("    VServo Scale: %u", setup_page[PX4IO_P_SETUP_VSERVO_SCALE]);
	PX4_INFO("    Debug Level: %u", setup_page[PX4IO_P_SETUP_SET_DEBUG]);
	PX4_INFO("    Thermal: %u", setup_page[PX4IO_P_SETUP_THERMAL]);
	PX4_INFO("    Flight Termination: %u", setup_page[PX4IO_P_SETUP_ENABLE_FLIGHTTERMINATION]);

	PX4_INFO("  Direct PWM Page (54):");
	for (int i = 0; i < 8; i++) {
		PX4_INFO("    Channel %d: %u", i, pwm_info_page[i]);
	}

	PX4_INFO("  Failsafe PWM Page (55):");
	for (int i = 0; i < 8; i++) {
		PX4_INFO("    Channel %d: %u", i, failsafe_pwm_page[i]);
	}

	PX4_INFO("  Disarmed PWM Page (109):");
	for (int i = 0; i < 8; i++) {
		PX4_INFO("    Channel %d: %u", i, disarmed_pwm_page[i]);
	}

	return 0;
}

void CuberedBridgeSecondary::update_pwm_outputs()
{
	static unsigned update_counter = 0;
	
	if (!_pwm_initialized) {
		if (++update_counter % 400 == 0) {
			printf("CuberedBridgeSecondary: update_pwm_outputs called but PWM not initialized\n");
		}
		return;
	}

	// Output PWM values to hardware
	for (unsigned i = 0; i < DIRECT_PWM_OUTPUT_CHANNELS && i < sizeof(direct_pwm_page)/sizeof(direct_pwm_page[0]); i++) {
		if (_pwm_mask & (1 << i)) {
			int ret = up_pwm_servo_set(i, direct_pwm_page[i]);
			if (ret < 0 && update_counter % 400 == 0) {
				printf("CuberedBridgeSecondary: up_pwm_servo_set(%u, %u) failed: %d\n", i, direct_pwm_page[i], ret);
			}
		} else if (update_counter % 400 == 0) {
			printf("CuberedBridgeSecondary: Channel %u not in PWM mask (0x%02" PRIx32 ")\n", i, _pwm_mask);
		}
	}
	
	if (++update_counter % 400 == 0) {
		printf("CuberedBridgeSecondary: update_pwm_outputs called (%u times)\n", update_counter);
	}
}

void CuberedBridgeSecondary::set_pwm_armed(bool armed)
{
	if (_pwm_initialized && _pwm_armed != armed) {
		_pwm_armed = armed;
		// Don't change hardware arm state - PWM hardware stays always enabled
		// The flight control system sends appropriate values (disarmed/test/active)
		printf("CuberedBridgeSecondary: Flight control %s (mask=0x%02" PRIx32 ")\n", armed ? "ARMED" : "DISARMED", _pwm_mask);
	}
}

extern "C" __EXPORT int cubered_bridge_secondary_main(int argc, char *argv[])
{
	return ModuleBase::main(CuberedBridgeSecondary::desc, argc, argv);
}
