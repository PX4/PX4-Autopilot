/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file esc_4way.cpp
 *
 * Host side: MSP v1 ('$M<' size cmd payload xor) until MSP_SET_PASSTHROUGH, then 4way frames
 * ('/' cmd addrH addrL n params crc16-xmodem, n = 0 means 256), replied as
 * '.' cmd addrH addrL n params ack crc16. A '$' frame is MSP and a '/' frame is 4way in either mode.
 *
 * ESC side: the BLHeli/AM32 bootloader at 19200 baud, half duplex on the ESC signal pin.
 * Bootloader frames are a command or payload followed by a CRC16 (poly 0xA001, LSB first,
 * low byte first). Each frame goes out in one bitbang_uart_write() because the bootloader
 * drops a frame on an inter-byte gap of more than about 260 us.
 *
 * A session ends in a reboot (InterfaceExit after a device command, or the serial device going
 * away), which restores the output drivers stopped at the first valid request.
 */

#include "esc_4way.hpp"

#include <board_config.h>
#include <drivers/drv_bitbang_uart.h>
#include <drivers/drv_hrt.h>
#include <parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/shutdown.h>
#include <px4_platform_common/tasks.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>

#include <ctype.h>
#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#if defined(CONFIG_CDCACM)
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/cdcacm.h>
#include <sys/ioctl.h>
#endif // CONFIG_CDCACM

extern "C" {
#include <lib/crc/crc.h>
	int dshot_main(int argc, char *argv[]);
	int pwm_out_main(int argc, char *argv[]);
}

ModuleBase::Descriptor Esc4way::desc{task_spawn, custom_command, print_usage};

static constexpr uint8_t MOTOR_COUNT = DIRECT_PWM_OUTPUT_CHANNELS < 8 ? DIRECT_PWM_OUTPUT_CHANNELS : 8;

// 4way interface
static constexpr uint8_t FOURWAY_REQ = 0x2F;
static constexpr uint8_t FOURWAY_REPLY = 0x2E;
static constexpr uint8_t TEST_ALIVE = 0x30;
static constexpr uint8_t GET_PROTOCOL_VERSION = 0x31;
static constexpr uint8_t GET_NAME = 0x32;
static constexpr uint8_t GET_IF_VERSION = 0x33;
static constexpr uint8_t EXIT = 0x34;
static constexpr uint8_t DEVICE_RESET = 0x35;
static constexpr uint8_t DEVICE_INIT_FLASH = 0x37;
static constexpr uint8_t DEVICE_PAGE_ERASE = 0x39;
static constexpr uint8_t DEVICE_READ = 0x3A;
static constexpr uint8_t DEVICE_WRITE = 0x3B;
static constexpr uint8_t SET_MODE = 0x3F;
static constexpr uint8_t ACK_OK = 0x00;
static constexpr uint8_t ACK_I_INVALID_CMD = 0x02;
static constexpr uint8_t ACK_I_INVALID_CRC = 0x03;
static constexpr uint8_t ACK_I_INVALID_CHANNEL = 0x08;
static constexpr uint8_t ACK_I_INVALID_PARAM = 0x09;
static constexpr uint8_t ACK_D_GENERAL_ERROR = 0x0F;
static constexpr uint8_t MODE_SIL_BLB = 1;	// SiLabs BLHeli_S / Bluejay bootloader
static constexpr uint8_t MODE_ARM_BLB = 4;	// BLHeli_32 / AM32 bootloader

// BLHeli bootloader
static constexpr uint8_t BL_RUN = 0x00;
static constexpr uint8_t BL_PROG_FLASH = 0x01;
static constexpr uint8_t BL_ERASE_FLASH = 0x02;
static constexpr uint8_t BL_READ_FLASH = 0x03;
static constexpr uint8_t BL_KEEP_ALIVE = 0xFD;
static constexpr uint8_t BL_SET_BUFFER = 0xFE;
static constexpr uint8_t BL_SET_ADDRESS = 0xFF;
static constexpr uint8_t BL_ACK_OK = 0x30;
static constexpr uint32_t BL_NO_ACK_US = 2000;
static constexpr uint32_t BL_ACK_TIMEOUT_US = 50000;
static constexpr uint32_t BL_PROG_TIMEOUT_US = 1000000;
static constexpr uint32_t BL_READ_TIMEOUT_US = 500000;

static constexpr uint32_t SESSION_CONNECT_US = 1500000;	// inside the configurator's 2 s per InitFlash try
static constexpr uint32_t REBOOT_DELAY_US = 500000;	// lets the last reply drain before the USB drops
static constexpr int RX_GAP_MS = 100;			// a longer gap inside a host frame drops it
static constexpr uint32_t UART_RETRY_US = 10000;	// a pending bidirectional DShot capture completes well within this
static constexpr uint32_t DISARM_POLL_US = 100000;
static constexpr hrt_abstime DTR_CLOSED_US = 1000000;	// DTR low this long: the host closed the port
static constexpr int32_t OUTPUT_FUNCTION_MOTOR1 = 101;	// PWM_*_FUNCn value of Motor1, Motor2 is 102, ...

// MSP replies, byte-exact as the configurators parse them without bounds checks
static constexpr uint8_t api_version[] {0, 1, 42};
static constexpr uint8_t fc_variant[] {'P', 'X', '4', ' '};
static constexpr uint8_t fc_version[] {3, 3, 1};
static constexpr uint8_t board_info[] {'P', 'X', '4', ' ', 0, 0, 0};
static constexpr char build_info[] = __DATE__ __TIME__;	// "Mmm dd yyyyhh:mm:ss", 19 bytes
// cycle time, i2c errors, sensors, modes (u32), profile, load, gyro cycle, extra mode bytes,
// arming flag count 26, arming disable flags (u32) with RX_FAILSAFE (bit 2): no motor test,
// reboot required (esc-configurator reads it and fails the ESC read without it)
static constexpr uint8_t status[22] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 26, 1 << 2, 0, 0, 0, 0};
// 3D deadband low 1406, high 1514, neutral 1460
static constexpr uint8_t motor_3d_config[] {0x7E, 0x05, 0xEA, 0x05, 0xB4, 0x05};
// min throttle 1070, max throttle 2000, min command 1000, ESC count (set at start), poles 14, no 3D, no ESC sensor
static uint8_t motor_config[] {0x2E, 0x04, 0xD0, 0x07, 0xE8, 0x03, 0, 14, 0, 0};
static uint8_t uid[12];	// the CPU unique id, set at start
static constexpr uint8_t NO_OUTPUT = 0xFF;
static uint8_t esc_output[MOTOR_COUNT];	// 4way ESC index -> output channel, set at start
static constexpr uint8_t zeros[16] {};

static const struct {
	uint8_t cmd;
	const uint8_t *data;
	uint8_t len;
} msp_replies[] {
	{1, api_version, sizeof(api_version)},				// MSP_API_VERSION
	{2, fc_variant, sizeof(fc_variant)},				// MSP_FC_VARIANT
	{3, fc_version, sizeof(fc_version)},				// MSP_FC_VERSION
	{4, board_info, sizeof(board_info)},				// MSP_BOARD_INFO
	{5, (const uint8_t *)build_info, sizeof(build_info) - 1},	// MSP_BUILD_INFO
	{36, zeros, 4},							// MSP_FEATURE_CONFIG
	{101, status, sizeof(status)},					// MSP_STATUS
	{104, zeros, 16},						// MSP_MOTOR
	{124, motor_3d_config, sizeof(motor_3d_config)},		// MSP_MOTOR_3D_CONFIG
	{131, motor_config, sizeof(motor_config)},			// MSP_MOTOR_CONFIG
	{160, uid, sizeof(uid)},					// MSP_UID
	{214, zeros, 0},						// MSP_SET_MOTOR: acked, motors are never driven
	{245, &motor_config[6], 1},					// MSP_SET_PASSTHROUGH: ESC count, then 4way
};

static uint16_t bl_crc(const uint8_t *data, size_t len)
{
	uint16_t crc = 0;

	for (size_t i = 0; i < len; i++) {
		uint8_t b = data[i];

		for (int bit = 0; bit < 8; bit++, b >>= 1) {
			crc = ((b ^ crc) & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
		}
	}

	return crc;
}

// Only the esc_4way task calls this
static bool armed()
{
	static uORB::Subscription armed_sub{ORB_ID(actuator_armed)};
	actuator_armed_s armed{};
	armed_sub.copy(&armed);
	return armed.armed;
}

// Starts the ESC pin UART, stopping dshot and pwm_out first when they own the ESC pins
static int uart_begin()
{
	int ret = bitbang_uart_init(19200, true);

	if (ret == -EBUSY) {
		char stop[] = "stop";
#if defined(CONFIG_DRIVERS_DSHOT)
		char dshot[] = "dshot";
		char *dshot_argv[] {dshot, stop, nullptr};
		dshot_main(2, dshot_argv);
#endif
#if defined(CONFIG_DRIVERS_PWM_OUT)
		char pwm_out[] = "pwm_out";
		char *pwm_out_argv[] {pwm_out, stop, nullptr};
		pwm_out_main(2, pwm_out_argv);
#endif
		(void)stop;
		ret = bitbang_uart_init(19200, true);

		// A bidirectional DShot capture completion that was already scheduled can claim a pin again
		// right after the stop
		if (ret == -EBUSY) {
			px4_usleep(UART_RETRY_US);
			ret = bitbang_uart_init(19200, true);
		}
	}

	if (ret != 0) {
		PX4_ERR("ESC pin UART init failed (%d)", ret);
	}

	return ret;
}

// Raw binary stream: no echo, no line discipline, no output processing
static int open_raw(const char *device)
{
	int fd = open(device, O_RDWR | O_NOCTTY);
	termios t{};

	if (fd >= 0 && tcgetattr(fd, &t) == 0) {
		cfmakeraw(&t);
		tcsetattr(fd, TCSANOW, &t);
	}

	return fd;
}

static bool usb_connected()
{
#if defined(CONFIG_DRIVERS_CDCACM_AUTOSTART)
	return board_read_VBUS_state() == PX4_OK;
#else
	return true;	// no cdcacm_autostart to start a new instance: keep the port
#endif
}

Esc4way::~Esc4way()
{
	if (_fd >= 0) {
		close(_fd);
	}
}

// Before the first valid request nothing was stopped. A host-side USB reset (VBUS still present,
// e.g. a usbipd attach) only needs the port reopened; after an unplug the task exits and
// cdcacm_autostart starts a new instance when the USB port comes back.
void Esc4way::reopen()
{
	close(_fd);
	_fd = -1;
	_dtr_seen = false;
	_dtr_low_since = 0;

	do {
		px4_usleep(200000);
	} while (!should_exit() && usb_connected() && (_fd = open_raw(_device)) < 0);

	if (_fd < 0) {
		request_stop();
	}
}

bool Esc4way::receive(uint8_t *buf, size_t len)
{
	while (len > 0 && _pending_pos < _pending_len) {
		*buf++ = _pending[_pending_pos++];
		len--;
	}

	while (len > 0) {
		pollfd fds{_fd, POLLIN, 0};

		if (poll(&fds, 1, RX_GAP_MS) <= 0) {
			return false;
		}

		ssize_t n = read(_fd, buf, len);

		if (n < 0 && errno == EINTR) {
			continue;
		}

		if (n <= 0) {
			// The serial device went away
			if (_session) {
				_reboot = true;

			} else {
				reopen();
			}

			return false;
		}

		buf += n;
		len -= n;
	}

	return true;
}

// A USB host drops DTR when it closes the port, e.g. a configurator tab closed without Disconnect.
// Only a drop that lasts, after DTR was seen raised in this session, counts: a host that never
// raises it, or one that lowers it briefly, is not affected.
bool Esc4way::host_closed()
{
#if defined(CONFIG_CDCACM)
	int line = 0;

	if (ioctl(_fd, CAIOC_GETCTRLLINE, (unsigned long)&line) == 0) {
		if (line & CDCACM_UART_DTR) {
			_dtr_seen = true;
			_dtr_low_since = 0;

		} else if (_dtr_seen) {
			if (_dtr_low_since == 0) {
				_dtr_low_since = hrt_absolute_time();
			}

			return hrt_elapsed_time(&_dtr_low_since) > DTR_CLOSED_US;
		}
	}

#endif // CONFIG_CDCACM
	return false;
}

// The ESC app enters its bootloader 2 s after its signal stops and the configurator allows two
// 2 s InitFlash tries, so the outputs are stopped and the lines driven high at the first request.
bool Esc4way::start_session()
{
	if (!_session) {
		if (armed()) {
			return false;
		}

		// only the request that started the session is answered, and only this host's DTR counts
		tcflush(_fd, TCIFLUSH);
		_pending_len = 0;
		_dtr_seen = false;
		_uart = uart_begin() == 0;
		_session = true;
		PX4_INFO("ESC configuration session started%s", _uart ? ", outputs off until reboot" : " without ESC access");
	}

	return true;
}

void Esc4way::run()
{
	// Requests of an earlier host run are not answered. Bytes that cdcacm_autostart read belong to
	// the current run, whose host is waiting for their reply.
	if (_pending_len == 0) {
		tcflush(_fd, TCIFLUSH);
	}

	while (!should_exit() && !_reboot) {
		uint8_t c;

		if (receive(&c, 1)) {
			if (c == '$') {
				handle_msp();

			} else if (c == FOURWAY_REQ) {
				handle_4way();
			}

		} else if (_served) {
			// Idle: the ESC pin UART leaves the line receiving after an exchange and drives it high
			// again (never low) on a read once it has been quiet for a while. Only after a device
			// command, so the read does not switch an untouched channel to receive.
			bitbang_uart_read(_esc.channel, &c, 1, 1);
		}

		if (host_closed() && _session) {
			PX4_INFO("host closed %s", _device);
			_reboot = true;
		}
	}

	if (_uart) {
		bitbang_uart_deinit();
	}

	// Never reboot an armed vehicle (motors on other outputs, e.g. PX4IO or CAN): wait for disarm
	if (_reboot && armed()) {
		PX4_WARN("ESC configuration session ended while armed, rebooting after disarm");

		while (armed() && !should_exit()) {
			px4_usleep(DISARM_POLL_US);
		}
	}

	if (_reboot && !armed()) {
#if defined(CONFIG_BOARDCTL_RESET)
		PX4_INFO("ESC configuration session ended, rebooting");
		px4_reboot_request(REBOOT_REQUEST, REBOOT_DELAY_US);
#else
		PX4_INFO("ESC configuration session ended, reboot to restore the outputs");
#endif // CONFIG_BOARDCTL_RESET

	} else if (_session) {
		PX4_WARN("ESC configuration session stopped, motor outputs stay off until reboot");
	}
}

void Esc4way::handle_msp()
{
	// 'M' '<' size cmd payload checksum
	uint8_t *f = _frame;

	if (!receive(f, 4) || f[0] != 'M' || f[1] != '<' || !receive(f + 4, f[2] + 1)) {
		return;
	}

	const uint8_t size = f[2];
	const uint8_t cmd = f[3];
	uint8_t x = 0;

	for (int i = 2; i < 4 + size; i++) {
		x ^= f[i];
	}

	if (x != f[4 + size] || !start_session()) {
		return;
	}

	_fourway = cmd == 245;	// MSP_SET_PASSTHROUGH

	if (cmd == 130) {	// MSP_BATTERY_STATE
		// cells, capacity 0, voltage (0.1 V), drawn mAh 0, current 0, state ok, voltage (0.01 V); all zero
		// without a battery. The configurators read all 11 bytes without a length check.
		static uORB::Subscription battery_sub{ORB_ID(battery_status)};
		battery_status_s b{};
		battery_sub.copy(&b);
		const uint16_t cv = b.connected ? (uint16_t)(b.voltage_v * 100.f) : 0;
		const uint8_t battery[11] {b.connected ? b.cell_count : (uint8_t)0, 0, 0, (uint8_t)(cv > 2550 ? 255 : cv / 10),
					   0, 0, 0, 0, 0, (uint8_t)cv, (uint8_t)(cv >> 8)
					  };	// the 0.1 V byte saturates above 25.5 V
		send_msp(cmd, battery, sizeof(battery));
		return;
	}

	for (const auto &r : msp_replies) {
		if (r.cmd == cmd) {
			send_msp(cmd, r.data, r.len);
			return;
		}
	}

	send_msp(cmd, zeros, 0, '!');
}

void Esc4way::send_msp(uint8_t cmd, const uint8_t *payload, uint8_t size, uint8_t direction)
{
	uint8_t *r = _reply;
	r[0] = '$';
	r[1] = 'M';
	r[2] = direction;
	r[3] = size;
	r[4] = cmd;
	memcpy(&r[5], payload, size);
	uint8_t x = 0;

	for (int i = 3; i < 5 + size; i++) {
		x ^= r[i];
	}

	r[5 + size] = x;
	write(_fd, r, 6 + size);
}

void Esc4way::handle_4way()
{
	uint8_t *f = _frame;
	f[0] = FOURWAY_REQ;

	if (!receive(&f[1], 4) || !receive(&f[5], (f[4] ? f[4] : 256) + 2)) {
		return;
	}

	const uint8_t cmd = f[1];
	const uint16_t addr = (f[2] << 8) | f[3];
	const size_t n = f[4] ? f[4] : 256;
	uint8_t *params = &f[5];
	const uint16_t crc = (params[n] << 8) | params[n + 1];

	if (crc16_signature(0, 5 + n, f) != crc) {
		send_4way(cmd, addr, nullptr, 0, ACK_I_INVALID_CRC);
		return;
	}

	// The configurators send InterfaceExit in their connect recovery before any passthrough and
	// expect an undisturbed MSP stream: in MSP mode it is ignored. It ends a session that ran a
	// device command, or one without ESC access, where the outputs may already be stopped.
	if (cmd == EXIT) {
		if (_fourway) {
			send_4way(cmd, addr, nullptr, 0, ACK_OK);
			_reboot = _served || (_session && !_uart);
			_fourway = false;
		}

		return;
	}

	if (!start_session()) {
		return;
	}

	_fourway = true;

	// DeviceReadEEprom/DeviceWriteEEprom are for Atmel ESCs only, answered as invalid like Betaflight
	// does in SiLabs and ARM mode
	const bool device_cmd = cmd == DEVICE_RESET || cmd == DEVICE_INIT_FLASH || cmd == DEVICE_PAGE_ERASE
				|| cmd == DEVICE_READ || cmd == DEVICE_WRITE;

	if (device_cmd && (!_uart || armed())) {
		send_4way(cmd, addr, nullptr, 0, ACK_D_GENERAL_ERROR);
		return;
	}

	_served |= device_cmd;
	uint8_t ack = ACK_OK;

	switch (cmd) {
	case TEST_ALIVE:
		// No exchange on the ESC pin while armed
		ack = (_esc.info[3] == 0 || (!armed() && _esc.keep_alive())) ? ACK_OK : ACK_D_GENERAL_ERROR;
		send_4way(cmd, addr, nullptr, 0, ack);
		break;

	case GET_PROTOCOL_VERSION: {
			static constexpr uint8_t version = 108;
			send_4way(cmd, addr, &version, 1, ack);
			break;
		}

	case GET_NAME:
		send_4way(cmd, addr, (const uint8_t *)"PX4", 3, ack);
		break;

	case GET_IF_VERSION: {
			static constexpr uint8_t version[] {200, 6};	// Betaflight's interface version 20.0.06
			send_4way(cmd, addr, version, sizeof(version), ack);
			break;
		}

	case SET_MODE:
		send_4way(cmd, addr, nullptr, 0, (params[0] == MODE_SIL_BLB || params[0] == MODE_ARM_BLB) ? ACK_OK : ACK_I_INVALID_PARAM);
		break;

	case DEVICE_RESET:

		// The app starts and drops back into the bootloader 2 s after the line goes quiet;
		// the idle loop keeps the line driven high for that.
		if (params[0] >= motor_config[6] || esc_output[params[0]] == NO_OUTPUT) {
			send_4way(cmd, addr, nullptr, 0, ACK_I_INVALID_CHANNEL);
			break;
		}

		_esc.channel = esc_output[params[0]];
		_esc.info[3] = 0;
		send_4way(cmd, addr, nullptr, 0, _esc.run() ? ACK_OK : ACK_D_GENERAL_ERROR);
		break;

	case DEVICE_INIT_FLASH:
		if (params[0] >= motor_config[6] || esc_output[params[0]] == NO_OUTPUT) {
			memset(_esc.info, 0, sizeof(_esc.info));
			ack = ACK_I_INVALID_CHANNEL;

		} else {
			ack = _esc.connect(esc_output[params[0]], SESSION_CONNECT_US) ? ACK_OK : ACK_D_GENERAL_ERROR;
		}

		send_4way(cmd, 0, _esc.info, sizeof(_esc.info), ack);
		break;

	case DEVICE_READ: {
			const size_t len = params[0] ? params[0] : 256;

			if (_esc.read(BL_READ_FLASH, addr, len)) {
				send_4way(cmd, addr, _esc.buf, len, ack);

			} else {
				send_4way(cmd, addr, nullptr, 0, ACK_D_GENERAL_ERROR);
			}

			break;
		}

	case DEVICE_PAGE_ERASE: {
			// Page number in the first parameter; EFM8 pages are 512 bytes, ARM ESC pages 1024
			const uint32_t page_addr = params[0] * (_esc.info[3] == MODE_SIL_BLB ? 512u : 1024u);
			ack = page_addr > 0xFFFF ? ACK_I_INVALID_PARAM : _esc.erase(page_addr) ? ACK_OK : ACK_D_GENERAL_ERROR;
			send_4way(cmd, addr, nullptr, 0, ack);
			break;
		}

	case DEVICE_WRITE:
		ack = _esc.write(BL_PROG_FLASH, addr, params, n) ? ACK_OK : ACK_D_GENERAL_ERROR;
		send_4way(cmd, addr, nullptr, 0, ack);
		break;

	default:
		send_4way(cmd, addr, nullptr, 0, ACK_I_INVALID_CMD);
		break;
	}
}

void Esc4way::send_4way(uint8_t cmd, uint16_t addr, const uint8_t *params, size_t n, uint8_t ack)
{
	static constexpr uint8_t none = 0;

	if (n == 0) {
		params = &none;	// a reply carries at least one parameter byte
		n = 1;
	}

	uint8_t *r = _reply;
	r[0] = FOURWAY_REPLY;
	r[1] = cmd;
	r[2] = addr >> 8;
	r[3] = addr & 0xFF;
	r[4] = n & 0xFF;
	memcpy(&r[5], params, n);
	r[5 + n] = ack;
	const uint16_t crc = crc16_signature(0, 6 + n, r);
	r[6 + n] = crc >> 8;
	r[7 + n] = crc & 0xFF;
	write(_fd, r, 8 + n);
}

// Appends the CRC in place: frame needs two spare bytes after len
bool EscBootloader::send(uint8_t *frame, size_t len)
{
	const uint16_t crc = bl_crc(frame, len);
	frame[len] = crc & 0xFF;
	frame[len + 1] = crc >> 8;
	return bitbang_uart_write(channel, frame, len + 2) == (int)(len + 2);
}

// Bootloader reply byte, -1 if none
int EscBootloader::ack(uint32_t timeout_us)
{
	uint8_t a;
	return bitbang_uart_read(channel, &a, 1, timeout_us) == 1 ? a : -1;
}

bool EscBootloader::init()
{
	// 0xF4 0x7D is the bootloader CRC of "BLHeli"
	static constexpr uint8_t init[] {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x0D, 'B', 'L', 'H', 'e', 'l', 'i', 0xF4, 0x7D};
	// '4' '7' '1' pinCode sigHi sigLo bootloader version pages ack
	uint8_t *r = buf;

	if (bitbang_uart_write(channel, init, sizeof(init)) != (int)sizeof(init)
	    || bitbang_uart_read(channel, r, 9, BL_ACK_TIMEOUT_US) != 9
	    || memcmp(r, "471", 3) != 0 || r[8] != BL_ACK_OK) {
		return false;
	}

	// Betaflight's device match: SiLabs (BLHeli_S, Bluejay) or ARM (BLHeli_32, AM32: MCU id high byte
	// 0x01..0x8F, low byte 0x06). Atmel BLHeli and unknown ids are not served by these configurators.
	const uint16_t sig = (r[4] << 8) | r[5];

	if (sig > 0xE800 && sig < 0xF900) {
		info[3] = MODE_SIL_BLB;

	} else if (r[4] > 0x00 && r[4] < 0x90 && r[5] == 0x06) {
		info[3] = MODE_ARM_BLB;

	} else {
		return false;
	}

	info[0] = r[5];
	info[1] = r[4];
	info[2] = r[3];
	return true;
}

// An ESC app drops into its bootloader up to ~3 s after the signal stops; keep probing. Between
// probes the line is driven high (the AM32 bootloader leaves on a low line), and the 250 ms gap
// keeps the probes from looking like a signal to the ESC application.
bool EscBootloader::connect(uint8_t ch, uint32_t timeout_us)
{
	channel = ch;
	const hrt_abstime start = hrt_absolute_time();
	bool ok;

	while (!(ok = init()) && hrt_elapsed_time(&start) < timeout_us) {
		bitbang_uart_idle(channel);
		px4_usleep(250000);
	}

	if (!ok) {
		memset(info, 0, sizeof(info));
	}

	return ok;
}

bool EscBootloader::set_address(uint16_t addr)
{
	uint8_t frame[6] {BL_SET_ADDRESS, 0, (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF)};
	return send(frame, 4) && ack(BL_ACK_TIMEOUT_US) == BL_ACK_OK;
}

// n bytes into buf, followed by the CRC and the ack
bool EscBootloader::read(uint8_t cmd, uint16_t addr, size_t n)
{
	uint8_t frame[4] {cmd, (uint8_t)(n & 0xFF)};

	return set_address(addr) && send(frame, 2)
	       && bitbang_uart_read(channel, buf, n + 3, BL_READ_TIMEOUT_US) == (int)(n + 3)
	       && bl_crc(buf, n) == (buf[n] | (buf[n + 1] << 8)) && buf[n + 2] == BL_ACK_OK;
}

bool EscBootloader::write(uint8_t cmd, uint16_t addr, uint8_t *data, size_t n)
{
	uint8_t set_buffer[6] {BL_SET_BUFFER, 0, (uint8_t)(n >> 8), (uint8_t)(n & 0xFF)};
	uint8_t prog[4] {cmd, 1};

	// SET_BUFFER is not answered: wait briefly and drain a stray byte. The payload is acked.
	return set_address(addr) && send(set_buffer, 4) && (ack(BL_NO_ACK_US), true)
	       && send(data, n) && ack(BL_ACK_TIMEOUT_US) == BL_ACK_OK
	       && send(prog, 2) && ack(BL_PROG_TIMEOUT_US) == BL_ACK_OK;
}

bool EscBootloader::erase(uint16_t addr)
{
	uint8_t frame[4] {BL_ERASE_FLASH, 1};
	return info[3] != 0 && set_address(addr) && send(frame, 2) && ack(BL_PROG_TIMEOUT_US) == BL_ACK_OK;
}

bool EscBootloader::run()
{
	uint8_t frame[4] {BL_RUN, 0};
	return send(frame, 2);
}

// The bootloader answers the keep alive with a bad command ack
bool EscBootloader::keep_alive()
{
	uint8_t frame[4] {BL_KEEP_ALIVE, 0};
	return send(frame, 2) && ack(BL_ACK_TIMEOUT_US) >= 0;
}

int Esc4way::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return Esc4way::instantiate(ac, av);
	}, argc, argv);
}

int Esc4way::task_spawn(int argc, char *argv[])
{
	// The ESC pin UART runs the bootloader frames from its interrupt, so this task's priority
	// only sets how fast requests are answered
	int task_id = px4_task_spawn_cmd("esc_4way", SCHED_DEFAULT, SCHED_PRIORITY_ACTUATOR_OUTPUTS,
					 PX4_STACK_ADJUSTED(2048), run_trampoline, (char *const *)argv);

	if (task_id < 0) {
		return -errno;
	}

	desc.task_id = task_id;
	return 0;
}

Esc4way *Esc4way::instantiate(int argc, char *argv[])
{
	const char *device = nullptr;
	const char *request = "";
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:r:", &myoptind, &myoptarg)) != EOF) {
		if (ch == 'd') {
			device = myoptarg;

		} else if (ch == 'r') {
			request = myoptarg;
		}
	}

	uint8_t pending[sizeof(_pending)];
	size_t pending_len = 0;
	const size_t hex_len = strlen(request);

	for (size_t i = 0; i < hex_len; i += 2) {
		if (hex_len % 2 != 0 || hex_len > 2 * sizeof(pending) || !isxdigit(request[i]) || !isxdigit(request[i + 1])) {
			PX4_ERR("invalid request hex: %s", request);
			return nullptr;
		}

		const char byte[3] {request[i], request[i + 1], '\0'};
		pending[pending_len++] = strtoul(byte, nullptr, 16);
	}

	if (!device) {
		print_usage("missing device");
		return nullptr;
	}

	if (strlen(device) >= sizeof(_device)) {
		PX4_ERR("device path too long: %s", device);
		return nullptr;
	}

	int fd = open_raw(device);

	if (fd < 0) {
		PX4_ERR("open %s failed (%d)", device, errno);
		return nullptr;
	}

	// The configurators number ESCs like Betaflight's motors: ESC n is the output mapped to Motor n+1,
	// and the ESC count is the highest mapped motor. Without any motor mapped,
	// ESC n is output n.
	uint8_t count = 0;
	memset(esc_output, NO_OUTPUT, sizeof(esc_output));

	for (int i = 0; i < MOTOR_COUNT; i++) {
		char name[17];
		snprintf(name, sizeof(name), PARAM_PREFIX "_FUNC%d", i + 1);
		int32_t func = 0;
		param_get(param_find(name), &func);
		const int motor = func - OUTPUT_FUNCTION_MOTOR1;

		if (motor >= 0 && motor < MOTOR_COUNT && esc_output[motor] == NO_OUTPUT) {
			esc_output[motor] = i;

			if (motor + 1 > count) {
				count = motor + 1;
			}
		}
	}

	if (count == 0) {
		for (int i = 0; i < MOTOR_COUNT; i++) {
			esc_output[i] = i;
		}

		count = MOTOR_COUNT;
	}

	motor_config[6] = count;

	px4_guid_t guid{};
	board_get_px4_guid(guid);
	memcpy(uid, guid + PX4_GUID_BYTE_LENGTH - sizeof(uid), sizeof(uid));	// the CPU unique id is at the end

	return new Esc4way(fd, device, pending, pending_len);
}

int Esc4way::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
ESC configuration server for the BLHeli/AM32 ESC configurators (am32.ca, esc-configurator.com).

It answers MSP and the 4way interface on a serial device and runs the ESC bootloader protocol
on the ESC signal pins. At the first valid request it stops the dshot and pwm_out drivers; the
session ends with a reboot when the configurator disconnects, the USB host closes the port or
the serial device goes away. Requests are ignored while armed.

Started by cdcacm_autostart on the USB port when `PASSTHRU_EN` is 2, or manually on any serial port.

### Examples
$ esc_4way start -d /dev/ttyACM0
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("esc_4way", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_STRING('r', nullptr, "<hex>", "Request bytes already read from the device, answered first", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int esc_4way_main(int argc, char *argv[])
{
	return ModuleBase::main(Esc4way::desc, argc, argv);
}
