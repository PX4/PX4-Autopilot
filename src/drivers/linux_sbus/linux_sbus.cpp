/****************************************************************************
 *
 *   Copyright (C) 2017 Mark Charl. All rights reserved.
 *   Copyright (C) 2017 Fan.zhang. All rights reserved.
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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
#include "linux_sbus.h"

#include <stdlib.h>
#include <string.h>

using namespace linux_sbus;

//---------------------------------------------------------------------------------------------------------//
int RcInput::init()
{
	int i;

	/**
	 * initialize the data of each channel
	 */
	for (i = 0; i < input_rc_s::RC_INPUT_MAX_CHANNELS; ++i) {
		_data.values[i] = UINT16_MAX;
	}

	_rcinput_pub = orb_advertise(ORB_ID(input_rc), &_data);

	if (nullptr == _rcinput_pub) {
		PX4_WARN("error: advertise failed");
		return -1;
	}

	/**
	 * open the serial port
	 */
	_device_fd = open(_device, O_RDWR | O_NONBLOCK | O_CLOEXEC);

	if (-1 == _device_fd) {
		PX4_ERR("Open SBUS input %s failed, status %d \n", _device,
			(int) _device_fd);
		fflush(stdout);
		return -1;
	}

	struct termios2 tio { };

	if (0 != ioctl(_device_fd, TCGETS2, &tio)) {
		close(_device_fd);
		_device_fd = -1;
		return -1;
	}

	/**
	 * Setting serial port,8E2, non-blocking.100Kbps
	 */
	tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL
			 | IXON);
	tio.c_iflag |= (INPCK | IGNPAR);
	tio.c_oflag &= ~OPOST;
	tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	tio.c_cflag &= ~(CSIZE | CRTSCTS | PARODD | CBAUD);
	/**
	 * use BOTHER to specify speed directly in c_[io]speed member
	 */
	tio.c_cflag |= (CS8 | CSTOPB | CLOCAL | PARENB | BOTHER | CREAD);
	tio.c_ispeed = 100000;
	tio.c_ospeed = 100000;
	tio.c_cc[VMIN] = 25;
	tio.c_cc[VTIME] = 0;

	if (0 != ioctl(_device_fd, TCSETS2, &tio)) {
		close(_device_fd);
		_device_fd = -1;
		return -1;
	}

	return 0;
}
//---------------------------------------------------------------------------------------------------------//
int RcInput::start(char *device, int channels)
{
	int result = 0;
	strcpy(_device, device);
	PX4_INFO("Device %s , channels: %d \n", device, channels);
	_channels = channels;
	result = init();

	if (0 != result) {
		PX4_WARN("error: RC initialization failed");
		return -1;
	}

	_isRunning = true;

	ScheduleNow();

	if (result == -1) {
		_isRunning = false;
	}

	return result;
}
//---------------------------------------------------------------------------------------------------------//
void RcInput::stop()
{
	close(_device_fd);
	_shouldExit = true;
}

//---------------------------------------------------------------------------------------------------------//
void RcInput::Run()
{
	_measure();

	if (!_shouldExit) {
		ScheduleDelayed(RCINPUT_MEASURE_INTERVAL_US);
	}
}
//---------------------------------------------------------------------------------------------------------//
void RcInput::_measure(void)
{
	uint64_t ts;
	int nread;
	fd_set fds;
	FD_ZERO(&fds);
	FD_SET(_device_fd, &fds);
	/**
	 *error counter to count the lost frame
	 */
	int count = 0; //

	while (1) {
		nread = read(_device_fd, &_sbusData, sizeof(_sbusData));

		if (25 == nread) {
			/**
			 * Notice: most sbus rx device support sbus1
			 */
			if (0x0f == _sbusData[0] && 0x00 == _sbusData[24]) {
				break;
			}
		}

		++count;
		usleep(RCINPUT_MEASURE_INTERVAL_US);
	}

	/**
	 * parse sbus data to pwm
	 */
	_channels_data[0] =
		(uint16_t)(((_sbusData[1] | _sbusData[2] << 8) & 0x07FF)
			   * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	_channels_data[1] = (uint16_t)(((_sbusData[2] >> 3 | _sbusData[3] << 5)
					& 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	_channels_data[2] = (uint16_t)(((_sbusData[3] >> 6 | _sbusData[4] << 2
					 | _sbusData[5] << 10) & 0x07FF) * SBUS_SCALE_FACTOR + .5f)
			    + SBUS_SCALE_OFFSET;
	_channels_data[3] = (uint16_t)(((_sbusData[5] >> 1 | _sbusData[6] << 7)
					& 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	_channels_data[4] = (uint16_t)(((_sbusData[6] >> 4 | _sbusData[7] << 4)
					& 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	_channels_data[5] = (uint16_t)(((_sbusData[7] >> 7 | _sbusData[8] << 1
					 | _sbusData[9] << 9) & 0x07FF) * SBUS_SCALE_FACTOR + .5f)
			    + SBUS_SCALE_OFFSET;
	_channels_data[6] = (uint16_t)(((_sbusData[9] >> 2 | _sbusData[10] << 6)
					& 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	_channels_data[7] = (uint16_t)(((_sbusData[10] >> 5 | _sbusData[11] << 3)
					& 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET; // & the other 8 + 2 channels if you need them
	_channels_data[8] = (uint16_t)(((_sbusData[12] | _sbusData[13] << 8)
					& 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	_channels_data[9] = (uint16_t)(((_sbusData[13] >> 3 | _sbusData[14] << 5)
					& 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	_channels_data[10] = (uint16_t)(((_sbusData[14] >> 6 | _sbusData[15] << 2
					  | _sbusData[16] << 10) & 0x07FF) * SBUS_SCALE_FACTOR + .5f)
			     + SBUS_SCALE_OFFSET;
	_channels_data[11] = (uint16_t)(((_sbusData[16] >> 1 | _sbusData[17] << 7)
					 & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	_channels_data[12] = (uint16_t)(((_sbusData[17] >> 4 | _sbusData[18] << 4)
					 & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	_channels_data[13] = (uint16_t)(((_sbusData[18] >> 7 | _sbusData[19] << 1
					  | _sbusData[20] << 9) & 0x07FF) * SBUS_SCALE_FACTOR + .5f)
			     + SBUS_SCALE_OFFSET;
	_channels_data[14] = (uint16_t)(((_sbusData[20] >> 2 | _sbusData[21] << 6)
					 & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	_channels_data[15] = (uint16_t)(((_sbusData[21] >> 5 | _sbusData[22] << 3)
					 & 0x07FF) * SBUS_SCALE_FACTOR + .5f) + SBUS_SCALE_OFFSET;
	int i = 0;

	for (i = 0; i < _channels; ++i) {
		_data.values[i] = _channels_data[i];
	}

	ts = hrt_absolute_time();
	_data.timestamp = ts;
	_data.timestamp_last_signal = ts;
	_data.channel_count = _channels;
	_data.rssi = 100;
	_data.rc_lost_frame_count = count;
	_data.rc_total_frame_count = 1;
	_data.rc_ppm_frame_length = 0;
	_data.rc_failsafe = (_sbusData[23] & (1 << 3)) ? true : false;
	_data.rc_lost = (_sbusData[23] & (1 << 2)) ? true : false;
	_data.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;

	orb_publish(ORB_ID(input_rc), _rcinput_pub, &_data);
}
//---------------------------------------------------------------------------------------------------------//
/**
 * Print the correct usage.
 */
static void linux_sbus::usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s", reason);
	}

	PX4_INFO("Usage: linux_sbus {start|stop|status} -d <device>  -c <channel>");
}
//---------------------------------------------------------------------------------------------------------//
int linux_sbus_main(int argc, char **argv)
{
	int start;
	int command = -1;
	/**
	 * ttyS1 for default, it can  be changed through -d parameter
	 */
	char device[30] = "/dev/ttyS1";
	/**
	 * 8 channel for default setting, it can  be changed through -c parameter
	 */
	int max_channel = 8;

	/**
	 * Parse command line and get device and channels count from consolex
	 */
	for (start = 0; start < argc; ++start) {
		if (0 == strcmp(argv[start], "start")) {
			command = 0;
			continue;
		}

		if (0 == strcmp(argv[start], "stop")) {
			command = 1;
			continue;
		}

		if (0 == strcmp(argv[start], "status")) {
			command = 2;
			continue;
		}

		if (0 == strcmp(argv[start], "-d")) {
			if (argc > (start + 1)) {
				strcpy(device, argv[start + 1]);
			}

			continue;
		}

		if (0 == strcmp(argv[start], "-c")) {
			if (argc > (start + 1)) {
				max_channel = atoi(argv[start + 1]);
			}

			continue;
		}
	}

	/**
	 * Channels count can't be higher than 16;
	 */
	max_channel = (max_channel > 16) ? 16 : max_channel;

	if (0 == command) {
		if (nullptr != rc_input && rc_input->isRunning()) {
			PX4_WARN("running");
			return 0;
		}

		rc_input = new RcInput();

		/** Check if alloc worked. */
		if (nullptr == rc_input) {
			PX4_ERR("Sbus driver initialization failed");
			return -1;
		}

		int ret = rc_input->start(device, max_channel);

		if (ret != 0) {
			PX4_ERR("Linux sbus module failure");
		}

		return 0;
	}

	if (1 == command) {
		if (rc_input == nullptr || !rc_input->isRunning()) {
			PX4_WARN("Not running \n");
			/* this is not an error */
			return 0;
		}

		rc_input->stop();
		/**
		 * Wait for task to die
		 */
		int i = 0;

		do {
			/* wait up to 100ms */
			usleep(100000);
		} while (rc_input->isRunning() && ++i < 30);

		delete rc_input;
		rc_input = nullptr;
		return 0;
	}

	if (2 == command) {
		if (rc_input != nullptr && rc_input->isRunning()) {
			PX4_INFO("running");

		} else {
			PX4_INFO("Not running \n");
		}

		return 0;
	}

	linux_sbus::usage(
		"Usage: linux_sbus start|stop|status -d <device>  -c <channel>");
	return 0;
}
//---------------------------------------------------------------------------------------------------------//
