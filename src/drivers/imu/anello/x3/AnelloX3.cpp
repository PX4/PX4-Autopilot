/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "AnelloX3.hpp"

#include <lib/drivers/device/Device.hpp>
#include <fcntl.h>
#include <poll.h>

// /dev/serial/by-id/usb-FTDI_TTL232R-3V3_FTGCVVU0-if00-port0 -> ../../ttyUSB1
// /dev/serial/by-id/usb-FTDI_USB-RS422_Cable_FT7UFGP8-if00-port0 -> ../../ttyUSB0

// Constructor
AnelloX3::AnelloX3(const char *port, uint8_t rotation) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	ModuleParams(nullptr),
	_px4_accel_og(0, static_cast<enum Rotation>(rotation)),
	_px4_gyro_og(0, static_cast<enum Rotation>(rotation)),
	_px4_accel(0, static_cast<enum Rotation>(rotation)),
	_px4_gyro(0, static_cast<enum Rotation>(rotation)),
	_px4_mag(0, static_cast<enum Rotation>(rotation))
{

	// Update parameters from storage
	ModuleParams::updateParams();

	// store port name
	strncpy(_port, port, sizeof(_port) - 1);

	// enforce null termination
	_port[sizeof(_port) - 1] = '\0';
	PX4_INFO("port: %s", _port);

	const uint8_t bus_num = atoi(&_port[strlen(_port) - 1]); // Assuming '/dev/ttySx'

	// device id
	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.bus = math::constrain<uint8_t>(bus_num, 0, 9);

	// optical gyro (DRV_IMU_DEVTYPE_ANELLO_X3_OG)
	device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ANELLO_X3_OG;
	_px4_accel_og.set_device_id(device_id.devid);
	_px4_gyro_og.set_device_id(device_id.devid);

	// mems gyro (DRV_IMU_DEVTYPE_ANELLO_X3)
	device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ANELLO_X3;

	_px4_accel.set_device_id(device_id.devid);
	_px4_gyro.set_device_id(device_id.devid);

	// magnetometer
	device_id.devid_s.devtype = DRV_IMU_DEVTYPE_ANELLO_X3_MAG;
	_px4_mag.set_device_id(device_id.devid);

	// Anello: According to Wolfram Alpha local vertical gravity in Santa Clara, CA is 9.79915 m/s/s.
	// “Standard” gravity is 9.80665 m/s/s.
	static constexpr float ANELLO_CONSTANTS_ONE_G = 9.79915f;

	_px4_accel.set_scale(ANELLO_CONSTANTS_ONE_G);
	_px4_accel.set_range(16.f * ANELLO_CONSTANTS_ONE_G);
	_px4_accel_og.set_scale(ANELLO_CONSTANTS_ONE_G);
	_px4_accel_og.set_range(16.f * ANELLO_CONSTANTS_ONE_G);

	float range_dps = 2000.f;
	_px4_gyro.set_scale(0.0174533);
	_px4_gyro.set_range(math::radians(range_dps));
	_px4_gyro_og.set_scale(0.0174533);
	_px4_gyro_og.set_range(math::radians(range_dps));

	_px4_mag.set_scale(1.f);
}


// Destructor
AnelloX3::~AnelloX3()
{
	// make sure we are truly inactive
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}


// Name: ConfigureUART
// Description: Configures the UART interface to the Anello X3.
int AnelloX3::ConfigureUART()
{
	// Status of UART configuration.
	int ret = 0;

	// Configure UART.
	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("Error opening fd %d", _fd);
		return -1;
	}

	// Baudrate 460800, 8 bits, no parity, 1 stop bit.
	unsigned speed = B460800;
	termios uart_config{};
	int termios_state{};

	tcgetattr(_fd, &uart_config);

	// Clear ONLCR flag (which appends a CR for every LF).
	uart_config.c_oflag &= ~ONLCR;

	uart_config.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS);

	uart_config.c_cflag |= (CS8 | CREAD | CLOCAL);

	// Setup for non-canonical mode.
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	uart_config.c_oflag &= ~OPOST;

	// Fetch bytes as they become available.
	uart_config.c_cc[VMIN] = 1;
	uart_config.c_cc[VTIME] = 1;

	// Set Baudrate.
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
		ret = -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d OSPD\n", termios_state);
		ret = -1;
	}

	if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
		ret = -1;
	}

	if (_fd < 0) {
		PX4_ERR("FAIL: laser fd");
		ret = -1;
	}

	::close(_fd);
	_fd = -1;

	return ret;
}


int AnelloX3::ConfigureX3()
{
	if (_fd < 0)
	{
		// open fd
		PX4_INFO("Opening port %s", _port);
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
		tcflush(_fd, TCIFLUSH);
	}

	// Get updated parameters.
	uint32_t updated_odr = _x3_odr.get();
	uint32_t updated_lpa = _x3_lpa.get();
	uint32_t updated_lpw = _x3_lpw.get();
	uint32_t updated_lpo = _x3_lpo.get();

	// Construct X3 request to update parameters.
	char buf[256];

	// Set param odr.
	snprintf(buf, sizeof(buf), "APCFG,W,odr,%ld", updated_odr);
	generate_x3_request(buf, _x3_request, strlen(buf));
	write_x3();
	px4_usleep(100_ms);

	// Set param lpa.
	snprintf(buf, sizeof(buf), "APCFG,W,lpa,%ld", updated_lpa);
	generate_x3_request(buf, _x3_request, strlen(buf));
	write_x3();
	px4_usleep(100_ms);

	// Set param lpw.
	snprintf(buf, sizeof(buf), "APCFG,W,lpw,%ld", updated_lpw);
	generate_x3_request(buf, _x3_request, strlen(buf));
	write_x3();
	px4_usleep(100_ms);

	// Set param lpo.
	snprintf(buf, sizeof(buf), "APCFG,W,lpo,%ld", updated_lpo);
	generate_x3_request(buf, _x3_request, strlen(buf));
	write_x3();
	px4_usleep(100_ms);

	::close(_fd);
	_fd = -1;

	return PX4_OK;
}


// Name: init()
// Description: Initializes the Anello X3 driver.
int AnelloX3::init()
{
	int ret = PX4_OK;

	ret = ConfigureUART();
	if(ret != PX4_OK)
	{
		PX4_ERR("Failed to configure UART");
		return ret;
	}

	ret = ConfigureX3();
	if(ret != PX4_OK)
	{
		PX4_ERR("Failed to configure Anello X3");
		return ret;
	}

	// Start the driver.
	start();

	return ret;
}


// Name: ParseBinary
// Description: Reads out data from UART buffer, detects binary messages, and parses them.
void AnelloX3::ParseBinary()
{
	// Poll with 5ms timeout.
	pollfd fds[1] {};
	fds[0].fd = _fd;
	fds[0].events = POLLIN;
	int poll_ret = ::poll(fds, 1, 5);

	// Timeout: this means none of our providers is giving us data.
	if (poll_ret == 0)
	{
		perf_count(_poll_timeout_perf);
		ScheduleNow();
		return;

	}
	// Poll failed.
	else if (poll_ret < 0)
	{
		PX4_ERR("poll error: %d", poll_ret);
		perf_count(_poll_error_perf);
		tcflush(_fd, TCIFLUSH);
		ScheduleDelayed(5_ms);
		return;

	}
	else
	{
		// Data is available to read.
		if (fds[0].revents & POLLIN)
		{
			// Clear line buffer if there's not enough room.
			if (_linebuf_index >= sizeof(_linebuf) - 256)
			{
				memset(_linebuf, 0, sizeof(_linebuf));
				_linebuf_index = 0;
			}

			perf_begin(_sample_perf);

			char *readbuf = &_linebuf[_linebuf_index];
			unsigned readlen = 256;
			memset(readbuf, 0, readlen);

			// Read from the sensor (uart buffer)
			int read_ret = ::read(_fd, readbuf, readlen);

			if (read_ret < 0)
			{
				PX4_ERR("read err: %d errno: %d (%s)", read_ret, errno, strerror(errno));

				// Clear line buffer.
				memset(_linebuf, 0, sizeof(_linebuf));
				_linebuf_index = 0;

				perf_count(_read_error_perf);
				perf_count(_comms_errors);
				perf_end(_sample_perf);

				tcflush(_fd, TCIFLUSH);
				ScheduleDelayed(1_ms);
				return;
			} else if (read_ret > 0)
			{
				_linebuf_index += read_ret;
				_bytes_read += read_ret;

				imu_msg_t msg;
				bool start_of_msg_flag = false;

				if(_linebuf_index >= sizeof(msg))
				{
					// Find the preamble 0xC5 0x50.
					for(unsigned int i = 0; i < _linebuf_index; i++)
					{
						if ((_linebuf[i] == 0xC5 && _linebuf[i + 1] == 0x50) && (_linebuf_index - i) >= sizeof(msg.fields))
						{
							memcpy(&msg.fields, &_linebuf[i], sizeof(msg.fields));
							start_of_msg_flag = true;
							break;
						}
						else if((_linebuf_index - i) < sizeof(msg.fields))
						{
							// Not enough bytes to read a complete message. Read more data.
							// PX4_ERR("Not enough bytes left to read a complete message. Getting more data.");
							ScheduleDelayed(1_ms);
							break;
						}
					}

					if(start_of_msg_flag)
					{
						//PX4_INFO("Found start of message.");
						uint64_t timestamp_sample = hrt_absolute_time();

						// Find the message type.
						if(msg.fields.msg_type == (uint8_t) APIMU)
						{
							uint16_t checksum_calculated = 0;
							uint8_t checksum_a = 0;
							uint8_t checksum_b = 0;

							for (uint8_t c = 2; c < 4 + msg.fields.payload_length; c++) {
								checksum_a = checksum_a + msg.bytes[c];
								checksum_b = checksum_b + checksum_a;
							}
							checksum_calculated = (((checksum_a & 0xFF) << 8) | (checksum_b & 0xFF));
							if (checksum_calculated == ((msg.fields.checksum[0] << 8) | msg.fields.checksum[1]))
							{
								perf_count(_checksum_good_perf);

							}
							else {
								perf_count(_checksum_bad_perf);
								// PX4_INFO("Checksum mismatch: calculated %04X, received %02X%02X",
								//	checksum_calculated, msg.fields.checksum[0], msg.fields.checksum[1]);
								PX4_INFO("readbuf: %s", readbuf);

								perf_end(_sample_perf);

								ScheduleNow();
								return;
							}

							// Handle the payload and publish the data to the appropriate topics.
							// Publish temperature.
							float temperature = (float) (msg.fields.temperature / 100.f);
							if (PX4_ISFINITE(temperature))
							{
								_px4_accel.set_temperature(temperature);
								_px4_accel_og.set_temperature(temperature);
								_px4_gyro.set_temperature(temperature);
								_px4_gyro_og.set_temperature(temperature);
								_px4_mag.set_temperature(temperature);
							}

							// Publish accel.
							uint16_t accel_range = (msg.fields.mems_range >> 11) & 0x1F;
							float ax1 = (float) (msg.fields.ax1 * accel_range * (0.0305 / 1000));
							float ay1 = (float) (msg.fields.ay1 * accel_range * (0.0305 / 1000));
							float az1 = (float) (msg.fields.az1 * accel_range * (0.0305 / 1000));
							if (PX4_ISFINITE(ax1) && PX4_ISFINITE(ay1) && PX4_ISFINITE(az1))
							{
								perf_count(_publish_accel_perf);
								_px4_accel.update(timestamp_sample, ax1, ay1, az1);
								_px4_accel_og.update(timestamp_sample, ax1, ay1, az1);
							}

							// Publish gyro.
							uint16_t mems_rate_range = (msg.fields.mems_range) & 0x07FF;
							float wx1 = (float) (msg.fields.wx1 * mems_rate_range * (0.035 / 1000));
							float wy1 = (float) (msg.fields.wy1 * mems_rate_range * (0.035 / 1000));
							float wz1 = (float) (msg.fields.wz1 * mems_rate_range * (0.035 / 1000));
							if (PX4_ISFINITE(wx1) && PX4_ISFINITE(wy1) && PX4_ISFINITE(wz1)) {
								perf_count(_publish_gyro_perf);
								_px4_gyro.update(timestamp_sample, wx1, wy1, wz1);
							}


							float blended_rate_range = INT32_MAX / mems_rate_range;
							float og_wx = (float) (msg.fields.og_wx / blended_rate_range);
							float og_wy = (float) (msg.fields.og_wy / blended_rate_range);
							float og_wz = (float) (msg.fields.og_wz / blended_rate_range);
							if (PX4_ISFINITE(og_wx) && PX4_ISFINITE(og_wy) && PX4_ISFINITE(og_wz))
							{
								_px4_gyro_og.update(timestamp_sample, og_wx, og_wy, og_wz);
							}

							// Publish magnetometer.
							float mag_x = (float) (msg.fields.mag_x / 4096.f);
							float mag_y = (float) (msg.fields.mag_y / 4096.f);
							float mag_z = (float) (msg.fields.mag_z / 4096.f);
							if (PX4_ISFINITE(mag_x) && PX4_ISFINITE(mag_y) && PX4_ISFINITE(mag_z))
							{
								if ((fabsf(mag_x - _mag_prev[0]) > FLT_EPSILON)
									|| (fabsf(mag_y - _mag_prev[1]) > FLT_EPSILON)
									|| (fabsf(mag_z - _mag_prev[2]) > FLT_EPSILON)
									) {
									_px4_mag.update(timestamp_sample, mag_x, mag_y, mag_z);

									_mag_prev[0] = mag_x;
									_mag_prev[1] = mag_y;
									_mag_prev[2] = mag_z;
								}
							}

							// Clear line buffer after parsing a complete message.
							memset(_linebuf, 0, sizeof(_linebuf));
							_linebuf_index = 0;
							perf_end(_sample_perf);
							ScheduleDelayed(4_ms);
						}
						else
						{
							PX4_ERR("Unknown message type: %02X", msg.fields.msg_type);
							memset(_linebuf, 0, sizeof(_linebuf));
							_linebuf_index = 0;
							ScheduleDelayed(1_ms);
							return;
						}
					}
					else
					{
						ScheduleNow();
						return;
					}
				}
				else
				{
					ScheduleNow();
					return;
				}

			}
		}
		// Some other event has occured.
		else
		{
			perf_count(_comms_errors);
			ScheduleNow();
			return;
		}
	}
}


// Name: ParseAscii
// Description: Reads out data from UART buffer, detects ASCII messages, and parses them.
void AnelloX3::ParseAscii()
{
	// poll with 8ms timeout
	pollfd fds[1] {};
	fds[0].fd = _fd;
	fds[0].events = POLLIN;
	int poll_ret = ::poll(fds, 1, 5);

	if (poll_ret == 0) {
		// timeout: this means none of our providers is giving us data
		perf_count(_poll_timeout_perf);
		ScheduleNow();
		return;

	} else if (poll_ret < 0) {
		// poll failed
		PX4_ERR("poll error: %d", poll_ret);
		perf_count(_poll_error_perf);
		tcflush(_fd, TCIFLUSH);
		ScheduleDelayed(5_ms);
		return;

	} else {
		// read data
		if (fds[0].revents & POLLIN) {
			// POLLIN
		} else {
			// no pollin
		}
	}


	// clear buffer and reset if there's not enough room
	if (_linebuf_index >= sizeof(_linebuf) - 256) {
		// clear line buffer
		memset(_linebuf, 0, sizeof(_linebuf));
		_linebuf_index = 0;
	}


	perf_begin(_sample_perf);

	// clear buffer if last read was too long ago
	int64_t read_elapsed = hrt_elapsed_time(&_last_read);

	// the buffer for read chars is buflen minus null termination
	char *readbuf = &_linebuf[_linebuf_index];
	unsigned readlen = 256;

	// read from the sensor (uart buffer)
	int ret = ::read(_fd, readbuf, readlen);

	if (ret < 0) {
		PX4_ERR("read err: %d errno: %d (%s)", ret, errno, strerror(errno));

		// clear line buffer
		memset(_linebuf, 0, sizeof(_linebuf));
		_linebuf_index = 0;

		perf_count(_read_error_perf);
		perf_count(_comms_errors);
		perf_end(_sample_perf);

		tcflush(_fd, TCIFLUSH);
		ScheduleDelayed(1_ms);
		return;// ret;

	} else if (ret > 0) {
		_linebuf_index += ret;
		_bytes_read += ret;

	} else {
		// no bytes to read
		ScheduleDelayed(1_ms);
		return;// ret;
	}

	_last_read = hrt_absolute_time();

	char *ret_strstr_start = (char *)memchr(&_linebuf[0], '#', _linebuf_index);
 	char *ret_strstr_end = (char *)memchr(&_linebuf[0], '*', _linebuf_index);

	bool accel_published = false;
	bool gyro_published = false;
	bool optical_gyro_published = false;

	// printf("_linebuf_index = %d, ", _linebuf_index);
	// for(unsigned int i = 0; i < _linebuf_index; i++)
	// {
	// 	printf("%c", _linebuf[i]);
	// }
	// printf("\n");

	// Require at least 2 characters after * for checksum.
	// if (ret_strstr_start && (strlen(ret_strstr_start) >= 100) && ret_strstr_end && (strlen(ret_strstr_end) >= 3))
	if (ret_strstr_start && ret_strstr_end)
	{
		// Validate that the start of message has a supported the message type.
		uint8_t msg_type_value = is_message_type_valid(ret_strstr_start + 1);

		if(msg_type_value != 0)
		{
			// the checksum is a simple xor of all elements between the start character (#) and the checksum indicator (*)
			char checksum_str[10] {};
			calculate_checksum(ret_strstr_start + 1, ret_strstr_end, checksum_str);

			// Validate checksum of received message.
			const bool checksum_match = (strncmp(checksum_str, ret_strstr_end + 1, 2) == 0);

			if (checksum_match) {
				perf_count(_checksum_good_perf);

			} else {
				perf_count(_checksum_bad_perf);

				perf_end(_sample_perf);

				ScheduleNow();
				return;
			}


			bool apimu_found = false;
			int field_index = 0;

			uint64_t timestamp_sample = 0;
			float accel[3] {NAN, NAN, NAN};
			float gyro[3] {NAN, NAN, NAN};
			float optical_gyro[3] {NAN, NAN, NAN};
			float mag[3] {NAN, NAN, NAN};
			float temperature_C = NAN;

			if(msg_type_value == APIMU)
			{
				timestamp_sample = hrt_absolute_time();
				apimu_found = true;
				field_index = 0;

				const char delimiter[2] = ",";
				// get the first token
				char *token = strtok(ret_strstr_start + 1, delimiter);

				/* walk through other tokens */
				while (token != NULL) {

					int cmp_result = strcmp("#APIMU", token);
					//fprintf(stderr, "%d |%s|\n", field_index, token);

					if (cmp_result == 0) {


					} else {
						field_index++;

						switch (field_index) {
						case 1: { // Time ms MCU time since power on
								float time_ms = atof(token);
								//printf("%d |Time = %.3f ms|\n", field_index, time_ms);
							}
							break;

						case 2: { // T_Sync
								float T_Sync = atof(token);
								//printf("%d |T_Sync = %.3f ms|\n", field_index, T_Sync);
							}
							break;

						case 3: { // AX
								float AX = atof(token);
								//printf("%d |AX = %.3f g|\n", field_index, AX);
								accel[0] = AX;
							}
							break;

						case 4: { // AY
								float AY = atof(token);
								//printf("%d |AY = %.3f g|\n", field_index, AY);
								accel[1] = AY;
							}
							break;

						case 5: { // AZ
								float AZ = atof(token);
								//printf("%d |AZ = %.3f g|\n", field_index, AZ);
								accel[2] = AZ;
							}
							break;

						case 6: { // WX
								float WX = atof(token);
								//printf("%d |WX = %.3f Deg/s|\n", field_index, WX);
								gyro[0] = WX;
							}
							break;

						case 7: { // WY
								float WY = atof(token);
								//printf("%d |WY = %.3f Deg/s|\n", field_index, WY);
								gyro[1] = WY;
							}
							break;

						case 8: { // WZ
								float WZ = atof(token);
								//printf("%d |WZ = %.3f Deg/s|\n", field_index, WZ);
								gyro[2] = WZ;
							}
							break;

						case 9: { // OG_WX
								float OG_WX = atof(token);
								optical_gyro[0] = OG_WX;
							}
							break;

						case 10: { // OG_WY
								float OG_WY = atof(token);
								optical_gyro[1] = OG_WY;
							}
							break;

						case 11: { // OG_WZ
								float OG_WZ = atof(token);
								optical_gyro[2] = OG_WZ;
							}
							break;

						case 12: { // MAG_X
								float MAG_X = atof(token);
								mag[0] = -MAG_X;
							}
							break;

						case 13: { // MAG_Y
								float MAG_Y = atof(token);
								mag[1] = -MAG_Y;
							}
							break;

						case 14: { // MAG_Z
								float MAG_Z = atof(token);
								mag[2] = MAG_Z;
							}
							break;

						case 15: { // Temp C
								float temperature = atof(token);
								//printf("%d |temperature = %.3f C|\n", field_index, temperature);

								temperature_C = temperature;
							}
							break;

						default:
							// too high for APIMU?
							timestamp_sample = 0;
							apimu_found = false;
							break;
						}

						if (apimu_found && (field_index == 15)) {
							// all fields found, publish
							if (PX4_ISFINITE(temperature_C)) {
								_px4_accel.set_temperature(temperature_C);
								_px4_accel_og.set_temperature(temperature_C);
								_px4_gyro.set_temperature(temperature_C);
								_px4_gyro_og.set_temperature(temperature_C);
								_px4_mag.set_temperature(temperature_C);
							}

							if (timestamp_sample != 0) {
								if (PX4_ISFINITE(accel[0]) && PX4_ISFINITE(accel[1]) && PX4_ISFINITE(accel[2])) {
									if (!accel_published) {
										perf_count(_publish_accel_perf);
										_px4_accel.update(timestamp_sample, accel[0], accel[1], accel[2]);
										_px4_accel_og.update(timestamp_sample, accel[0], accel[1], accel[2]);
										accel[0] = NAN;
										accel[1] = NAN;
										accel[2] = NAN;

										accel_published = true;
									}
								}

								if (PX4_ISFINITE(gyro[0]) && PX4_ISFINITE(gyro[1]) && PX4_ISFINITE(gyro[2])) {
									if (!gyro_published) {
										perf_count(_publish_gyro_perf);
										_px4_gyro.update(timestamp_sample, gyro[0], gyro[1], gyro[2]);
										gyro[0] = NAN;
										gyro[1] = NAN;
										gyro[2] = NAN;

										gyro_published = true;
									}
								}

								if (PX4_ISFINITE(optical_gyro[0]) && PX4_ISFINITE(optical_gyro[1]) && PX4_ISFINITE(optical_gyro[2])) {
									if (!optical_gyro_published) {
										//perf_count(_publish_gyro_og_perf);
										_px4_gyro_og.update(timestamp_sample, optical_gyro[0], optical_gyro[1], optical_gyro[2]);
										optical_gyro[0] = NAN;
										optical_gyro[1] = NAN;
										optical_gyro[2] = NAN;

										optical_gyro_published = true;
									}
								}

								if (PX4_ISFINITE(mag[0]) && PX4_ISFINITE(mag[1]) && PX4_ISFINITE(mag[2])) {

									if ((fabsf(mag[0] - _mag_prev[0]) > FLT_EPSILON)
									|| (fabsf(mag[1] - _mag_prev[1]) > FLT_EPSILON)
									|| (fabsf(mag[2] - _mag_prev[2]) > FLT_EPSILON)
									) {
										_px4_mag.update(timestamp_sample, mag[0], mag[1], mag[2]);

										_mag_prev[0] = mag[0];
										_mag_prev[1] = mag[1];
										_mag_prev[2] = mag[2];
									}
								}

								timestamp_sample = 0;

								if (accel_published && gyro_published && optical_gyro_published) {
									// clear line buffer
									memset(_linebuf, 0, sizeof(_linebuf));
									_linebuf_index = 0;

									ScheduleDelayed(1_ms);
									perf_end(_sample_perf);
									return;
								}


							}
						}
					}

					token = strtok(NULL, delimiter);
				}
			}
			else if(msg_type_value == APCFG)
			{
				strncpy(_x3_response, ret_strstr_start, ret_strstr_end - ret_strstr_start + 2);
				_x3_response[ret_strstr_end - ret_strstr_start + 3] = '\0';
				_resp_received_flag = 1;

				// clear line buffer
				memset(_linebuf, 0, sizeof(_linebuf));
				_linebuf_index = 0;

				ScheduleDelayed(1_ms);
				perf_end(_sample_perf);
				return;
			}
		}
	}


	if (_print_debug.load())
	{
		_print_debug.store(false);
	}

	perf_end(_sample_perf);

	if (_linebuf_index > 0)
	{
		if (ret_strstr_start)
		{
			ScheduleNow();
			return;
		}
	}

	ScheduleDelayed(3_ms);
}


// Name: Run
// Description: Run loop for the Anello X3 driver.
void AnelloX3::Run()
{
	if (_fd < 0)
	{
		// open fd
		PX4_INFO("Opening port %s", _port);
		_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
		tcflush(_fd, TCIFLUSH);
	}


	if(get_config_mode())
	{
		// Handle requests to X3.
		if(_send_req_flag)
		{
			// clear line buffer
			memset(_linebuf, 0, sizeof(_linebuf));
			_linebuf_index = 0;

			// Clear UART buffer
			tcflush(_fd, TCIFLUSH);


			write_x3();

			_send_req_flag = 0;
			_resp_received_flag = 0;
		}

		ParseAscii();
	}
	else
	{
		ParseBinary();
	}

}


// Name: start
// Description: Sets up the first call of Run in PX4 scheduler.
void AnelloX3::start()
{
	// schedule a cycle to start things (the sensor sends at 100Hz, but we run a bit faster to avoid missing data)
	ScheduleOnInterval(1_ms);
}


// Name: stop
// Description: Stops the Anello X3 driver.
void AnelloX3::stop()
{
	ScheduleClear();
}


// Name: print_info
// Description: Prints status information about the Anello X3 driver.
void AnelloX3::print_info()
{
	PX4_INFO_RAW("Using port '%s'\n", _port);
	PX4_INFO_RAW("bytes read %d\n", _bytes_read);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_poll_timeout_perf);
	perf_print_counter(_poll_error_perf);
	perf_print_counter(_read_error_perf);
	perf_print_counter(_checksum_good_perf);
	perf_print_counter(_checksum_bad_perf);
	perf_print_counter(_publish_accel_perf);
	perf_print_counter(_publish_gyro_perf);

	_print_debug.store(true);
}


// Name: start_config_mode
// Description: Starts configuration mode.
void AnelloX3::start_config_mode()
{
	_config_mode = 1;

	ScheduleNow();
}


// Name: stop_config_mode
// Description: Stops the configuration mode.
void AnelloX3::stop_config_mode()
{
	// Clear UART buffer.
	tcflush(_fd, TCIFLUSH);

	// Clear line buffer.
	memset(_linebuf, 0, sizeof(_linebuf));
	_linebuf_index = 0;

	_config_mode = 0;

	ScheduleNow();
}


// Name: get_config_mode
// Description: Gets the current configuration mode.
int AnelloX3::get_config_mode()
{
	return _config_mode;
}


// Name: write_x3
// Description: Writes user input requests to the X3 device.
// Returns: Number of bytes written, or -1 on error.
ssize_t AnelloX3::write_x3()
{
	// Write to X3 (uart buffer).
	ssize_t num_bytes = ::write(_fd, _x3_request, strlen(_x3_request));

	if (num_bytes < 0) {
		PX4_ERR("Failed to send message, num_bytes = %d, errorno = %s", num_bytes, strerror(errno));
		return PX4_ERROR;
	}

	return num_bytes;
}


// Name: generate_x3_request
// Description: Generates an X3 request string from a given message.
// Parameters:
// - msg: Pointer to the message string to include in the request.
// - len: Length of the message string.
void AnelloX3::generate_x3_request(const char* msg, char* full_msg, size_t len)
{
	char buf[len];
	memcpy(buf, msg, len);

	char *buf_start = buf;
	char *buf_end = buf + len;
	char checksum[3] = {0};
	calculate_checksum(buf_start, buf_end, checksum);

	// Create an X3 request with '#' preamble, the message, and the checksum.
	full_msg[0] = '#';
	memcpy(&full_msg[1], msg, len);
	full_msg[1 + len] = '*';
	memcpy(&full_msg[1 + len + 1], checksum, 2);
	full_msg[1 + len + 3] = '\r';
	full_msg[1 + len + 4] = '\n';
	full_msg[1 + len + 5] = '\0';
}


// Name: send_x3_request
// Description: Sets flag to send request stored in _x3_request to X3.
// Parameters:
// - msg: Pointer to the request string to send.
// - len: Length of the request string.
void AnelloX3::send_x3_request(const char* msg, size_t len)
{
	generate_x3_request(msg, _x3_request, len);

	PX4_INFO("Sending message: %s", _x3_request);

	_send_req_flag = 1;

	ScheduleNow();
}


// Name: calculate_checksum
// Description: Calculates the checksum for a given buffer (assumes ASCII format).
// Parameters:
// - buf_start: Pointer to the start of the buffer containing the message.
// - buf_end: Pointer to the end of the buffer containing the message.
// - checksum_str: Pointer to a character array where the checksum will be stored.
void AnelloX3::calculate_checksum(char *buf_start, char *buf_end, char *checksum_str)
{
	if (buf_start == nullptr || buf_end == nullptr) {
		PX4_ERR("Invalid buffer or length");
		return;
	}

	// The checksum is a simple xor of all elements of message before adding in the preamble and checksum.
	uint8_t checksum_calculated = 0;

	for (char *c = buf_start; c < buf_end; c++) {
		checksum_calculated = checksum_calculated ^ uint8_t(*c);
	}

	sprintf(checksum_str, "%X\n", checksum_calculated);
}


// Singular struct to pair a msg_type with a unique value.
struct MessageTypeMapEntry
{
	const char *msg_type;
	uint8_t value;
};


// Map data structure to map supported message types to their unique values.
static MessageTypeMapEntry message_types[] = {
	// Public output message types.
	{"APIMU",    APIMU},
	// User input message types.
	{"APCFG",    APCFG},
	{"APRST",    APRST},
	{"APPNG",    APPNG}
};


// Name: is_message_type_valid
// Description: Checks if the provided message type is valid.
// Parameters:
// - message_type: Pointer to a character array containing the message type to validate.
// Returns: 0 if the message type is invalid, the corresponding message type value otherwise.
uint8_t AnelloX3::is_message_type_valid(const char *message_type)
{
	if (message_type == nullptr) {
		PX4_ERR("Invalid message type");
		return 0;
	}

	for(unsigned int i = 0; i < sizeof(message_types) / sizeof(MessageTypeMapEntry); i++)
	{
		if (strncmp(message_type, message_types[i].msg_type, strlen(message_types[i].msg_type)) == 0) {
			return message_types[i].value;
		}
	}

	// PX4_ERR("Unknown message type: %s", message_type);
	return 0;
}


// Name: get_x3_response
// Description: Get the response from X3 to a previously sent request.
// Parameters:
// - response: Pointer to a character array where the response will be stored.
int AnelloX3::get_x3_response(char *response)
{
	if(_resp_received_flag)
	{
		PX4_INFO("_x3_response = %s", _x3_response);
		strncpy(response, _x3_response, strlen(_x3_response));
		// _x3_response[0] = '\0'; // Clear the response after reading it.
	}
	return _resp_received_flag;
}
