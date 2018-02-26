/****************************************************************************
 *
 *   Copyright (c) 2016-2018 PX4 Development Team. All rights reserved.
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

#include "IridiumSBD.h"

#include <px4_tasks.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <pthread.h>

#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include "drivers/drv_iridiumsbd.h"

static constexpr const char *satcom_state_string[4] = {"STANDBY", "SIGNAL CHECK", "SBD SESSION", "TEST"};

#define VERBOSE_INFO(...) if (verbose) { PX4_INFO(__VA_ARGS__); }

IridiumSBD *IridiumSBD::instance;
int IridiumSBD::task_handle;

IridiumSBD::IridiumSBD()
	: CDev("iridiumsbd", IRIDIUMSBD_DEVICE_PATH)
{
}

///////////////////////////////////////////////////////////////////////
// public functions                                                  //
///////////////////////////////////////////////////////////////////////

int IridiumSBD::start(int argc, char *argv[])
{
	PX4_INFO("starting");

	if (IridiumSBD::instance != nullptr) {
		PX4_WARN("already started");
		return PX4_ERROR;
	}

	IridiumSBD::instance = new IridiumSBD();

	IridiumSBD::task_handle = px4_task_spawn_cmd("iridiumsbd", SCHED_DEFAULT,
				  SCHED_PRIORITY_SLOW_DRIVER, 1200, (main_t)&IridiumSBD::main_loop_helper, argv);

	return OK;
}

int IridiumSBD::stop()
{
	if (IridiumSBD::instance == nullptr) {
		PX4_WARN("not started");
		return PX4_ERROR;
	}

	PX4_WARN("stopping...");

	IridiumSBD::instance->task_should_exit = true;

	// give it enough time to stop
	//param_timeout_s = 10;

	// TODO
	for (int i = 0; (i < 10 + 1) && (IridiumSBD::task_handle != -1); i++) {
		sleep(1);
	}

	// well, kill it anyway, though this may crash
	if (IridiumSBD::task_handle != -1) {
		PX4_WARN("killing task forcefully");

		::close(IridiumSBD::instance->uart_fd);
		task_delete(IridiumSBD::task_handle);
		IridiumSBD::task_handle = -1;
		delete IridiumSBD::instance;
		IridiumSBD::instance = nullptr;
	}

	return OK;
}

void IridiumSBD::status()
{
	if (IridiumSBD::instance == nullptr) {
		PX4_WARN("not started");
		return;
	}

	PX4_INFO("started");
	PX4_INFO("state: %s", satcom_state_string[instance->state]);

	PX4_INFO("TX buf written:               %d", instance->tx_buf_write_idx);
	PX4_INFO("Signal quality:               %d", instance->signal_quality);
	PX4_INFO("Time since last signal check: %lld", hrt_absolute_time() - instance->last_signal_check);
	PX4_INFO("Last heartbeat:               %lld", instance->last_heartbeat);
}

void IridiumSBD::test(int argc, char *argv[])
{
	if (instance == nullptr) {
		PX4_WARN("not started");
		return;
	}

	if (instance->state != SATCOM_STATE_STANDBY || instance->test_pending) {
		PX4_WARN("MODEM BUSY!");
		return;
	}

	if (argc > 2) {
		strncpy(instance->test_command, argv[2], sizeof(instance->test_command));
		instance->test_command[sizeof(instance->test_command) - 1] = 0;

	} else {
		instance->test_command[0] = 0;
	}

	instance->schedule_test();
}

int IridiumSBD::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case FIONREAD: {
			int count = rx_msg_end_idx - rx_msg_read_idx;
			*(int *)arg = count;

			return OK;
		}

	case FIONSPACE: {
			int count = SATCOM_TX_BUF_LEN - tx_buf_write_idx;
			*(int *)arg = count;

			return OK;
		}

	default: {

			/* see if the parent class can make any use of it */
			return CDev::ioctl(filp, cmd, arg);
		}
	}
}

///////////////////////////////////////////////////////////////////////
// private functions                                                 //
///////////////////////////////////////////////////////////////////////

void IridiumSBD::main_loop_helper(int argc, char *argv[])
{
	// start the main loop and stay in it
	IridiumSBD::instance->main_loop(argc, argv);

	// tear down everything after the main loop exits
	::close(IridiumSBD::instance->uart_fd);
	IridiumSBD::task_handle = -1;
	delete IridiumSBD::instance;
	IridiumSBD::instance = nullptr;

	PX4_WARN("stopped");
}

void IridiumSBD::main_loop(int argc, char *argv[])
{
	CDev::init();

	pthread_mutex_init(&tx_buf_mutex, NULL);

	int arg_i = 3;
	int arg_uart_name = 0;

	while (arg_i < argc) {
		if (!strcmp(argv[arg_i], "-d")) {
			arg_i++;
			arg_uart_name = arg_i;

		} else if (!strcmp(argv[arg_i], "-v")) {
			PX4_WARN("verbose mode ON");
			verbose = true;
		}

		arg_i++;
	}

	if (arg_uart_name == 0) {
		PX4_WARN("no Iridium SBD modem UART port provided!");
		task_should_exit = true;
		return;
	}

	if (open_uart(argv[arg_uart_name]) != SATCOM_UART_OK) {
		PX4_WARN("failed to open UART port!");
		task_should_exit = true;
		return;
	}

	// disable flow control
	write_at("AT&K0");

	if (read_at_command() != SATCOM_RESULT_OK) {
		PX4_WARN("modem not responding");
		return;
	}

	// disable command echo
	write_at("ATE0");

	if (read_at_command() != SATCOM_RESULT_OK) {
		PX4_WARN("modem not responding");
		return;
	}

	param_t param_pointer;

	param_pointer = param_find("ISBD_READ_INT");
	param_get(param_pointer, &param_read_interval_s);

//	if (param_read_interval_s < 0) {
//		param_read_interval_s = 10;
//	}

	param_pointer = param_find("ISBD_SBD_TIMEOUT");
	param_get(param_pointer, &param_session_timeout_s);

	if (param_session_timeout_s < 0) {
		param_session_timeout_s = 60;
	}

	param_pointer = param_find("ISBD_STACK_TIME");
	param_get(param_pointer, &param_stacking_time_ms);

	if (param_stacking_time_ms < 0) {
		param_stacking_time_ms = 0;
	}

	VERBOSE_INFO("read interval: %d s", param_read_interval_s);
	VERBOSE_INFO("SBD session timeout: %d s", param_session_timeout_s);
	VERBOSE_INFO("SBD stack time: %d ms", param_stacking_time_ms);

	while (!task_should_exit) {
		switch (state) {
		case SATCOM_STATE_STANDBY:
			standby_loop();
			break;

		case SATCOM_STATE_CSQ:
			csq_loop();
			break;

		case SATCOM_STATE_SBDSESSION:
			sbdsession_loop();
			break;

		case SATCOM_STATE_TEST:
			test_loop();
			break;
		}

		if (new_state != state) {
			VERBOSE_INFO("SWITCHING STATE FROM %s TO %s", satcom_state_string[state], satcom_state_string[new_state]);
			state = new_state;

		} else {
			usleep(100000);	// 100ms
		}
	}
}

void IridiumSBD::standby_loop(void)
{
	if (test_pending) {
		test_pending = false;

		if (!strcmp(test_command, "s")) {
			write(0, "kreczmer", 8);

		} else if (!strcmp(test_command, "read")) {
			rx_session_pending = true;

		} else {
			test_timer = hrt_absolute_time();
			start_test();
			return;
		}
	}

	// check for incoming SBDRING, handled inside read_at_command()
	read_at_command();

	if (param_read_interval_s > 0 && ((int64_t)(hrt_absolute_time() - last_read_time) > param_read_interval_s * 1000000)) {
		rx_session_pending = true;
	}

	// write the MO buffer when the message stacking time expires
	if ((tx_buf_write_idx > 0) && ((int64_t)(hrt_absolute_time() - last_write_time) > param_stacking_time_ms * 1000)) {
		write_tx_buf();
	}

	// do not start an SBD session if there is still data in the MT buffer, or it will be lost
	if ((tx_session_pending || rx_session_pending) && !rx_read_pending) {
		if (signal_quality > 0) {
			// clear the MO buffer if we only want to read a message
			if (rx_session_pending && !tx_session_pending) {
				if (clear_mo_buffer()) {
					start_sbd_session();
				}

			} else {
				start_sbd_session();
			}

		} else {
			start_csq();
		}
	}

	// start a signal check if requested
	if ((hrt_absolute_time() - last_signal_check) > SATCOM_SIGNAL_REFRESH_DELAY) {
		start_csq();
	}

	// only read the MT buffer if the higher layer (mavlink app) read the previous message
	if (rx_read_pending && (rx_msg_read_idx == rx_msg_end_idx)) {
		read_rx_buf();
	}
}

void IridiumSBD::csq_loop(void)
{
	int res = read_at_command();

	if (res == SATCOM_RESULT_NA) {
		return;
	}

	if (res != SATCOM_RESULT_OK) {
		VERBOSE_INFO("UPDATE SIGNAL QUALITY: ERROR");

		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	if (strncmp((const char *)rx_command_buf, "+CSQ:", 5)) {
		VERBOSE_INFO("UPDATE SIGNAL QUALITY: WRONG ANSWER:");
		VERBOSE_INFO("%s", rx_command_buf);

		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	signal_quality = rx_command_buf[5] - 48;

	VERBOSE_INFO("SIGNAL QUALITY: %d", signal_quality);

	new_state = SATCOM_STATE_STANDBY;

	publish_telemetry_status();
}

void IridiumSBD::sbdsession_loop(void)
{
	int res = read_at_command();

	if (res == SATCOM_RESULT_NA) {
		if ((param_session_timeout_s > 0)
		    && ((int64_t)((hrt_absolute_time() - session_start_time))
			> param_session_timeout_s * 1000000)) {

			PX4_WARN("SBD SESSION: TIMEOUT!");
			new_state = SATCOM_STATE_STANDBY;
		}

		return;
	}

	if (res != SATCOM_RESULT_OK) {
		VERBOSE_INFO("SBD SESSION: ERROR. RESULT: %d", res);

		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	if (strncmp((const char *)rx_command_buf, "+SBDIX:", 7)) {

		VERBOSE_INFO("SBD SESSION: WRONG ANSWER: %s", rx_command_buf);

		new_state = SATCOM_STATE_STANDBY;
		return;
	}

	int mo_status, mt_status, mt_len, mt_queued;
	const char *p = (const char *)rx_command_buf + 7;
	char **rx_buf_parse = (char **)&p;

	mo_status = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	strtol(*rx_buf_parse, rx_buf_parse, 10); // MOMSN, ignore it
	(*rx_buf_parse)++;
	mt_status = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	strtol(*rx_buf_parse, rx_buf_parse, 10); // MTMSN, ignore it
	(*rx_buf_parse)++;
	mt_len = strtol(*rx_buf_parse, rx_buf_parse, 10);
	(*rx_buf_parse)++;
	mt_queued = strtol(*rx_buf_parse, rx_buf_parse, 10);

	VERBOSE_INFO("MO ST: %d, MT ST: %d, MT LEN: %d, MT QUEUED: %d", mo_status, mt_status, mt_len, mt_queued);

	switch (mo_status) {
	case 0:
	case 2:
	case 3:
	case 4:
		VERBOSE_INFO("SBD SESSION: SUCCESS (%d)", mo_status);

		ring_pending = false;
		rx_session_pending = false;
		tx_session_pending = false;
		last_read_time = hrt_absolute_time();
		last_heartbeat = last_read_time;

		if (mt_len > 0) {
			rx_read_pending = true;
		}

		publish_telemetry_status();

		break;

	case 1:
		VERBOSE_INFO("SBD SESSION: MO SUCCESS, MT FAIL");
		last_heartbeat = hrt_absolute_time();
		publish_telemetry_status();

		tx_session_pending = false;
		break;

	case 32:
		VERBOSE_INFO("SBD SESSION: NO NETWORK SIGNAL");

		signal_quality = 0;

		break;

	default:
		VERBOSE_INFO("SBD SESSION: FAILED (%d)", mo_status);
	}

	new_state = SATCOM_STATE_STANDBY;
}

void IridiumSBD::test_loop(void)
{
	int res = read_at_command();

	if (res != SATCOM_RESULT_NA) {
		PX4_INFO("TEST RESULT: %d, LENGTH %d\nDATA:\n%s", res, rx_command_len, rx_command_buf);
		PX4_INFO("TEST DONE, TOOK %lld MS", (hrt_absolute_time() - test_timer) / 1000);
		new_state = SATCOM_STATE_STANDBY;
	}

	// timeout after 60 s in the test state
	if ((int64_t)((hrt_absolute_time() - test_timer)) > 60000000) {
		PX4_WARN("TEST TIMEOUT AFTER %lld S", (hrt_absolute_time() - test_timer) / 1000000);
		new_state = SATCOM_STATE_STANDBY;
	}
}

void IridiumSBD::start_csq(void)
{
	VERBOSE_INFO("UPDATING SIGNAL QUALITY");

	last_signal_check = hrt_absolute_time();

	if (!is_modem_ready()) {
		VERBOSE_INFO("UPDATE SIGNAL QUALITY: MODEM NOT READY!");
		return;
	}

	write_at("AT+CSQ");
	new_state = SATCOM_STATE_CSQ;
}

void IridiumSBD::start_sbd_session(void)
{
	VERBOSE_INFO("STARTING SBD SESSION");

	if (!is_modem_ready()) {
		VERBOSE_INFO("SBD SESSION: MODEM NOT READY!");
		return;
	}

	if (ring_pending) {
		write_at("AT+SBDIXA");

	} else {
		write_at("AT+SBDIX");
	}

	new_state = SATCOM_STATE_SBDSESSION;
	session_start_time = hrt_absolute_time();
}

void IridiumSBD::start_test(void)
{
	int res = read_at_command();

	if (res != SATCOM_RESULT_NA) {
		PX4_WARN("SOMETHING WAS IN BUFFER");
		printf("TEST RESULT: %d, LENGTH %d\nDATA:\n%s\nRAW DATA:\n", res, rx_command_len, rx_command_buf);

		for (int i = 0; i < rx_command_len; i++) {
			printf("%d ", rx_command_buf[i]);
		}

		printf("\n");
	}

	if (!is_modem_ready()) {
		PX4_WARN("MODEM NOT READY!");
		return;
	}

	if (strlen(test_command) != 0) {
		if ((strstr(test_command, "AT") != nullptr) || (strstr(test_command, "at") != nullptr)) {
			PX4_INFO("TEST %s", test_command);
			write_at(test_command);
			new_state = SATCOM_STATE_TEST;

		} else {
			PX4_WARN("The test command does not include AT or at: %s, ignoring it.", test_command);
			new_state = SATCOM_STATE_STANDBY;
		}

	} else {
		PX4_INFO("TEST DONE");
	}
}

ssize_t IridiumSBD::write(struct file *filp, const char *buffer, size_t buflen)
{
	VERBOSE_INFO("WRITE: LEN %d, TX WRITTEN: %d", buflen, tx_buf_write_idx);

	if ((ssize_t)buflen > SATCOM_TX_BUF_LEN - tx_buf_write_idx) {
		return PX4_ERROR;
	}

	pthread_mutex_lock(&tx_buf_mutex);

	memcpy(tx_buf + tx_buf_write_idx, buffer, buflen);

	tx_buf_write_idx += buflen;
	last_write_time = hrt_absolute_time();

	pthread_mutex_unlock(&tx_buf_mutex);

	return buflen;
}

ssize_t IridiumSBD::read(struct file *filp, char *buffer, size_t buflen)
{
	VERBOSE_INFO("READ: LEN %d, RX: %d RX END: %d", buflen, rx_msg_read_idx, rx_msg_end_idx);

	if (rx_msg_read_idx < rx_msg_end_idx) {
		size_t bytes_to_copy = rx_msg_end_idx - rx_msg_read_idx;

		if (bytes_to_copy > buflen) {
			bytes_to_copy = buflen;
		}

		memcpy(buffer, &rx_msg_buf[rx_msg_read_idx], bytes_to_copy);

		rx_msg_read_idx += bytes_to_copy;

		return bytes_to_copy;

	} else {
		return -EAGAIN;
	}
}

void IridiumSBD::write_tx_buf()
{
	if (!is_modem_ready()) {
		VERBOSE_INFO("WRITE SBD: MODEM NOT READY!");
		return;
	}

	pthread_mutex_lock(&tx_buf_mutex);

	char command[13];
	sprintf(command, "AT+SBDWB=%d", tx_buf_write_idx);
	write_at(command);

	if (read_at_command() != SATCOM_RESULT_READY) {
		VERBOSE_INFO("WRITE SBD: MODEM NOT RESPONDING!");
		return;
	}

	int sum = 0;

	int written = 0;

	while (written != tx_buf_write_idx) {
		written += ::write(uart_fd, tx_buf + written, tx_buf_write_idx - written);
	}

	for (int i = 0; i < tx_buf_write_idx; i++) {
		sum += tx_buf[i];
	}

	uint8_t checksum[2] = {(uint8_t)(sum / 256), (uint8_t)(sum & 255)};
	::write(uart_fd, checksum, 2);


	VERBOSE_INFO("SEND SBD: CHECKSUM %d %d", checksum[0], checksum[1]);

	if (read_at_command(250) != SATCOM_RESULT_OK) {
		VERBOSE_INFO("WRITE SBD: ERROR WHILE WRITING DATA TO MODEM!");

		pthread_mutex_unlock(&tx_buf_mutex);
		return;
	}

	if (rx_command_buf[0] != '0') {

		VERBOSE_INFO("WRITE SBD: ERROR WHILE WRITING DATA TO MODEM! (%d)", rx_command_buf[0] - '0');

		pthread_mutex_unlock(&tx_buf_mutex);
		return;
	}

	VERBOSE_INFO("WRITE SBD: DATA WRITTEN TO MODEM");

	tx_buf_write_idx = 0;

	pthread_mutex_unlock(&tx_buf_mutex);

	tx_session_pending = true;
}

void IridiumSBD::read_rx_buf(void)
{
	if (!is_modem_ready()) {
		VERBOSE_INFO("READ SBD: MODEM NOT READY!");
		return;
	}

	write_at("AT+SBDRB");

	if (read_at_msg() != SATCOM_RESULT_OK) {
		VERBOSE_INFO("READ SBD: MODEM NOT RESPONDING!");
		return;
	}

	int data_len = (rx_msg_buf[0] << 8) + rx_msg_buf[1];

	// rx_buf contains 2 byte length, data, 2 byte checksum and /r/n delimiter
	if (data_len != rx_msg_end_idx - 6) {
		VERBOSE_INFO("READ SBD: WRONG DATA LENGTH");
		return;
	}

	int checksum = 0;

	for (int i = 2; i < data_len + 2; i++) {
		checksum += rx_msg_buf[i];
	}

	if ((checksum / 256 != rx_msg_buf[rx_msg_end_idx - 4]) || ((checksum & 255) != rx_msg_buf[rx_msg_end_idx - 3])) {
		VERBOSE_INFO("READ SBD: WRONG DATA CHECKSUM");
		return;
	}

	rx_msg_read_idx = 2;	// ignore the length
	rx_msg_end_idx -= 4;	// ignore the checksum and delimiter
	rx_read_pending = false;

	VERBOSE_INFO("READ SBD: SUCCESS, LEN: %d", data_len);
}

void IridiumSBD::write_at(const char *command)
{
	VERBOSE_INFO("WRITING AT COMMAND: %s", command);

	::write(uart_fd, command, strlen(command));
	::write(uart_fd, "\r", 1);
}

satcom_result_code IridiumSBD::read_at_command(int16_t timeout)
{
	return read_at(rx_command_buf, &rx_command_len, timeout);
}

satcom_result_code IridiumSBD::read_at_msg(int16_t timeout)
{
	return read_at(rx_msg_buf, &rx_msg_end_idx, timeout);
}

satcom_result_code IridiumSBD::read_at(uint8_t *rx_buf, int *rx_len, int16_t timeout)
{
	struct pollfd fds[1];
	fds[0].fd = uart_fd;
	fds[0].events = POLLIN;

	uint8_t buf = 0;
	int last_rn_idx = 0;
	int rx_buf_pos = 0;
	*rx_len = 0;


	while (1) {
		if (::poll(&fds[0], 1, timeout) > 0) {
			if (::read(uart_fd, &buf, 1) > 0) {
				if (rx_buf_pos == 0 && (buf == '\r' || buf == '\n')) {
					// ignore the leading \r\n
					continue;
				}

				rx_buf[rx_buf_pos++] = buf;

				if (rx_buf[rx_buf_pos - 1] == '\n' && rx_buf[rx_buf_pos - 2] == '\r') {
					// found the \r\n delimiter
					if (rx_buf_pos == last_rn_idx + 2) {
						//if (verbose) { PX4_INFO("second in a row, ignore it");}
						; // second in a row, ignore it

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "OK\r\n", 4)) {
						rx_buf[*rx_len] = 0; 	// null terminator after the information response for printing purposes
						return SATCOM_RESULT_OK;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "ERROR\r\n", 7)) {
						return SATCOM_RESULT_ERROR;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "SBDRING\r\n", 9)) {
						ring_pending = true;
						rx_session_pending = true;

						VERBOSE_INFO("GET SBDRING");

						return SATCOM_RESULT_SBDRING;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "READY\r\n", 7)) {
						return SATCOM_RESULT_READY;

					} else if (!strncmp((const char *)&rx_buf[last_rn_idx], "HARDWARE FAILURE", 16)) {
						PX4_WARN("HARDWARE FAILURE!");
						return SATCOM_RESULT_HWFAIL;

					} else {
						*rx_len = rx_buf_pos;	// that was the information response, result code incoming
					}

					last_rn_idx = rx_buf_pos;
				}
			}

		} else {
			break;
		}
	}

	return SATCOM_RESULT_NA;
}

void IridiumSBD::schedule_test(void)
{
	test_pending = true;
}

bool IridiumSBD::clear_mo_buffer()
{
	write_at("AT+SBDD0");

	if (read_at_command() != SATCOM_RESULT_OK || rx_command_buf[0] != '0') {
		VERBOSE_INFO("CLEAR MO BUFFER: ERROR");
		return false;
	}

	return true;
}

satcom_uart_status IridiumSBD::open_uart(char *uart_name)
{
	VERBOSE_INFO("opening Iridium SBD modem UART: %s", uart_name);

	uart_fd = ::open(uart_name, O_RDWR | O_BINARY);

	if (uart_fd < 0) {
		VERBOSE_INFO("UART open failed!");
		return SATCOM_UART_OPEN_FAIL;
	}

	// set the UART speed to 115200
	struct termios uart_config;
	tcgetattr(uart_fd, &uart_config);
	cfsetspeed(&uart_config, 115200);
	tcsetattr(uart_fd, TCSANOW, &uart_config);

	VERBOSE_INFO("UART opened");

	return SATCOM_UART_OK;
}

bool IridiumSBD::is_modem_ready(void)
{
	write_at("AT");

	if (read_at_command() == SATCOM_RESULT_OK) {
		return true;

	} else {
		return false;
	}
}

pollevent_t IridiumSBD::poll_state(struct file *filp)
{
	pollevent_t pollstate = 0;

	if (rx_msg_read_idx < rx_msg_end_idx) {
		pollstate |= POLLIN;
	}

	if (SATCOM_TX_BUF_LEN - tx_buf_write_idx > 0) {
		pollstate |= POLLOUT;
	}

	return pollstate;
}

void IridiumSBD::publish_telemetry_status()
{
	// publish telemetry status for logger
	struct telemetry_status_s tstatus = {};

	tstatus.timestamp = hrt_absolute_time();
	tstatus.telem_time = tstatus.timestamp;
	tstatus.type = telemetry_status_s::TELEMETRY_STATUS_RADIO_TYPE_IRIDIUM;
	tstatus.rssi = signal_quality;
	tstatus.txbuf = tx_buf_write_idx;
	tstatus.heartbeat_time = last_heartbeat;

	if (telemetry_status_pub == nullptr) {
		int multi_instance;
		telemetry_status_pub = orb_advertise_multi(ORB_ID(telemetry_status), &tstatus, &multi_instance, ORB_PRIO_LOW);

	} else {
		orb_publish(ORB_ID(telemetry_status), telemetry_status_pub, &tstatus);
	}
}

int iridiumsbd_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {
		return IridiumSBD::start(argc, argv);

	} else if (!strcmp(argv[1], "stop")) {
		return IridiumSBD::stop();

	} else if (!strcmp(argv[1], "status")) {
		IridiumSBD::status();
		return OK;

	} else if (!strcmp(argv[1], "test")) {
		IridiumSBD::test(argc, argv);
		return OK;
	}

	PX4_INFO("usage: iridiumsbd {start|stop|status|test} [-d uart_device]");

	return PX4_ERROR;
}
