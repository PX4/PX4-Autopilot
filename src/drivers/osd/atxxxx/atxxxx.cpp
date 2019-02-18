/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file atxxxx.cpp
 * @author Daniele Pettenuzzo
 *
 * Driver for the ATXXXX chip on the omnibus fcu connected via SPI.
 */

#include "atxxxx.h"


OSDatxxxx::OSDatxxxx(int bus) :
	SPI("OSD", OSD_DEVICE_PATH, bus, OSD_SPIDEV, SPIDEV_MODE0, OSD_SPI_BUS_SPEED),
	_measure_ticks(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "osd_read")),
	_comms_errors(perf_alloc(PC_COUNT, "osd_com_err")),
	_battery_sub(-1),
	_local_position_sub(-1),
	_vehicle_status_sub(-1),
	_battery_voltage_filtered_v(0),
	_battery_discharge_mah(0),
	_battery_valid(false),
	_local_position_z(0),
	_local_position_valid(false),
	_arming_state(1),
	_arming_timestamp(0)
{
	_p_tx_mode = param_find("OSD_ATXXXX_CFG");
	param_get(_p_tx_mode, &_tx_mode);

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

OSDatxxxx::~OSDatxxxx()
{
	/* make sure we are truly inactive */
	stop();

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}


int
OSDatxxxx::init()
{
	/* do SPI init (and probe) first */
	if (SPI::init() != PX4_OK) {
		goto fail;
	}

	if (reset() != PX4_OK) {
		goto fail;
	}

	if (init_osd() != PX4_OK) {
		goto fail;
	}

	for (int i = 0; i < OSD_CHARS_PER_ROW; i++) {
		for (int j = 0; j < 17; j++) {
			add_character_to_screen(' ', i, j);
		}
	}

	_battery_sub = orb_subscribe(ORB_ID(battery_status));
	// _local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

	return PX4_OK;

fail:
	return PX4_ERROR;

}


int
OSDatxxxx::probe()
{
	uint8_t data = 0;
	int ret = PX4_OK;

	ret |= writeRegister(0x00, 0x01); //disable video output
	ret |= readRegister(0x00, &data, 1);

	if (data != 1 || ret != PX4_OK) {
		printf("probe error\n");
	}

	return ret;
}

int
OSDatxxxx::init_osd()
{
	int ret = PX4_OK;
	uint8_t data = OSD_ZERO_BYTE;

	if (_tx_mode == 2) {
		data |= OSD_PAL_TX_MODE;
	}

	ret |= writeRegister(0x00, data);
	ret |= writeRegister(0x04, OSD_ZERO_BYTE);

	enable_screen();

	// writeRegister(0x00, 0x48) != PX4_OK) { //DMM set to 0
	// 	goto fail;
	// }

	return ret;

}


int
OSDatxxxx::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	return -1;
}

ssize_t
OSDatxxxx::read(device::file_t *filp, char *buffer, size_t buflen)
{
	return -1;
}


int
OSDatxxxx::readRegister(unsigned reg, uint8_t *data, unsigned count)
{
	uint8_t cmd[5]; 					// read up to 4 bytes
	int ret;

	cmd[0] = DIR_READ(reg);

	ret = transfer(&cmd[0], &cmd[0], count + 1);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	memcpy(&data[0], &cmd[1], count);

	return ret;

}


int
OSDatxxxx::writeRegister(unsigned reg, uint8_t data)
{
	uint8_t cmd[2]; 						// write 1 byte
	int ret;

	cmd[0] = DIR_WRITE(reg);
	cmd[1] = data;

	ret = transfer(&cmd[0], nullptr, 2);

	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_LOG("spi::transfer returned %d", ret);
		return ret;
	}

	return ret;

}

int
OSDatxxxx::add_character_to_screen(char c, uint8_t pos_x, uint8_t pos_y)
{

	uint16_t position = (OSD_CHARS_PER_ROW * pos_y) + pos_x;
	uint8_t position_lsb;
	int ret = PX4_OK;

	if (position > 0xFF) {
		position_lsb = static_cast<uint8_t>(position) - 0xFF;
		ret |= writeRegister(0x05, 0x01); //DMAH

	} else {
		position_lsb = static_cast<uint8_t>(position);
		ret |= writeRegister(0x05, 0x00); //DMAH
	}

	ret |= writeRegister(0x06, position_lsb); //DMAL
	ret |= writeRegister(0x07, c);

	return ret;
}

int
OSDatxxxx::add_battery_symbol(uint8_t pos_x, uint8_t pos_y)
{
	return add_character_to_screen(146, pos_x, pos_y);
}

int
OSDatxxxx::add_battery_info(uint8_t pos_x, uint8_t pos_y)
{
	char buf[5];
	int ret = PX4_OK;

	sprintf(buf, "%4.2f", (double)_battery_voltage_filtered_v);

	for (int i = 0; i < 5; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen('V', pos_x + 5, pos_y);

	pos_y++;

	sprintf(buf, "%4d", (int)_battery_discharge_mah);

	for (int i = 0; i < 5; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen(7, pos_x + 5, pos_y); // mAh symbol

	return ret;
}

int
OSDatxxxx::add_altitude_symbol(uint8_t pos_x, uint8_t pos_y)
{
	return add_character_to_screen(154, pos_x, pos_y);
}

int
OSDatxxxx::add_altitude(uint8_t pos_x, uint8_t pos_y)
{
	char buf[5];
	int ret = PX4_OK;

	sprintf(buf, "%4.2f", (double)_local_position_z);

	for (int i = 0; i < 5; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	ret |= add_character_to_screen('m', pos_x + 5, pos_y);

	return ret;
}

int
OSDatxxxx::add_flighttime_symbol(uint8_t pos_x, uint8_t pos_y)
{
	return add_character_to_screen(112, pos_x, pos_y);
}

int
OSDatxxxx::add_flighttime(float flight_time, uint8_t pos_x, uint8_t pos_y)
{
	char buf[6];
	int ret = PX4_OK;

	sprintf(buf, "%5.1f", (double)flight_time);

	for (int i = 0; i < 6; i++) {
		ret |= add_character_to_screen(buf[i], pos_x + i, pos_y);
	}

	return ret;
}

int
OSDatxxxx::enable_screen()
{
	uint8_t data;
	int ret = PX4_OK;

	ret |= readRegister(0x00, &data, 1);
	ret |= writeRegister(0x00, data | 0x48);

	return ret;
}

int
OSDatxxxx::disable_screen()
{
	uint8_t data;
	int ret = PX4_OK;

	ret |= readRegister(0x00, &data, 1);
	ret |= writeRegister(0x00, data & 0xF7);

	return ret;
}


int
OSDatxxxx::update_topics()//TODO have an argument to choose what to update and return validity
{
	struct battery_status_s battery = {};
	// struct vehicle_local_position_s local_position = {};
	struct vehicle_status_s vehicle_status = {};

	bool updated = false;

	/* update battery subscription */
	orb_check(_battery_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_sub, &battery);

		if (battery.connected) {
			_battery_voltage_filtered_v = battery.voltage_filtered_v;
			_battery_discharge_mah = battery.discharged_mah;
			_battery_valid = true;

		} else {
			_battery_valid = false;
		}
	}

	/* update vehicle local position subscription */
	// orb_check(_local_position_sub, &updated);

	// if (updated) {
	// 	if (local_position.z_valid) {
	// 		_local_position_z = -local_position.z;
	// 		_local_position_valid = true;

	// 	} else {
	// 		_local_position_valid = false;
	// 	}
	// }

	/* update vehicle status subscription */
	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		if (vehicle_status.arming_state == 2 && _arming_state == 1) {
			_arming_timestamp = hrt_absolute_time();
			_arming_state = 2;

		} else if (vehicle_status.arming_state == 1 && _arming_state == 2) {
			_arming_state = 1;
		}


		if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_ACRO) {
			add_character_to_screen('A', 1, 7);
			add_character_to_screen('C', 2, 7);
			add_character_to_screen('R', 3, 7);
			add_character_to_screen('O', 4, 7);

		} else if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_STAB) {
			add_character_to_screen('S', 1, 8);
			add_character_to_screen('T', 2, 8);
			add_character_to_screen('A', 3, 8);
			add_character_to_screen('B', 4, 8);
			add_character_to_screen('I', 5, 8);
			add_character_to_screen('L', 6, 8);
			add_character_to_screen('I', 7, 8);
			add_character_to_screen('Z', 8, 8);
			add_character_to_screen('E', 9, 8);

		} else if (vehicle_status.nav_state == vehicle_status.NAVIGATION_STATE_MANUAL) {
			// add_character_to_screen('A', uint8_t pos_x, uint8_t pos_y)
			// add_character_to_screen('C', uint8_t pos_x, uint8_t pos_y)
			// add_character_to_screen('R', uint8_t pos_x, uint8_t pos_y)
			// add_character_to_screen('O', uint8_t pos_x, uint8_t pos_y)
		}

		// _arming_state = vehicle_status.arming_state;
	}

	return PX4_OK;
}


int
OSDatxxxx::update_screen()
{
	int ret = PX4_OK;

	if (_battery_valid) {
		ret |= add_battery_symbol(1, 1);
		ret |= add_battery_info(2, 1);
	}

	if (_local_position_valid) {
		ret |= add_altitude_symbol(1, 3);
		ret |= add_altitude(2, 3);
	}

	// if (_arming_state == 2) {
	float flight_time_sec = static_cast<float>((hrt_absolute_time() - _arming_timestamp) / (1.0e6));
	ret |= add_flighttime_symbol(1, 5);
	ret |= add_flighttime(flight_time_sec, 2, 5);
	// }

	// enable_screen();

	return ret;

}


void
OSDatxxxx::start()
{
	/* schedule a cycle to start things */
	work_queue(LPWORK, &_work, (worker_t)&OSDatxxxx::cycle_trampoline, this, USEC2TICK(OSD_US));
}

void
OSDatxxxx::stop()
{
	work_cancel(LPWORK, &_work);
}

int
OSDatxxxx::reset()
{
	int ret = writeRegister(0x00, 0x02);
	usleep(100);

	return ret;
}

void
OSDatxxxx::cycle_trampoline(void *arg)
{
	OSDatxxxx *dev = (OSDatxxxx *)arg;

	dev->cycle();
}

void
OSDatxxxx::cycle()
{
	update_topics();

	if (_battery_valid || _local_position_valid || _arming_state > 1) {
		update_screen();
	}

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(LPWORK,
		   &_work,
		   (worker_t)&OSDatxxxx::cycle_trampoline,
		   this,
		   USEC2TICK(OSD_UPDATE_RATE));

}

void
OSDatxxxx::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("battery_status: %.3f\n", (double)_battery_voltage_filtered_v);
	printf("arming_state: %d\n", _arming_state);
	printf("arming_timestamp: %5.1f\n", (double)_arming_timestamp);

}


/**
 * Local functions in support of the shell command.
 */
namespace osd
{

OSDatxxxx	*g_dev;

int	start(int spi_bus);
int	stop();
int	info();
void usage();


/**
 * Start the driver.
 */
int
start(int spi_bus)
{
	int fd;

	if (g_dev != nullptr) {
		PX4_ERR("already started");
		goto fail;
	}

	/* create the driver */
	g_dev = new OSDatxxxx(spi_bus);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	fd = open(OSD_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	g_dev->start();

	return PX4_OK;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	PX4_ERR("driver start failed");
	return PX4_ERROR;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

/**
 * Print a little info about how to start/stop/use the driver
 */
void usage()
{
	PX4_INFO("usage: atxxxx {start|stop|status'}");
	PX4_INFO("    [-b SPI_BUS]");
}

} // namespace osd


int
atxxxx_main(int argc, char *argv[])
{
	if (argc < 2) {
		osd::usage();
		return PX4_ERROR;
	}

	// don't exit from getopt loop to leave getopt global variables in consistent state,
	// set error flag instead
	bool err_flag = false;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int spi_bus = OSD_BUS;

	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			spi_bus = (uint8_t)atoi(myoptarg);
			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		osd::usage();
		return PX4_ERROR;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		return osd::start(spi_bus);
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return osd::stop();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return osd::info();
	}

	osd::usage();
	return PX4_ERROR;
}
