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
 * @author Beat Küng <beat-kueng@gmx.net>
 *
 * Driver for the ATXXXX chip on the omnibus fcu connected via SPI.
 */

#include "atxxxx.h"

struct work_s OSDatxxxx::_work = {};


OSDatxxxx::OSDatxxxx(int bus) :
	SPI("OSD", OSD_DEVICE_PATH, bus, OSD_SPIDEV, SPIDEV_MODE0, OSD_SPI_BUS_SPEED),
	ModuleParams(nullptr)
{
	_battery_sub = orb_subscribe(ORB_ID(battery_status));
	_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
}

OSDatxxxx::~OSDatxxxx()
{
	orb_unsubscribe(_battery_sub);
	orb_unsubscribe(_local_position_sub);
	orb_unsubscribe(_vehicle_status_sub);
}

int OSDatxxxx::task_spawn(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int spi_bus = OSD_BUS;

	while ((ch = px4_getopt(argc, argv, "b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			spi_bus = (uint8_t)atoi(myoptarg);
			break;
		}
	}

	int ret = work_queue(LPWORK, &_work, (worker_t)&OSDatxxxx::initialize_trampoline, (void *)(long)spi_bus, 0);

	if (ret < 0) {
		return ret;
	}

	ret = wait_until_running();

	if (ret < 0) {
		return ret;
	}

	_task_id = task_id_is_work_queue;

	return 0;
}


int
OSDatxxxx::init()
{
	/* do SPI init (and probe) first */
	int ret = SPI::init();

	if (ret != PX4_OK) {
		return ret;
	}

	if ((ret = reset()) != PX4_OK) {
		return ret;
	}

	if ((ret = init_osd()) != PX4_OK) {
		return ret;
	}

	for (int i = 0; i < OSD_CHARS_PER_ROW; i++) {
		for (int j = 0; j < 17; j++) {
			add_character_to_screen(' ', i, j);
		}
	}

	return ret;
}

int OSDatxxxx::start()
{
	if (is_running()) {
		return 0;
	}

	init();

	// Kick off the cycling. We can call it directly because we're already in the work queue context.
	cycle();

	return 0;
}

void OSDatxxxx::initialize_trampoline(void *arg)
{
	OSDatxxxx *osd = new OSDatxxxx((long)arg);

	if (!osd) {
		PX4_ERR("alloc failed");
		return;
	}

	osd->start();
	_object.store(osd);
}

void OSDatxxxx::cycle_trampoline(void *arg)
{
	OSDatxxxx *obj = reinterpret_cast<OSDatxxxx *>(arg);

	obj->cycle();
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

	if (_param_atxxxx_cfg.get() == 2) {
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
OSDatxxxx::readRegister(unsigned reg, uint8_t *data, unsigned count)
{
	uint8_t cmd[5]; 					// read up to 4 bytes
	int ret;

	cmd[0] = DIR_READ(reg);

	ret = transfer(&cmd[0], &cmd[0], count + 1);

	if (OK != ret) {
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

int
OSDatxxxx::reset()
{
	int ret = writeRegister(0x00, 0x02);
	usleep(100);

	return ret;
}

void
OSDatxxxx::cycle()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

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

int OSDatxxxx::print_usage(const char *reason)
{
	if (reason) {
		printf("%s\n\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
OSD driver for the ATXXXX chip that is mounted on the OmnibusF4SD board for example.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("atxxxx", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_INT('b', -1, 0, 100, "SPI bus (default: use board-specific bus)", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int OSDatxxxx::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognized command");
}

int atxxxx_main(int argc, char *argv[])
{
	return OSDatxxxx::main(argc, argv);
}
