/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file blinkm_systemstate.cpp
 *
 * Driver for the BlinkM LED controller connected via I2C.
 */



#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>

#include <drivers/drv_blinkm.h>

#include <nuttx/wqueue.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <systemlib/systemlib.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_status.h>

class BlinkMSystemstate : public device::I2C
{
public:
	BlinkMSystemstate(int bus);
	~BlinkMSystemstate();

	virtual int		init();
	virtual int		probe();

	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	static const char	*script_names[];

private:
	enum ScriptID {
		USER		= 0,
		RGB,
		WHITE_FLASH,
		RED_FLASH,
		GREEN_FLASH,
		BLUE_FLASH,
		CYAN_FLASH,
		MAGENTA_FLASH,
		YELLOW_FLASH,
		BLACK,
		HUE_CYCLE,
		MOOD_LIGHT,
		VIRTUAL_CANDLE,
		WATER_REFLECTIONS,
		OLD_NEON,
		THE_SEASONS,
		THUNDERSTORM,
		STOP_LIGHT,
		MORSE_CODE
	};

	work_s			_work;

	static int led_color_1;
	static int led_color_2;
	static int led_color_3;
	static int led_color_4;
	static int led_color_5;
	static int led_color_6;
	static int led_blink;


	void 			setLEDColor(int ledcolor);
	static void		led_trampoline(void *arg);
	void			led();
	void			setLEDPattern(int c1, int c2, int c3, int c4, int c5, int c6, int doBlink);

	int			set_rgb(uint8_t r, uint8_t g, uint8_t b);

	int			fade_rgb(uint8_t r, uint8_t g, uint8_t b);
	int			fade_hsb(uint8_t h, uint8_t s, uint8_t b);

	int			fade_rgb_random(uint8_t r, uint8_t g, uint8_t b);
	int			fade_hsb_random(uint8_t h, uint8_t s, uint8_t b);

	int			set_fade_speed(uint8_t s);

	int			play_script(uint8_t script_id);
	int			play_script(const char *script_name);
	int			stop_script();

	int			write_script_line(uint8_t line, uint8_t ticks, uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3);
	int			read_script_line(uint8_t line, uint8_t &ticks, uint8_t cmd[4]);
	int			set_script(uint8_t length, uint8_t repeats);

	int			get_rgb(uint8_t &r, uint8_t &g, uint8_t &b);

	int			get_firmware_version(uint8_t version[2]);
};

/* for now, we only support one BlinkM */
namespace
{
	BlinkMSystemstate *g_blinkm;
}

/* list of script names, must match script ID numbers */
const char *BlinkMSystemstate::script_names[] = {
	"USER",
	"RGB",
	"WHITE_FLASH",
	"RED_FLASH",
	"GREEN_FLASH",
	"BLUE_FLASH",
	"CYAN_FLASH",
	"MAGENTA_FLASH",
	"YELLOW_FLASH",
	"BLACK",
	"HUE_CYCLE",
	"MOOD_LIGHT",
	"VIRTUAL_CANDLE",
	"WATER_REFLECTIONS",
	"OLD_NEON",
	"THE_SEASONS",
	"THUNDERSTORM",
	"STOP_LIGHT",
	"MORSE_CODE",
	nullptr
};


#define MAX_CELL_VOLTAGE 	43	/* cell voltage if full charged */
#define WARN_CELL_VOLTAGE 	33	/* warn, if cell voltage is below this */
#define MIN_CELL_VOLTAGE 	31	/* minimum cell voltage - emergency*/

#define LED_OFF  		0
#define LED_RED  		1
#define LED_YELLOW  	2
#define LED_PURPLE  	3
#define LED_GREEN  		4
#define LED_BLUE  		5
#define LED_WHITE  		6
#define LED_ONTIME  	150
#define LED_OFFTIME  	150
#define LED_BLINK  		1
#define LED_NOBLINK  	0

#define VEHICLE_FLIGHT_MODE_MANUAL			0
#define VEHICLE_FLIGHT_MODE_ATTITUDE		1
#define VEHICLE_FLIGHT_MODE_STABILIZED		2
#define VEHICLE_FLIGHT_MODE_AUTO			3

int BlinkMSystemstate::led_color_1 = LED_OFF;
int BlinkMSystemstate::led_color_2 = LED_OFF;
int BlinkMSystemstate::led_color_3 = LED_OFF;
int BlinkMSystemstate::led_color_4 = LED_OFF;
int BlinkMSystemstate::led_color_5 = LED_OFF;
int BlinkMSystemstate::led_color_6 = LED_OFF;
int BlinkMSystemstate::led_blink = LED_NOBLINK;

extern "C" __EXPORT int blinkm_systemstate_main(int argc, char *argv[]);

BlinkMSystemstate::BlinkMSystemstate(int bus) :
	I2C("blinkm", BLINKM_DEVICE_PATH, bus, 0x09, 100000)
{
	memset(&_work, 0, sizeof(_work));
}

BlinkMSystemstate::~BlinkMSystemstate()
{
}

int
BlinkMSystemstate::init()
{
	int ret;

	ret = I2C::init();

	if (ret != OK) {
		warnx("I2C init failed");
		return ret;
	}

	/* set some sensible defaults */
	set_fade_speed(255);

	/* turn off by default */
	play_script(BLACK);
	set_fade_speed(255);
	stop_script();
	set_rgb(0,0,0);

	/* start the system monitor as a low-priority workqueue entry */
	work_queue(LPWORK, &_work, (worker_t)&BlinkMSystemstate::led_trampoline, this, 1);

	return OK;
}

int
BlinkMSystemstate::probe()
{
	uint8_t version[2];
	int ret;

	ret = get_firmware_version(version);

	if (ret == OK)
		log("found BlinkM firmware version %c%c", version[1], version[0]);

	return ret;
}

int
BlinkMSystemstate::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = ENOTTY;

	switch (cmd) {
	case BLINKM_PLAY_SCRIPT_NAMED:
		if (arg == 0) {
			ret = EINVAL;
			break;
		}
		ret = play_script((const char *)arg);
		break;

	case BLINKM_PLAY_SCRIPT:
		ret = play_script(arg);
		break;

	case BLINKM_SET_USER_SCRIPT: {
		if (arg == 0) {
			ret = EINVAL;
			break;
		}

		unsigned lines = 0;
		const uint8_t *script = (const uint8_t *)arg;

		while ((lines < 50) && (script[1] != 0)) {
			ret = write_script_line(lines, script[0], script[1], script[2], script[3], script[4]);
			if (ret != OK)
				break;
			script += 5;
		}
		if (ret == OK)
			ret = set_script(lines, 0);
		break;
	}

	default:
		break;
	}

	return ret;
}


void
BlinkMSystemstate::led_trampoline(void *arg)
{
	BlinkMSystemstate *bm = (BlinkMSystemstate *)arg;

	bm->led();
}



void
BlinkMSystemstate::led()
{
	static bool topic_initialized = false;
	static bool detected_cells_blinked = false;
	static int detected_cells_runcount = 0;
	static int sensor_sub_fd;
	int system_voltage = 0;
	static int num_of_cells = 0;
	static int t_led_color_1 = 0;
	static int t_led_color_2 = 0;
	static int t_led_color_3 = 0;
	static int t_led_color_4 = 0;
	static int t_led_color_5 = 0;
	static int t_led_color_6 = 0;
	static int t_led_blink = 0;

	int poll_ret;
	static int led_thread_runcount=1;
	static bool led_thread_ready = true;
	static int led_interval = 1000;

	if(!topic_initialized) {
		sensor_sub_fd = orb_subscribe(ORB_ID(vehicle_status));
		orb_set_interval(sensor_sub_fd, 1000);
		topic_initialized = true;
	}

	pollfd fds[1];
	fds[0].fd = sensor_sub_fd;
	fds[0].events = POLLIN;

	if(led_thread_ready == true) {
		if(!detected_cells_blinked) {
			if(num_of_cells > 0) {
				t_led_color_1 = LED_PURPLE;
			}
			if(num_of_cells > 1) {
				t_led_color_2 = LED_PURPLE;
			}
			if(num_of_cells > 2) {
				t_led_color_3 = LED_PURPLE;
			}
			if(num_of_cells > 3) {
				t_led_color_4 = LED_PURPLE;
			}
			if(num_of_cells > 4) {
				t_led_color_5 = LED_PURPLE;
			}
			t_led_color_6 = LED_OFF;
			t_led_blink = LED_BLINK;
		} else {
			t_led_color_1 = BlinkMSystemstate::led_color_1;
			t_led_color_2 = BlinkMSystemstate::led_color_2;
			t_led_color_3 = BlinkMSystemstate::led_color_3;
			t_led_color_4 = BlinkMSystemstate::led_color_4;
			t_led_color_5 = BlinkMSystemstate::led_color_5;
			t_led_color_6 = BlinkMSystemstate::led_color_6;
			t_led_blink = BlinkMSystemstate::led_blink;
		}
		led_thread_ready = false;
	}

	switch(led_thread_runcount) {
		case 1:	// 1. LED on
			BlinkMSystemstate::setLEDColor(t_led_color_1);
			led_thread_runcount++;
			led_interval = LED_ONTIME;
			break;
		case 2:	// 1. LED off
			if(t_led_blink == LED_BLINK) {
				BlinkMSystemstate::setLEDColor(LED_OFF);
			}
			led_thread_runcount++;
			led_interval = LED_OFFTIME;
			break;
		case 3:	// 2. LED on
			BlinkMSystemstate::setLEDColor(t_led_color_2);
			led_thread_runcount++;
			led_interval = LED_ONTIME;
			break;
		case 4:	// 2. LED off
			if(t_led_blink == LED_BLINK) {
				BlinkMSystemstate::setLEDColor(LED_OFF);
			}
			led_thread_runcount++;
			led_interval = LED_OFFTIME;
			break;
		case 5:	// 3. LED on
			BlinkMSystemstate::setLEDColor(t_led_color_3);
			led_thread_runcount++;
			led_interval = LED_ONTIME;
			break;
		case 6:	// 3. LED off
			if(t_led_blink == LED_BLINK) {
				BlinkMSystemstate::setLEDColor(LED_OFF);
			}
			led_thread_runcount++;
			led_interval = LED_OFFTIME;
			break;
		case 7:	// 4. LED on
			BlinkMSystemstate::setLEDColor(t_led_color_4);
			led_thread_runcount++;
			led_interval = LED_ONTIME;
			break;
		case 8:	// 4. LED off
			if(t_led_blink == LED_BLINK) {
				BlinkMSystemstate::setLEDColor(LED_OFF);
			}
			led_thread_runcount++;
			led_interval = LED_OFFTIME;
			break;
		case 9:	// 5. LED on
			BlinkMSystemstate::setLEDColor(t_led_color_5);
			led_thread_runcount++;
			led_interval = LED_ONTIME;
			break;
		case 10:	// 5. LED off
			if(t_led_blink == LED_BLINK) {
				BlinkMSystemstate::setLEDColor(LED_OFF);
			}
			led_thread_runcount++;
			led_interval = LED_OFFTIME;
			break;
		case 11:	// 6. LED on
			BlinkMSystemstate::setLEDColor(t_led_color_6);
			led_thread_runcount++;
			led_interval = LED_ONTIME;
			break;
		case 12:	// 6. LED off
			if(t_led_blink == LED_BLINK) {
				BlinkMSystemstate::setLEDColor(LED_OFF);
			}

			poll_ret = ::poll(&fds[0],1,1000);

			if (poll_ret == 0) {
				/* this means none of our providers is giving us data */
				printf("[blinkm_systemstate_sensor] Got no data within a second\n");
			} else if (poll_ret < 0) {
				/* this is seriously bad - should be an emergency */
				log("poll error %d", errno);
				usleep(1000000);
			} else {
				if (fds[0].revents & POLLIN) {
					/* obtained data for the first file descriptor */
					struct vehicle_status_s raw;
					/* copy sensors raw data into local buffer */
					orb_copy(ORB_ID(vehicle_status), sensor_sub_fd, &raw);

					system_voltage = (int)raw.voltage_battery*10;

					if(num_of_cells == 0) {
						/* checking for lipo cells */
						printf("[blinkm_systemstate_sensor] checking cells\n");
						for(num_of_cells = 2; num_of_cells < 7; num_of_cells++) {
							if(system_voltage < num_of_cells * MAX_CELL_VOLTAGE) break;
						}
						printf("[blinkm_systemstate_sensor] cells found:%u\n", num_of_cells);

					} else {
						if((system_voltage <= WARN_CELL_VOLTAGE * num_of_cells) &&
							(system_voltage > MIN_CELL_VOLTAGE * num_of_cells)) {

							/* battery low warning stage 1 - fist warning yellow blink*/
							BlinkMSystemstate::setLEDPattern(LED_YELLOW,
															LED_YELLOW,
															LED_YELLOW,
															LED_YELLOW,
															LED_YELLOW,
															LED_YELLOW,
															LED_BLINK);

						} else if(system_voltage <= MIN_CELL_VOLTAGE * num_of_cells) {

							/* battery low warning stage 2 - second warning red blink - emergency */
							BlinkMSystemstate::setLEDPattern(LED_RED,
															LED_RED,
															LED_RED,
															LED_RED,
															LED_RED,
															LED_RED,
															LED_BLINK);

						} else {
							/* no battery warnings here */
							if(raw.flag_system_armed == false) {
								/* system not armed */
								BlinkMSystemstate::setLEDPattern(LED_RED,
																LED_RED,
																LED_RED,
																LED_RED,
																LED_RED,
																LED_RED,
																LED_NOBLINK);
							} else {
								switch((int)raw.flight_mode) {
									case VEHICLE_FLIGHT_MODE_MANUAL:
										BlinkMSystemstate::setLEDPattern(LED_RED,
																		LED_OFF,
																		LED_OFF,
																		LED_YELLOW,
																		LED_OFF,
																		LED_OFF,
																		LED_BLINK);
										break;

									case VEHICLE_FLIGHT_MODE_ATTITUDE:
										BlinkMSystemstate::setLEDPattern(LED_PURPLE,
																		LED_PURPLE,
																		LED_PURPLE,
																		LED_YELLOW,
																		LED_OFF,
																		LED_OFF,
																		LED_BLINK);
										break;

									case VEHICLE_FLIGHT_MODE_STABILIZED:
										BlinkMSystemstate::setLEDPattern(LED_BLUE,
																		LED_BLUE,
																		LED_BLUE,
																		LED_YELLOW,
																		LED_OFF,
																		LED_OFF,
																		LED_BLINK);
										break;

									case VEHICLE_FLIGHT_MODE_AUTO:
										BlinkMSystemstate::setLEDPattern(LED_GREEN,
																		LED_GREEN,
																		LED_GREEN,
																		LED_YELLOW,
																		LED_OFF,
																		LED_OFF,
																		LED_BLINK);
										break;
								}
							}
						}
					}

					/*printf( "[blinkm_systemstate_sensor] Volt:%8.4f\tLoad:%8.4f\tArmed:%4u\tMode:%4u\tCells:%4u\n",
					raw.voltage_battery,
					raw.load,
					raw.flag_system_armed,
					raw.flight_mode,
					BlinkMSystemstate::num_of_cells);*/
				}
			}

			led_thread_runcount=1;
			led_thread_ready = true;
			led_interval = LED_OFFTIME;

			if(detected_cells_runcount < 5){
				detected_cells_runcount++;

			} else {
				detected_cells_blinked = true;
			}


			break;
		default:
			led_thread_runcount=1;
			t_led_blink = 0;
			led_thread_ready = true;
			break;
	}

	/* re-queue ourselves to run again later */
	work_queue(LPWORK, &_work, (worker_t)&BlinkMSystemstate::led_trampoline, this, led_interval);
}

void BlinkMSystemstate::setLEDPattern(int c1, int c2, int c3, int c4, int c5, int c6, int doBlink) {
	BlinkMSystemstate::led_color_1 = c1;
	BlinkMSystemstate::led_color_2 = c2;
	BlinkMSystemstate::led_color_3 = c3;
	BlinkMSystemstate::led_color_4 = c4;
	BlinkMSystemstate::led_color_5 = c5;
	BlinkMSystemstate::led_color_6 = c6;
	BlinkMSystemstate::led_blink = doBlink;
}

void BlinkMSystemstate::setLEDColor(int ledcolor) {
	switch (ledcolor) {
		case 0:	// off
			BlinkMSystemstate::set_rgb(0,0,0);
			break;
		case 1:	// red
			BlinkMSystemstate::set_rgb(255,0,0);
			break;
		case 2:	// yellow
			BlinkMSystemstate::set_rgb(255,70,0);
			break;
		case 3:	// purple
			BlinkMSystemstate::set_rgb(255,0,255);
			break;
		case 4:	// green
			BlinkMSystemstate::set_rgb(0,255,0);
			break;
		case 5:	// blue
			BlinkMSystemstate::set_rgb(0,0,255);
			break;
		case 6:	// white
			BlinkMSystemstate::set_rgb(255,255,255);
			break;
	}
}

int
BlinkMSystemstate::set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t msg[4] = { 'n', r, g, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkMSystemstate::fade_rgb(uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t msg[4] = { 'c', r, g, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkMSystemstate::fade_hsb(uint8_t h, uint8_t s, uint8_t b)
{
	const uint8_t msg[4] = { 'h', h, s, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkMSystemstate::fade_rgb_random(uint8_t r, uint8_t g, uint8_t b)
{
	const uint8_t msg[4] = { 'C', r, g, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkMSystemstate::fade_hsb_random(uint8_t h, uint8_t s, uint8_t b)
{
	const uint8_t msg[4] = { 'H', h, s, b };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkMSystemstate::set_fade_speed(uint8_t s)
{
	const uint8_t msg[2] = { 'f', s };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkMSystemstate::play_script(uint8_t script_id)
{
	const uint8_t msg[4] = { 'p', script_id, 0, 0 };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkMSystemstate::play_script(const char *script_name)
{
	/* handle HTML colour encoding */
	if (isxdigit(script_name[0]) && (strlen(script_name) == 6)) {
		char code[3];
		uint8_t r, g, b;

		code[2] = '\0';

		code[0] = script_name[1];
		code[1] = script_name[2];
		r = strtol(code, 0, 16);
		code[0] = script_name[3];
		code[1] = script_name[4];
		g = strtol(code, 0, 16);
		code[0] = script_name[5];
		code[1] = script_name[6];
		b = strtol(code, 0, 16);

		stop_script();
		return set_rgb(r, g, b);
	}

	for (unsigned i = 0; script_names[i] != nullptr; i++)
		if (!strcasecmp(script_name, script_names[i]))
			return play_script(i);

	return -1;
}

int
BlinkMSystemstate::stop_script()
{
	const uint8_t msg[1] = { 'o' };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkMSystemstate::write_script_line(uint8_t line, uint8_t ticks, uint8_t cmd, uint8_t arg1, uint8_t arg2, uint8_t arg3)
{
	const uint8_t msg[8] = { 'W', 0, line, ticks, cmd, arg1, arg2, arg3 };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkMSystemstate::read_script_line(uint8_t line, uint8_t &ticks, uint8_t cmd[4])
{
	const uint8_t msg[3] = { 'R', 0, line };
	uint8_t result[5];

	int ret = transfer(msg, sizeof(msg), result, sizeof(result));

	if (ret == OK) {
		ticks = result[0];
		cmd[0] = result[1];
		cmd[1] = result[2];
		cmd[2] = result[3];
		cmd[3] = result[4];
	}

	return ret;
}

int
BlinkMSystemstate::set_script(uint8_t len, uint8_t repeats)
{
	const uint8_t msg[4] = { 'L', 0, len, repeats };

	return transfer(msg, sizeof(msg), nullptr, 0);
}

int
BlinkMSystemstate::get_rgb(uint8_t &r, uint8_t &g, uint8_t &b)
{
	const uint8_t msg = 'g';
	uint8_t result[3];

	int ret = transfer(&msg, sizeof(msg), result, sizeof(result));

	if (ret == OK) {
		r = result[0];
		g = result[1];
		b = result[2];
	}

	return ret;
}

int
BlinkMSystemstate::get_firmware_version(uint8_t version[2])
{
	const uint8_t msg = 'Z';

	return transfer(&msg, sizeof(msg), version, sizeof(version));
}

int
blinkm_systemstate_main(int argc, char *argv[])
{
	if (!strcmp(argv[1], "start")) {
		if (g_blinkm != nullptr)
			errx(1, "already started");

		g_blinkm = new BlinkMSystemstate(3);

		if (g_blinkm == nullptr)
			errx(1, "new failed");

		if (OK != g_blinkm->init()) {
			delete g_blinkm;
			g_blinkm = nullptr;
			errx(1, "init failed");
		}

		exit(0);
	}

	if (g_blinkm == nullptr)
		errx(1, "not started");

	if (!strcmp(argv[1], "list")) {
		for (unsigned i = 0; BlinkMSystemstate::script_names[i] != nullptr; i++)
			fprintf(stderr, "    %s\n", BlinkMSystemstate::script_names[i]);
		fprintf(stderr, "    <html color number>\n");
		exit(0);
	}

	/* things that require access to the device */
	int fd = open(BLINKM_DEVICE_PATH, 0);
	if (fd < 0)
		err(1, "can't open BlinkM device");

	if (ioctl(fd, BLINKM_PLAY_SCRIPT_NAMED, (unsigned long)argv[1]) == OK)
		exit(0);

	errx(1, "missing command, try 'start', 'list' or a script name");
}
