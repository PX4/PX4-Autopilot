/****************************************************************************
 *
 *   Copyright (C) 2012, 2013 PX4 Development Team. All rights reserved.
 *   Author: Randy Mackay <rmackay9@yahoo.com>
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
 * @file oreoled.cpp
 *
 * Driver for oreoled ESCs found in solo, connected via I2C.
 *
 */

#include <px4_config.h>

#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <ctype.h>
#include <sys/stat.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>

#include <board_config.h>

#include <drivers/drv_oreoled.h>
#include <drivers/device/ringbuffer.h>

#define OREOLED_NUM_LEDS		4			///< maximum number of LEDs the oreo led driver can support
#define OREOLED_BASE_I2C_ADDR	0x68		///< base i2c address (7-bit)
#define OPEOLED_I2C_RETRYCOUNT  2           ///< i2c retry count
#define OREOLED_TIMEOUT_USEC	2000000U	///< timeout looking for oreoleds 2 seconds after startup
#define OREOLED_GENERALCALL_US	4000000U	///< general call sent every 4 seconds
#define OREOLED_GENERALCALL_CMD	0x00		///< general call command sent at regular intervals

#define OREOLED_STARTUP_INTERVAL_US		(1000000U / 10U)	///< time in microseconds, measure at 10hz
#define OREOLED_UPDATE_INTERVAL_US		(1000000U / 50U)	///< time in microseconds, measure at 10hz

#define OREOLED_CMD_QUEUE_SIZE	10		///< up to 10 messages can be queued up to send to the LEDs

class OREOLED : public device::I2C
{
public:
	OREOLED(int bus, int i2c_addr, bool autoupdate, bool alwaysupdate);
	virtual ~OREOLED();

	virtual int		init();
	virtual int		probe();
	virtual int		info();
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/* send general call on I2C bus to syncronise all LEDs */
	int				send_general_call();

	/* send cmd to an LEDs (used for testing only) */
	int				send_cmd(oreoled_cmd_t sb);

	/* returns true once the driver finished bootloading and ready for commands */
	bool			is_ready();

private:

	/**
	 * Start periodic updates to the LEDs
	 */
	void			start();

	/**
	 * Stop periodic updates to the LEDs
	 */
	void			stop();

	/**
	 * static function that is called by worker queue
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * update the colours displayed by the LEDs
	 */
	void			cycle();

	int				bootloader_app_reset(int led_num);
	int				bootloader_app_ping(int led_num);
	uint16_t		bootloader_inapp_checksum(int led_num);
	int				bootloader_ping(int led_num);
	uint8_t			bootloader_version(int led_num);
	uint16_t		bootloader_app_version(int led_num);
	uint16_t		bootloader_app_checksum(int led_num);
	int				bootloader_set_colour(int led_num, uint8_t red, uint8_t green);
	int				bootloader_flash(int led_num);
	int				bootloader_boot(int led_num);
	uint16_t		bootloader_fw_checksum(void);
	int				bootloader_coerce_healthy(void);

	/* internal variables */
	work_s			_work;							///< work queue for scheduling reads
	bool			_healthy[OREOLED_NUM_LEDS];		///< health of each LED
	bool			_in_boot[OREOLED_NUM_LEDS];		///< true for each LED that is in bootloader mode
	uint8_t			_num_healthy;					///< number of healthy LEDs
	ringbuffer::RingBuffer	*_cmd_queue;					///< buffer of commands to send to LEDs
	uint8_t			_num_inboot;					///< number of LEDs in bootloader
	uint64_t		_last_gencall;
	uint64_t		_start_time;					///< system time we first attempt to communicate with battery
	bool			_autoupdate;					///< true if the driver should update all LEDs
	bool			_alwaysupdate;					///< true if the driver should update all LEDs
	bool			_is_bootloading;				///< true if a bootloading operation is in progress
	bool			_is_ready;						///< set to true once the driver has completly initialised
	uint16_t		_fw_checksum;					///< the current 16bit XOR checksum of the built in oreoled firmware binary

	/* performance checking */
	perf_counter_t      _call_perf;
	perf_counter_t      _gcall_perf;
	perf_counter_t      _probe_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _reply_errors;
};

/* for now, we only support one OREOLED */
namespace
{
OREOLED *g_oreoled = nullptr;
}

void oreoled_usage();

extern "C" __EXPORT int oreoled_main(int argc, char *argv[]);

/* constructor */
OREOLED::OREOLED(int bus, int i2c_addr, bool autoupdate, bool alwaysupdate) :
	I2C("oreoled", OREOLED0_DEVICE_PATH, bus, i2c_addr, 100000),
	_work{},
	_num_healthy(0),
	_num_inboot(0),
	_cmd_queue(nullptr),
	_last_gencall(0),
	_autoupdate(autoupdate),
	_alwaysupdate(alwaysupdate),
	_is_bootloading(false),
	_is_ready(false),
	_fw_checksum(0x0000),
	_call_perf(perf_alloc(PC_ELAPSED, "oreoled_call")),
	_gcall_perf(perf_alloc(PC_ELAPSED, "oreoled_gcall")),
	_probe_perf(perf_alloc(PC_ELAPSED, "oreoled_probe")),
	_comms_errors(perf_alloc(PC_COUNT, "oreoled_comms_errors")),
	_reply_errors(perf_alloc(PC_COUNT, "oreoled_reply_errors"))
{
	/* initialise to unhealthy */
	memset(_healthy, 0, sizeof(_healthy));

	/* initialise to in application */
	memset(_in_boot, 0, sizeof(_in_boot));

	/* capture startup time */
	_start_time = hrt_absolute_time();
}

/* destructor */
OREOLED::~OREOLED()
{
	/* make sure we are truly inactive */
	stop();

	/* clear command queue */
	if (_cmd_queue != nullptr) {
		delete _cmd_queue;
	}

	/* free perf counters */
	perf_free(_call_perf);
	perf_free(_gcall_perf);
	perf_free(_probe_perf);
	perf_free(_comms_errors);
	perf_free(_reply_errors);
}

int
OREOLED::init()
{
	int ret;

	/* initialise I2C bus */
	ret = I2C::init();

	if (ret != OK) {
		return ret;
	}

	/* allocate command queue */
	_cmd_queue = new ringbuffer::RingBuffer(OREOLED_CMD_QUEUE_SIZE, sizeof(oreoled_cmd_t));

	if (_cmd_queue == nullptr) {
		return ENOTTY;

	} else {
		/* start work queue */
		start();
	}

	return OK;
}

int
OREOLED::probe()
{
	/* set retry count */
	_retries = OPEOLED_I2C_RETRYCOUNT;

	/* always return true */
	return OK;
}

int
OREOLED::info()
{
	/* print health info on each LED */
	for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
		if (!_healthy[i]) {
			DEVICE_LOG("oreo %u: BAD", (unsigned)i);

		} else {
			DEVICE_LOG("oreo %u: OK", (unsigned)i);
		}
	}

	/* display perf info */
	perf_print_counter(_call_perf);
	perf_print_counter(_gcall_perf);
	perf_print_counter(_probe_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_reply_errors);

	return OK;
}

void
OREOLED::start()
{
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&OREOLED::cycle_trampoline, this, 1);
}

void
OREOLED::stop()
{
	work_cancel(HPWORK, &_work);
}

void
OREOLED::cycle_trampoline(void *arg)
{
	OREOLED *dev = (OREOLED *)arg;

	/* check global oreoled and cycle */
	if (g_oreoled != nullptr) {
		dev->cycle();
	}
}

void
OREOLED::cycle()
{
	/* check time since startup */
	uint64_t now = hrt_absolute_time();
	bool startup_timeout = (now - _start_time > OREOLED_TIMEOUT_USEC);

	/* prepare the response buffer */
	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	/* during startup period keep searching for unhealthy LEDs */
	if (!startup_timeout && _num_healthy < OREOLED_NUM_LEDS) {
		/* prepare command to turn off LED */
		/* add two bytes of pre-amble to for higher signal to noise ratio */
		uint8_t msg[] = {0xAA, 0x55, OREOLED_PATTERN_OFF, 0x00};

		/* attempt to contact each unhealthy LED */
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (!_healthy[i]) {
				perf_begin(_probe_perf);

				/* set I2C address */
				set_address(OREOLED_BASE_I2C_ADDR + i);

				/* Calculate XOR CRC and append to the i2c write data */
				msg[sizeof(msg) - 1] = OREOLED_BASE_I2C_ADDR + i;

				for (uint8_t j = 0; j < sizeof(msg) - 1; j++) {
					msg[sizeof(msg) - 1] ^= msg[j];
				}

				/* send I2C command */
				if (transfer(msg, sizeof(msg), reply, 3) == OK) {
					if (reply[1] == OREOLED_BASE_I2C_ADDR + i &&
					    reply[2] == msg[sizeof(msg) - 1]) {
						DEVICE_LOG("oreoled %u ok - in bootloader", (unsigned)i);
						_healthy[i] = true;
						_num_healthy++;

						/* If slaves are in application record that so we can reset if we need to bootload */
						/* This additional check is required for LED firmwares below v1.3 and can be
						   deprecated once all LEDs in the wild have firmware >= v1.3 */
						if (bootloader_ping(i) == OK) {
							_in_boot[i] = true;
							_num_inboot++;
						}

						/* Check for a reply with a checksum offset of 1,
						   which indicates a response from firmwares >= 1.3 */

					} else if (reply[1] == OREOLED_BASE_I2C_ADDR + i &&
						   reply[2] == msg[sizeof(msg) - 1] + 1) {
						DEVICE_LOG("oreoled %u ok - in application", (unsigned)i);
						_healthy[i] = true;
						_num_healthy++;

					} else {
						DEVICE_LOG("oreo reply errors: %u", (unsigned)_reply_errors);
						perf_count(_reply_errors);
					}

				} else {
					perf_count(_comms_errors);
				}

				perf_end(_probe_perf);
			}
		}

		/* schedule another attempt in 0.1 sec */
		work_queue(HPWORK, &_work, (worker_t)&OREOLED::cycle_trampoline, this,
			   USEC2TICK(OREOLED_STARTUP_INTERVAL_US));
		return;

	} else if (_alwaysupdate) {
		/* reset each healthy LED */
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i] && !_in_boot[i]) {
				/* reset the LED if it's not in the bootloader */
				/* (this happens during a pixhawk OTA update, since the LEDs stay powered) */
				bootloader_app_reset(i);
			}
		}

		/* attempt to update each healthy LED */
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i] && _in_boot[i]) {
				/* flash the new firmware */
				bootloader_flash(i);
			}
		}

		/* boot each healthy LED */
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i] && _in_boot[i]) {
				/* boot the application */
				bootloader_boot(i);
			}
		}

		/* coerce LEDs with startup issues to be healthy again */
		bootloader_coerce_healthy();

		/* mandatory updating has finished */
		_alwaysupdate = false;

		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK, &_work, (worker_t)&OREOLED::cycle_trampoline, this,
			   USEC2TICK(OREOLED_UPDATE_INTERVAL_US));
		return;

	} else if (_autoupdate) {
		/* check booted oreoleds to see if the app can report it's checksum (release versions >= v1.2) */
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i] && !_in_boot[i]) {
				/* put any out of date oreoleds into bootloader mode */
				/* being in bootloader mode signals to be code below that the will likey need updating */
				if (bootloader_inapp_checksum(i) != bootloader_fw_checksum()) {
					bootloader_app_reset(i);
				}
			}
		}

		/* reset all healthy oreoleds if the number of outdated oreoled's is > 0 */
		/* this is done for consistency, so if only one oreoled is updating, all LEDs show the same behaviour */
		/* otherwise a single oreoled could appear broken or failed. */
		if (_num_inboot > 0) {
			for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
				if (_healthy[i] && !_in_boot[i]) {
					/* reset the LED if it's not in the bootloader */
					/* (this happens during a pixhawk OTA update, since the LEDs stay powered) */
					bootloader_app_reset(i);
				}
			}

			/* update each outdated and healthy LED in bootloader mode */
			for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
				if (_healthy[i] && _in_boot[i]) {
					/* only flash LEDs with an old version of the applictioon */
					if (bootloader_app_checksum(i) != bootloader_fw_checksum()) {
						/* flash the new firmware */
						bootloader_flash(i);
					}
				}
			}

			/* boot each healthy LED */
			for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
				if (_healthy[i] && _in_boot[i]) {
					/* boot the application */
					bootloader_boot(i);
				}
			}

			/* coerce LEDs with startup issues to be healthy again */
			bootloader_coerce_healthy();
		}

		/* auto updating has finished */
		_autoupdate = false;

		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK, &_work, (worker_t)&OREOLED::cycle_trampoline, this,
			   USEC2TICK(OREOLED_UPDATE_INTERVAL_US));
		return;

	} else if (_num_inboot > 0) {
		/* boot any LEDs which are in still in bootloader mode */
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_in_boot[i]) {
				bootloader_boot(i);
			}
		}

		/* coerce LEDs with startup issues to be healthy again */
		bootloader_coerce_healthy();

		/* ensure we don't get stuck in a loop */
		_num_inboot = 0;

		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK, &_work, (worker_t)&OREOLED::cycle_trampoline, this,
			   USEC2TICK(OREOLED_UPDATE_INTERVAL_US));
		return;

	} else if (!_is_ready) {
		/* indicate a ready state since startup has finished */
		_is_ready = true;
	}

	/* get next command from queue */
	oreoled_cmd_t next_cmd;

	while (_cmd_queue->get(&next_cmd, sizeof(oreoled_cmd_t))) {
		/* send valid messages to healthy LEDs */
		if ((next_cmd.led_num < OREOLED_NUM_LEDS) && _healthy[next_cmd.led_num]
		    && (next_cmd.num_bytes <= OREOLED_CMD_LENGTH_MAX)) {
			/* start performance timer */
			perf_begin(_call_perf);

			/* set I2C address */
			set_address(OREOLED_BASE_I2C_ADDR + next_cmd.led_num);

			/* Calculate XOR CRC and append to the i2c write data */
			uint8_t next_cmd_xor = OREOLED_BASE_I2C_ADDR + next_cmd.led_num;

			for (uint8_t i = 0; i < next_cmd.num_bytes; i++) {
				next_cmd_xor ^= next_cmd.buff[i];
			}

			next_cmd.buff[next_cmd.num_bytes++] = next_cmd_xor;

			/* send I2C command with a retry limit */
			for (uint8_t retry = OEROLED_COMMAND_RETRIES; retry > 0; retry--) {
				if (transfer(next_cmd.buff, next_cmd.num_bytes, reply, 3) == OK) {
					if (reply[1] == (OREOLED_BASE_I2C_ADDR + next_cmd.led_num) && reply[2] == next_cmd_xor) {
						/* slave returned a valid response */
						break;

					} else {
						perf_count(_reply_errors);
					}

				} else {
					perf_count(_comms_errors);
				}
			}

			perf_end(_call_perf);
		}
	}

	/* send general call every 4 seconds, if we aren't bootloading*/
	if (!_is_bootloading && ((now - _last_gencall) > OREOLED_GENERALCALL_US)) {
		perf_begin(_gcall_perf);
		send_general_call();
		perf_end(_gcall_perf);
	}

	/* schedule a fresh cycle call when the command is sent */
	work_queue(HPWORK, &_work, (worker_t)&OREOLED::cycle_trampoline, this,
		   USEC2TICK(OREOLED_UPDATE_INTERVAL_US));
}

int
OREOLED::bootloader_app_reset(int led_num)
{
	_is_bootloading = true;
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	int ret = -1;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	/* send a reset */
	boot_cmd.buff[0] = OREOLED_PATTERN_PARAMUPDATE;
	boot_cmd.buff[1] = OREOLED_PARAM_RESET;
	boot_cmd.buff[2] = OEROLED_RESET_NONCE;
	boot_cmd.buff[3] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 4;

	for (uint8_t j = 0; j < boot_cmd.num_bytes - 1; j++) {
		boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[j];
	}

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	/* send I2C command with a retry limit */
	for (uint8_t retry = OEROLED_COMMAND_RETRIES; retry > 0; retry--) {
		if (transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 3) == OK) {
			if (reply[1] == (OREOLED_BASE_I2C_ADDR + boot_cmd.led_num) &&
			    reply[2] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
				/* slave returned a valid response */
				ret = OK;
				/* set this LED as being in boot mode now */
				_in_boot[led_num] = true;
				_num_inboot++;
				break;
			}
		}
	}

	/* Allow time for the LED to reboot */
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);

	_is_bootloading = false;
	return ret;
}

int
OREOLED::bootloader_app_ping(int led_num)
{
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	int ret = -1;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	/* send a pattern off command */
	boot_cmd.buff[0] = 0xAA;
	boot_cmd.buff[1] = 0x55;
	boot_cmd.buff[2] = OREOLED_PATTERN_OFF;
	boot_cmd.buff[3] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 4;

	for (uint8_t j = 0; j < boot_cmd.num_bytes - 1; j++) {
		boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[j];
	}

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	/* send I2C command with a retry limit */
	for (uint8_t retry = OEROLED_COMMAND_RETRIES; retry > 0; retry--) {
		if (transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 3) == OK) {
			if (reply[1] == (OREOLED_BASE_I2C_ADDR + boot_cmd.led_num) &&
			    reply[2] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
				/* slave returned a valid response */
				ret = OK;
				break;
			}
		}
	}

	return ret;
}

uint16_t
OREOLED::bootloader_inapp_checksum(int led_num)
{
	_is_bootloading = true;
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	uint16_t ret = 0x0000;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_PATTERN_PARAMUPDATE;
	boot_cmd.buff[1] = OREOLED_PARAM_APP_CHECKSUM;
	boot_cmd.buff[2] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 3;

	for (uint8_t j = 0; j < boot_cmd.num_bytes - 1; j++) {
		boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[j];
	}

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 6);

		/* Check the response */
		if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
		    reply[2] == OREOLED_PARAM_APP_CHECKSUM &&
		    reply[5] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
			warnx("bl app checksum OK from LED %i", boot_cmd.led_num);
			warnx("bl app checksum msb: 0x%x", reply[3]);
			warnx("bl app checksum lsb: 0x%x", reply[4]);
			ret = ((reply[3] << 8) | reply[4]);
			break;

		} else {
			warnx("bl app checksum FAIL from LED %i", boot_cmd.led_num);
			warnx("bl app checksum response  ADDR: 0x%x", reply[1]);
			warnx("bl app checksum response   CMD: 0x%x", reply[2]);
			warnx("bl app checksum response VER H: 0x%x", reply[3]);
			warnx("bl app checksum response VER L: 0x%x", reply[4]);
			warnx("bl app checksum response   XOR: 0x%x", reply[5]);

			if (retry > 1) {
				warnx("bl app checksum retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl app checksum failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	_is_bootloading = false;
	return ret;
}

int
OREOLED::bootloader_ping(int led_num)
{
	_is_bootloading = true;
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	int ret = -1;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_PING;
	boot_cmd.buff[1] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 2;

	for (uint8_t j = 0; j < boot_cmd.num_bytes - 1; j++) {
		boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[j];
	}

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 5);

		/* Check the response */
		if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
		    reply[2] == OREOLED_BOOT_CMD_PING &&
		    reply[3] == OREOLED_BOOT_CMD_PING_NONCE &&
		    reply[4] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
			warnx("bl ping OK from LED %i", boot_cmd.led_num);
			ret = OK;
			break;

		} else {
			warnx("bl ping FAIL from LED %i", boot_cmd.led_num);
			warnx("bl ping response  ADDR: 0x%x", reply[1]);
			warnx("bl ping response   CMD: 0x%x", reply[2]);
			warnx("bl ping response NONCE: 0x%x", reply[3]);
			warnx("bl ping response   XOR: 0x%x", reply[4]);

			if (retry > 1) {
				warnx("bl ping retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl ping failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	_is_bootloading = false;
	return ret;
}

uint8_t
OREOLED::bootloader_version(int led_num)
{
	_is_bootloading = true;
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	uint8_t ret = 0x00;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_BL_VER;
	boot_cmd.buff[1] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 2;

	for (uint8_t k = 0; k < boot_cmd.num_bytes - 1; k++) {
		boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[k];
	}

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 5);

		/* Check the response */
		if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
		    reply[2] == OREOLED_BOOT_CMD_BL_VER &&
		    reply[3] == OREOLED_BOOT_SUPPORTED_VER &&
		    reply[4] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
			warnx("bl ver from LED %i = %i", boot_cmd.led_num, reply[3]);
			ret = reply[3];
			break;

		} else {
			warnx("bl ver response  ADDR: 0x%x", reply[1]);
			warnx("bl ver response   CMD: 0x%x", reply[2]);
			warnx("bl ver response   VER: 0x%x", reply[3]);
			warnx("bl ver response   XOR: 0x%x", reply[4]);

			if (retry > 1) {
				warnx("bl ver retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl ver failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	_is_bootloading = false;
	return ret;
}

uint16_t
OREOLED::bootloader_app_version(int led_num)
{
	_is_bootloading = true;
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	uint16_t ret = 0x0000;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_APP_VER;
	boot_cmd.buff[1] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 2;

	for (uint8_t j = 0; j < boot_cmd.num_bytes - 1; j++) {
		boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[j];
	}

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 6);

		/* Check the response */
		if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
		    reply[2] == OREOLED_BOOT_CMD_APP_VER &&
		    reply[5] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
			warnx("bl app version OK from LED %i", boot_cmd.led_num);
			warnx("bl app version msb: 0x%x", reply[3]);
			warnx("bl app version lsb: 0x%x", reply[4]);
			ret = ((reply[3] << 8) | reply[4]);
			break;

		} else {
			warnx("bl app version FAIL from LED %i", boot_cmd.led_num);
			warnx("bl app version response  ADDR: 0x%x", reply[1]);
			warnx("bl app version response   CMD: 0x%x", reply[2]);
			warnx("bl app version response VER H: 0x%x", reply[3]);
			warnx("bl app version response VER L: 0x%x", reply[4]);
			warnx("bl app version response   XOR: 0x%x", reply[5]);

			if (retry > 1) {
				warnx("bl app version retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl app version failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	_is_bootloading = false;
	return ret;
}

uint16_t
OREOLED::bootloader_app_checksum(int led_num)
{
	_is_bootloading = true;
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	uint16_t ret = 0x0000;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_APP_CRC;
	boot_cmd.buff[1] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 2;

	for (uint8_t j = 0; j < boot_cmd.num_bytes - 1; j++) {
		boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[j];
	}

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 6);

		/* Check the response */
		if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
		    reply[2] == OREOLED_BOOT_CMD_APP_CRC &&
		    reply[5] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
			warnx("bl app checksum OK from LED %i", boot_cmd.led_num);
			warnx("bl app checksum msb: 0x%x", reply[3]);
			warnx("bl app checksum lsb: 0x%x", reply[4]);
			ret = ((reply[3] << 8) | reply[4]);
			break;

		} else {
			warnx("bl app checksum FAIL from LED %i", boot_cmd.led_num);
			warnx("bl app checksum response  ADDR: 0x%x", reply[1]);
			warnx("bl app checksum response   CMD: 0x%x", reply[2]);
			warnx("bl app checksum response VER H: 0x%x", reply[3]);
			warnx("bl app checksum response VER L: 0x%x", reply[4]);
			warnx("bl app checksum response   XOR: 0x%x", reply[5]);

			if (retry > 1) {
				warnx("bl app checksum retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl app checksum failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	_is_bootloading = false;
	return ret;
}

int
OREOLED::bootloader_set_colour(int led_num, uint8_t red, uint8_t green)
{
	_is_bootloading = true;
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	int ret = -1;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_SET_COLOUR;
	boot_cmd.buff[1] = red;
	boot_cmd.buff[2] = green;
	boot_cmd.buff[3] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 4;

	for (uint8_t j = 0; j < boot_cmd.num_bytes - 1; j++) {
		boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[j];
	}

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write+Read */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 4);

		/* Check the response */
		if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
		    reply[2] == OREOLED_BOOT_CMD_SET_COLOUR &&
		    reply[3] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
			warnx("bl set colour OK from LED %i", boot_cmd.led_num);
			ret = OK;
			break;

		} else {
			warnx("bl set colour FAIL from LED %i", boot_cmd.led_num);
			warnx("bl set colour response  ADDR: 0x%x", reply[1]);
			warnx("bl set colour response   CMD: 0x%x", reply[2]);
			warnx("bl set colour response   XOR: 0x%x", reply[3]);

			if (retry > 1) {
				warnx("bl app colour retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl app colour failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	_is_bootloading = false;
	return ret;
}

int
OREOLED::bootloader_flash(int led_num)
{
	_is_bootloading = true;
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	/* Open the bootloader file */
	int fd = ::open(OREOLED_FW_FILE, O_RDONLY);

	/* check for error opening the file */
	if (fd < 0) {
		return -1;
	}

	struct stat s;

	/* attempt to stat the file */
	if (stat(OREOLED_FW_FILE, &s) != 0) {
		::close(fd);
		return -1;
	}

	uint16_t fw_length = s.st_size - OREOLED_FW_FILE_HEADER_LENGTH;

	/* sanity-check file size */
	if (fw_length > OREOLED_FW_FILE_SIZE_LIMIT) {
		::close(fd);
		return -1;
	}

	uint8_t *buf = new uint8_t[s.st_size];

	/* check that the buffer has been allocated */
	if (buf == NULL) {
		::close(fd);
		return -1;
	}

	/* check that the firmware can be read into the buffer */
	if (::read(fd, buf, s.st_size) != s.st_size) {
		::close(fd);
		delete[] buf;
		return -1;
	}

	::close(fd);

	/* Grab the version bytes from the binary */
	uint8_t version_major = buf[0];
	uint8_t version_minor = buf[1];

	/* calculate flash pages (rounded up to nearest integer) */
	uint8_t flash_pages = ((fw_length + 64 - 1) / 64);

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];

	/* Loop through flash pages */
	for (uint8_t page_idx = 0; page_idx < flash_pages; page_idx++) {

		/* Send the first half of the 64 byte flash page */
		memset(boot_cmd.buff, 0, sizeof(boot_cmd.buff));
		boot_cmd.buff[0] = OREOLED_BOOT_CMD_WRITE_FLASH_A;
		boot_cmd.buff[1] = page_idx;
		memcpy(boot_cmd.buff + 2, buf + (page_idx * 64) + OREOLED_FW_FILE_HEADER_LENGTH, 32);
		boot_cmd.buff[32 + 2] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
		boot_cmd.num_bytes = 32 + 3;

		for (uint8_t k = 0; k < boot_cmd.num_bytes - 1; k++) {
			boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[k];
		}

		for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
			/* Send the I2C Write+Read */
			memset(reply, 0, sizeof(reply));
			transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 4);

			/* Check the response */
			if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
			    reply[2] == OREOLED_BOOT_CMD_WRITE_FLASH_A &&
			    reply[3] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
				warnx("bl flash %ia OK for LED %i", page_idx, boot_cmd.led_num);
				break;

			} else {
				warnx("bl flash %ia FAIL for LED %i", page_idx, boot_cmd.led_num);
				warnx("bl flash %ia response ADDR: 0x%x", page_idx, reply[1]);
				warnx("bl flash %ia response  CMD: 0x%x", page_idx, reply[2]);
				warnx("bl flash %ia response  XOR: 0x%x", page_idx, reply[3]);

				if (retry > 1) {
					warnx("bl flash %ia retrying LED %i", page_idx, boot_cmd.led_num);

				} else {
					warnx("bl flash %ia failed on LED %i", page_idx, boot_cmd.led_num);
					delete[] buf;
					return -1;
				}
			}
		}

		/* Send the second half of the 64 byte flash page */
		memset(boot_cmd.buff, 0, sizeof(boot_cmd.buff));
		boot_cmd.buff[0] = OREOLED_BOOT_CMD_WRITE_FLASH_B;
		memcpy(boot_cmd.buff + 1, buf + (page_idx * 64) + 32 + OREOLED_FW_FILE_HEADER_LENGTH, 32);
		boot_cmd.buff[32 + 1] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
		boot_cmd.num_bytes = 32 + 2;

		for (uint8_t k = 0; k < boot_cmd.num_bytes - 1; k++) {
			boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[k];
		}

		for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
			/* Send the I2C Write+Read */
			memset(reply, 0, sizeof(reply));
			transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 4);

			/* Check the response */
			if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
			    reply[2] == OREOLED_BOOT_CMD_WRITE_FLASH_B &&
			    reply[3] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
				warnx("bl flash %ib OK for LED %i", page_idx, boot_cmd.led_num);
				break;

			} else {
				warnx("bl flash %ib FAIL for LED %i", page_idx, boot_cmd.led_num);
				warnx("bl flash %ib response ADDR: 0x%x", page_idx, reply[1]);
				warnx("bl flash %ib response  CMD: 0x%x", page_idx, reply[2]);
				warnx("bl flash %ib response  XOR: 0x%x", page_idx, reply[3]);

				if (retry > 1) {
					warnx("bl flash %ib retrying LED %i", page_idx, boot_cmd.led_num);

				} else {
					errx(1, "bl flash %ib failed on LED %i", page_idx, boot_cmd.led_num);
					delete[] buf;
					return -1;
				}
			}
		}

		/* Sleep to allow flash to write */
		/* Wait extra long on the first write, to allow time for EEPROM updates */
		if (page_idx == 0) {
			usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);

		} else {
			usleep(OREOLED_BOOT_FLASH_WAITMS * 1000);
		}
	}

	uint16_t app_checksum = bootloader_fw_checksum();

	/* Flash writes must have succeeded so finalise the flash */
	boot_cmd.buff[0] = OREOLED_BOOT_CMD_FINALISE_FLASH;
	boot_cmd.buff[1] = version_major;
	boot_cmd.buff[2] = version_minor;
	boot_cmd.buff[3] = (uint8_t)(fw_length >> 8);
	boot_cmd.buff[4] = (uint8_t)(fw_length & 0xFF);
	boot_cmd.buff[5] = (uint8_t)(app_checksum >> 8);
	boot_cmd.buff[6] = (uint8_t)(app_checksum & 0xFF);
	boot_cmd.buff[7] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 8;

	for (uint8_t k = 0; k < boot_cmd.num_bytes - 1; k++) {
		boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[k];
	}

	/* Try to finalise for twice the amount of normal retries */
	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES * 2; retry > 0; retry--) {
		/* Send the I2C Write */
		memset(reply, 0, sizeof(reply));
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 4);

		/* Check the response */
		if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
		    reply[2] == OREOLED_BOOT_CMD_FINALISE_FLASH &&
		    reply[3] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
			warnx("bl finalise OK from LED %i", boot_cmd.led_num);
			break;

		} else {
			warnx("bl finalise response  ADDR: 0x%x", reply[1]);
			warnx("bl finalise response   CMD: 0x%x", reply[2]);
			warnx("bl finalise response   XOR: 0x%x", reply[3]);

			if (retry > 1) {
				warnx("bl finalise retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl finalise failed on LED %i", boot_cmd.led_num);
				delete[] buf;
				return -1;
			}
		}
	}

	/* allow time for flash to finalise */
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);

	/* clean up file buffer */
	delete[] buf;

	_is_bootloading = false;
	return 1;
}

int
OREOLED::bootloader_boot(int led_num)
{
	_is_bootloading = true;
	oreoled_cmd_t boot_cmd;
	boot_cmd.led_num = led_num;

	int ret = -1;

	/* Set the current address */
	set_address(OREOLED_BASE_I2C_ADDR + boot_cmd.led_num);

	boot_cmd.buff[0] = OREOLED_BOOT_CMD_BOOT_APP;
	boot_cmd.buff[1] = OREOLED_BOOT_CMD_BOOT_NONCE;
	boot_cmd.buff[2] = OREOLED_BASE_I2C_ADDR + boot_cmd.led_num;
	boot_cmd.num_bytes = 3;

	for (uint8_t k = 0; k < boot_cmd.num_bytes - 1; k++) {
		boot_cmd.buff[boot_cmd.num_bytes - 1] ^= boot_cmd.buff[k];
	}

	for (uint8_t retry = OEROLED_BOOT_COMMAND_RETRIES; retry > 0; retry--) {
		/* Send the I2C Write */
		uint8_t reply[OREOLED_CMD_READ_LENGTH_MAX];
		transfer(boot_cmd.buff, boot_cmd.num_bytes, reply, 4);

		/* Check the response */
		if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
		    reply[2] == OREOLED_BOOT_CMD_BOOT_APP &&
		    reply[3] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
			warnx("bl boot OK from LED %i", boot_cmd.led_num);
			/* decrement the inboot counter so we don't get confused */
			_in_boot[led_num] = false;
			_num_inboot--;
			ret = OK;
			break;

		} else if (reply[1] == OREOLED_BASE_I2C_ADDR + boot_cmd.led_num &&
			   reply[2] == OREOLED_BOOT_CMD_BOOT_NONCE &&
			   reply[3] == boot_cmd.buff[boot_cmd.num_bytes - 1]) {
			warnx("bl boot error from LED %i: no app", boot_cmd.led_num);
			break;

		} else {
			warnx("bl boot response  ADDR: 0x%x", reply[1]);
			warnx("bl boot response   CMD: 0x%x", reply[2]);
			warnx("bl boot response   XOR: 0x%x", reply[3]);

			if (retry > 1) {
				warnx("bl boot retrying LED %i", boot_cmd.led_num);

			} else {
				warnx("bl boot failed on LED %i", boot_cmd.led_num);
				break;
			}
		}
	}

	/* allow time for the LEDs to boot */
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);
	usleep(OREOLED_BOOT_FLASH_WAITMS * 1000 * 10);

	_is_bootloading = false;
	return ret;
}

uint16_t
OREOLED::bootloader_fw_checksum(void)
{
	/* Calculate the 16 bit XOR checksum of the firmware on the first call of this function */
	if (_fw_checksum == 0x0000) {
		/* Open the bootloader file */
		int fd = ::open(OREOLED_FW_FILE, O_RDONLY);

		/* check for error opening the file */
		if (fd < 0) {
			return -1;
		}

		struct stat s;

		/* attempt to stat the file */
		if (stat(OREOLED_FW_FILE, &s) != 0) {
			::close(fd);
			return -1;
		}

		uint16_t fw_length = s.st_size - OREOLED_FW_FILE_HEADER_LENGTH;

		/* sanity-check file size */
		if (fw_length > OREOLED_FW_FILE_SIZE_LIMIT) {
			::close(fd);
			return -1;
		}

		uint8_t *buf = new uint8_t[s.st_size];

		/* check that the buffer has been allocated */
		if (buf == NULL) {
			::close(fd);
			return -1;
		}

		/* check that the firmware can be read into the buffer */
		if (::read(fd, buf, s.st_size) != s.st_size) {
			::close(fd);
			delete[] buf;
			return -1;
		}

		::close(fd);

		/* Calculate a 16 bit XOR checksum of the flash */
		/* Skip the first two bytes which are the version information, plus
		   the next two bytes which are modified by the bootloader */
		uint16_t app_checksum = 0x0000;

		for (uint16_t j = 2 + OREOLED_FW_FILE_HEADER_LENGTH; j < s.st_size; j += 2) {
			app_checksum ^= (buf[j] << 8) | buf[j + 1];
		}

		delete[] buf;

		warnx("fw length = %i", fw_length);
		warnx("fw checksum = %i", app_checksum);

		/* Store the checksum so it's only calculated once */
		_fw_checksum = app_checksum;
	}

	return _fw_checksum;
}

int
OREOLED::bootloader_coerce_healthy(void)
{
	int ret = -1;

	/* check each unhealthy LED */
	/* this re-checks "unhealthy" LEDs as they can sometimes power up with the wrong ID, */
	/*  but will have the correct ID once they jump to the application and be healthy again */
	for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
		if (!_healthy[i] && bootloader_app_ping(i) == OK) {
			/* mark as healthy */
			_healthy[i] = true;
			_num_healthy++;
			ret = OK;
		}
	}

	return ret;
}

int
OREOLED::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret = -ENODEV;
	oreoled_cmd_t new_cmd;

	switch (cmd) {
	case OREOLED_SET_RGB:
		/* set the specified color */
		new_cmd.led_num = ((oreoled_rgbset_t *) arg)->instance;
		new_cmd.buff[0] = ((oreoled_rgbset_t *) arg)->pattern;
		new_cmd.buff[1] = OREOLED_PARAM_BIAS_RED;
		new_cmd.buff[2] = ((oreoled_rgbset_t *) arg)->red;
		new_cmd.buff[3] = OREOLED_PARAM_BIAS_GREEN;
		new_cmd.buff[4] = ((oreoled_rgbset_t *) arg)->green;
		new_cmd.buff[5] = OREOLED_PARAM_BIAS_BLUE;
		new_cmd.buff[6] = ((oreoled_rgbset_t *) arg)->blue;
		new_cmd.num_bytes = 7;

		/* special handling for request to set all instances rgb values */
		if (new_cmd.led_num == OREOLED_ALL_INSTANCES) {
			for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
				/* add command to queue for all healthy leds */
				if (_healthy[i]) {
					new_cmd.led_num = i;
					_cmd_queue->force(&new_cmd);
					ret = OK;
				}
			}

		} else if (new_cmd.led_num < OREOLED_NUM_LEDS) {
			/* request to set individual instance's rgb value */
			if (_healthy[new_cmd.led_num]) {
				_cmd_queue->force(&new_cmd);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_RUN_MACRO:
		/* run a macro */
		new_cmd.led_num = ((oreoled_macrorun_t *) arg)->instance;
		new_cmd.buff[0] = OREOLED_PATTERN_PARAMUPDATE;
		new_cmd.buff[1] = OREOLED_PARAM_MACRO;
		new_cmd.buff[2] = ((oreoled_macrorun_t *) arg)->macro;
		new_cmd.num_bytes = 3;

		/* special handling for request to set all instances */
		if (new_cmd.led_num == OREOLED_ALL_INSTANCES) {
			for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
				/* add command to queue for all healthy leds */
				if (_healthy[i]) {
					new_cmd.led_num = i;
					_cmd_queue->force(&new_cmd);
					ret = OK;
				}
			}

		} else if (new_cmd.led_num < OREOLED_NUM_LEDS) {
			/* request to set individual instance's rgb value */
			if (_healthy[new_cmd.led_num]) {
				_cmd_queue->force(&new_cmd);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_SEND_RESET:
		/* send a reset */
		new_cmd.led_num = OREOLED_ALL_INSTANCES;
		new_cmd.buff[0] = OREOLED_PATTERN_PARAMUPDATE;
		new_cmd.buff[1] = OREOLED_PARAM_RESET;
		new_cmd.buff[2] = OEROLED_RESET_NONCE;
		new_cmd.num_bytes = 3;

		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			/* add command to queue for all healthy leds */
			if (_healthy[i]) {
				warnx("sending a reset... to %i", i);
				new_cmd.led_num = i;
				_cmd_queue->force(&new_cmd);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_BL_PING:
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				bootloader_ping(i);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_BL_VER:
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				bootloader_version(i);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_BL_FLASH:
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				bootloader_flash(i);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_BL_APP_VER:
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				bootloader_app_version(i);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_BL_APP_CRC:
		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				bootloader_app_checksum(i);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_BL_SET_COLOUR:
		new_cmd.led_num = OREOLED_ALL_INSTANCES;

		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				bootloader_set_colour(i, ((oreoled_rgbset_t *) arg)->red, ((oreoled_rgbset_t *) arg)->green);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_BL_BOOT_APP:
		new_cmd.led_num = OREOLED_ALL_INSTANCES;

		for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
			if (_healthy[i]) {
				bootloader_boot(i);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_SEND_BYTES:
		/* send bytes */
		new_cmd = *((oreoled_cmd_t *) arg);

		/* special handling for request to set all instances */
		if (new_cmd.led_num == OREOLED_ALL_INSTANCES) {
			for (uint8_t i = 0; i < OREOLED_NUM_LEDS; i++) {
				/* add command to queue for all healthy leds */
				if (_healthy[i]) {
					new_cmd.led_num = i;
					_cmd_queue->force(&new_cmd);
					ret = OK;
				}
			}

		} else if (new_cmd.led_num < OREOLED_NUM_LEDS) {
			/* request to set individual instance's rgb value */
			if (_healthy[new_cmd.led_num]) {
				_cmd_queue->force(&new_cmd);
				ret = OK;
			}
		}

		return ret;

	case OREOLED_FORCE_SYNC:
		send_general_call();
		break;

	default:
		/* see if the parent class can make any use of it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}

	return ret;
}

/* send general call on I2C bus to syncronise all LEDs */
int
OREOLED::send_general_call()
{
	int ret = -ENODEV;

	/* set I2C address to zero */
	set_address(0);

	/* prepare command : 0x01 = general hardware call, 0x00 = I2C address of master (but we don't act as a slave so set to zero)*/
	uint8_t msg[] = {0x01, 0x00};

	/* send I2C command */
	if (transfer(msg, sizeof(msg), nullptr, 0) == OK) {
		ret = OK;
	}

	/* record time */
	_last_gencall = hrt_absolute_time();

	return ret;
}

/* send a cmd to an LEDs (used for testing only) */
int
OREOLED::send_cmd(oreoled_cmd_t new_cmd)
{
	int ret = -ENODEV;

	/* sanity check led number, health and cmd length */
	if ((new_cmd.led_num < OREOLED_NUM_LEDS) && _healthy[new_cmd.led_num] && (new_cmd.num_bytes < OREOLED_CMD_LENGTH_MAX)) {
		/* set I2C address */
		set_address(OREOLED_BASE_I2C_ADDR + new_cmd.led_num);

		/* add to queue */
		_cmd_queue->force(&new_cmd);
		ret = OK;
	}

	return ret;
}

/* return the internal _is_ready flag indicating if initialisation is complete */
bool
OREOLED::is_ready()
{
	return _is_ready;
}

void
oreoled_usage()
{
	warnx("missing command: try 'start', 'test', 'info', 'off', 'stop', 'reset', 'rgb 30 40 50', 'macro 4', 'gencall', 'bytes <lednum> 7 9 6'");
	warnx("bootloader commands: try 'blping', 'blver', 'blappver', 'blappcrc', 'blcolour <red> <green>', 'blflash', 'blboot'");
	warnx("options:");
	warnx("    -b i2cbus (%d)", PX4_I2C_BUS_LED);
	warnx("    -a addr (0x%x)", OREOLED_BASE_I2C_ADDR);
}

int
oreoled_main(int argc, char *argv[])
{
	int i2cdevice = -1;
	int i2c_addr = OREOLED_BASE_I2C_ADDR; /* 7bit */

	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "a:b:")) != EOF) {
		switch (ch) {
		case 'a':
			i2c_addr = (int)strtol(optarg, NULL, 0);
			break;

		case 'b':
			i2cdevice = (int)strtol(optarg, NULL, 0);
			break;

		default:
			oreoled_usage();
			exit(0);
		}
	}

	if (optind >= argc) {
		oreoled_usage();
		exit(1);
	}

	const char *verb = argv[optind];

	int ret;

	/* start driver */
	if (!strcmp(verb, "start")) {
		if (g_oreoled != nullptr) {
			errx(1, "already started");
		}

		/* by default use LED bus */
		if (i2cdevice == -1) {
			i2cdevice = PX4_I2C_BUS_LED;
		}

		/* handle update flags */
		bool autoupdate = false;
		bool alwaysupdate = false;

		if (argc > 2 && !strcmp(argv[2], "autoupdate")) {
			warnx("autoupdate enabled");
			autoupdate = true;

		} else if (argc > 2 && !strcmp(argv[2], "alwaysupdate")) {
			warnx("alwaysupdate enabled");
			alwaysupdate = true;
		}

		/* instantiate driver */
		g_oreoled = new OREOLED(i2cdevice, i2c_addr, autoupdate, alwaysupdate);

		/* check if object was created */
		if (g_oreoled == nullptr) {
			errx(1, "failed to allocated memory for driver");
		}

		/* check object was created successfully */
		if (g_oreoled->init() != OK) {
			delete g_oreoled;
			g_oreoled = nullptr;
			errx(1, "failed to start driver");
		}

		/* wait for up to 20 seconds for the driver become ready */
		for (uint8_t i = 0; i < 20; i++) {
			if (g_oreoled != nullptr && g_oreoled->is_ready()) {
				break;
			}

			sleep(1);
		}

		exit(0);
	}

	/* need the driver past this point */
	if (g_oreoled == nullptr) {
		warnx("not started");
		oreoled_usage();
		exit(1);
	}

	if (!strcmp(verb, "test")) {
		int fd = open(OREOLED0_DEVICE_PATH, O_RDWR);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		/* structure to hold desired colour */
		oreoled_rgbset_t rgb_set_red = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, 0xFF, 0x0, 0x0};
		oreoled_rgbset_t rgb_set_blue = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, 0x0, 0x0, 0xFF};
		oreoled_rgbset_t rgb_set_off = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_OFF, 0x0, 0x0, 0x0};

		/* flash red and blue for 3 seconds */
		for (uint8_t i = 0; i < 30; i++) {
			/* red */
			if ((ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_red)) != OK) {
				errx(1, " failed to update rgb");
			}

			/* sleep for 0.05 seconds */
			usleep(50000);

			/* blue */
			if ((ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_blue)) != OK) {
				errx(1, " failed to update rgb");
			}

			/* sleep for 0.05 seconds */
			usleep(50000);
		}

		/* turn off LED */
		if ((ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_off)) != OK) {
			errx(1, " failed to turn off led");
		}

		close(fd);
		exit(ret);
	}

	/* display driver status */
	if (!strcmp(verb, "info")) {
		g_oreoled->info();
		exit(0);
	}

	if (!strcmp(verb, "off") || !strcmp(verb, "stop")) {
		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		/* turn off LED */
		oreoled_rgbset_t rgb_set_off = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_OFF, 0x0, 0x0, 0x0};
		ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set_off);

		close(fd);

		/* delete the oreoled object if stop was requested, in addition to turning off the LED. */
		if (!strcmp(verb, "stop")) {
			OREOLED *tmp_oreoled = g_oreoled;
			g_oreoled = nullptr;
			delete tmp_oreoled;
			exit(0);
		}

		exit(ret);
	}

	/* send rgb request to all LEDS */
	if (!strcmp(verb, "rgb")) {
		if (argc < 5) {
			errx(1, "Usage: oreoled rgb <red> <green> <blue>");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		uint8_t red = (uint8_t)strtol(argv[2], NULL, 0);
		uint8_t green = (uint8_t)strtol(argv[3], NULL, 0);
		uint8_t blue = (uint8_t)strtol(argv[4], NULL, 0);
		oreoled_rgbset_t rgb_set = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, red, green, blue};

		if ((ret = ioctl(fd, OREOLED_SET_RGB, (unsigned long)&rgb_set)) != OK) {
			errx(1, "failed to set rgb");
		}

		close(fd);
		exit(ret);
	}

	/* send macro request to all LEDS */
	if (!strcmp(verb, "macro")) {
		if (argc < 3) {
			errx(1, "Usage: oreoled macro <macro_num>");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		uint8_t macro = (uint8_t)strtol(argv[2], NULL, 0);

		/* sanity check macro number */
		if (macro > OREOLED_PARAM_MACRO_ENUM_COUNT) {
			errx(1, "invalid macro number %d", (int)macro);
			exit(ret);
		}

		oreoled_macrorun_t macro_run = {OREOLED_ALL_INSTANCES, (enum oreoled_macro)macro};

		if ((ret = ioctl(fd, OREOLED_RUN_MACRO, (unsigned long)&macro_run)) != OK) {
			errx(1, "failed to run macro");
		}

		close(fd);
		exit(ret);
	}

	/* send reset request to all LEDS */
	if (!strcmp(verb, "reset")) {
		if (argc < 2) {
			errx(1, "Usage: oreoled reset");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		if ((ret = ioctl(fd, OREOLED_SEND_RESET, 0)) != OK) {
			errx(1, "failed to run macro");
		}

		close(fd);
		exit(ret);
	}

	/* attempt to flash all LEDS in bootloader mode*/
	if (!strcmp(verb, "blflash")) {
		if (argc < 2) {
			errx(1, "Usage: oreoled blflash");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		if ((ret = ioctl(fd, OREOLED_BL_FLASH, 0)) != OK) {
			errx(1, "failed to run flash");
		}

		close(fd);
		exit(ret);
	}

	/* send bootloader boot request to all LEDS */
	if (!strcmp(verb, "blboot")) {
		if (argc < 2) {
			errx(1, "Usage: oreoled blboot");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		if ((ret = ioctl(fd, OREOLED_BL_BOOT_APP, 0)) != OK) {
			errx(1, "failed to run boot");
		}

		close(fd);
		exit(ret);
	}

	/* send bootloader ping all LEDs */
	if (!strcmp(verb, "blping")) {
		if (argc < 2) {
			errx(1, "Usage: oreoled blping");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		if ((ret = ioctl(fd, OREOLED_BL_PING, 0)) != OK) {
			errx(1, "failed to run blping");
		}

		close(fd);
		exit(ret);
	}

	/* ask all LEDs for their bootloader version */
	if (!strcmp(verb, "blver")) {
		if (argc < 2) {
			errx(1, "Usage: oreoled blver");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		if ((ret = ioctl(fd, OREOLED_BL_VER, 0)) != OK) {
			errx(1, "failed to get bootloader version");
		}

		close(fd);
		exit(ret);
	}

	/* ask all LEDs for their application version */
	if (!strcmp(verb, "blappver")) {
		if (argc < 2) {
			errx(1, "Usage: oreoled blappver");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		if ((ret = ioctl(fd, OREOLED_BL_APP_VER, 0)) != OK) {
			errx(1, "failed to get boot app version");
		}

		close(fd);
		exit(ret);
	}

	/* ask all LEDs for their application crc */
	if (!strcmp(verb, "blappcrc")) {
		if (argc < 2) {
			errx(1, "Usage: oreoled blappcrc");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		if ((ret = ioctl(fd, OREOLED_BL_APP_CRC, 0)) != OK) {
			errx(1, "failed to get boot app crc");
		}

		close(fd);
		exit(ret);
	}

	/* set the default bootloader LED colour on all LEDs */
	if (!strcmp(verb, "blcolour")) {
		if (argc < 4) {
			errx(1, "Usage: oreoled blcolour <red> <green>");
		}

		int fd = open(OREOLED0_DEVICE_PATH, 0);

		if (fd == -1) {
			errx(1, "Unable to open " OREOLED0_DEVICE_PATH);
		}

		uint8_t red = (uint8_t)strtol(argv[2], NULL, 0);
		uint8_t green = (uint8_t)strtol(argv[3], NULL, 0);
		oreoled_rgbset_t rgb_set = {OREOLED_ALL_INSTANCES, OREOLED_PATTERN_SOLID, red, green, 0};

		if ((ret = ioctl(fd, OREOLED_BL_SET_COLOUR, (unsigned long)&rgb_set)) != OK) {
			errx(1, "failed to set boot startup colours");
		}

		close(fd);
		exit(ret);
	}

	/* send general hardware call to all LEDS */
	if (!strcmp(verb, "gencall")) {
		ret = g_oreoled->send_general_call();
		warnx("sent general call");
		exit(ret);
	}

	/* send a string of bytes to an LED using send_bytes function */
	if (!strcmp(verb, "bytes")) {
		if (argc < 3) {
			errx(1, "Usage: oreoled bytes <led_num> <byte1> <byte2> <byte3> ...");
		}

		/* structure to be sent */
		oreoled_cmd_t sendb;

		/* maximum of 20 bytes can be sent */
		if (argc > 20 + 3) {
			errx(1, "Max of 20 bytes can be sent");
		}

		/* check led num */
		sendb.led_num = (uint8_t)strtol(argv[optind + 1], NULL, 0);

		if (sendb.led_num > 3) {
			errx(1, "led number must be between 0 ~ 3");
		}

		/* get bytes */
		sendb.num_bytes = argc - (optind + 2);
		uint8_t byte_count;

		for (byte_count = 0; byte_count < sendb.num_bytes; byte_count++) {
			sendb.buff[byte_count] = (uint8_t)strtol(argv[byte_count + optind + 2], NULL, 0);
		}

		/* send bytes */
		if ((ret = g_oreoled->send_cmd(sendb)) != OK) {
			errx(1, "failed to send command");

		} else {
			warnx("sent %d bytes", (int)sendb.num_bytes);
		}

		exit(ret);
	}

	oreoled_usage();
	exit(0);
}
