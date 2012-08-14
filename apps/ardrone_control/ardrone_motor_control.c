/****************************************************************************
 *
 *   Copyright (C) 2008-2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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

/*
 * @file ardrone_motor_control.c
 * Implementation of AR.Drone 1.0 / 2.0 motor control interface
 */



#include "ardrone_motor_control.h"

static const unsigned long motor_gpios = GPIO_EXT_1 | GPIO_EXT_2 | GPIO_MULTI_1 | GPIO_MULTI_2;
static const unsigned long motor_gpio[4] = { GPIO_EXT_1, GPIO_EXT_2, GPIO_MULTI_1, GPIO_MULTI_2 };

typedef union {
	uint16_t motor_value;
	uint8_t bytes[2];
} motor_union_t;

/**
 * @brief Generate the 8-byte motor set packet
 *
 * @return the number of bytes (8)
 */
void ar_get_motor_packet(uint8_t *motor_buf, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4)
{
	motor_buf[0] = 0x20;
	motor_buf[1] = 0x00;
	motor_buf[2] = 0x00;
	motor_buf[3] = 0x00;
	motor_buf[4] = 0x00;
	/*
	 * {0x20, 0x00, 0x00, 0x00, 0x00};
	 * 0x20 is start sign / motor command
	 */
	motor_union_t curr_motor;
	uint16_t nineBitMask = 0x1FF;

	/* Set motor 1 */
	curr_motor.motor_value = (motor1 & nineBitMask) << 4;
	motor_buf[0] |= curr_motor.bytes[1];
	motor_buf[1] |= curr_motor.bytes[0];

	/* Set motor 2 */
	curr_motor.motor_value = (motor2 & nineBitMask) << 3;
	motor_buf[1] |= curr_motor.bytes[1];
	motor_buf[2] |= curr_motor.bytes[0];

	/* Set motor 3 */
	curr_motor.motor_value = (motor3 & nineBitMask) << 2;
	motor_buf[2] |= curr_motor.bytes[1];
	motor_buf[3] |= curr_motor.bytes[0];

	/* Set motor 4 */
	curr_motor.motor_value = (motor4 & nineBitMask) << 1;
	motor_buf[3] |= curr_motor.bytes[1];
	motor_buf[4] |= curr_motor.bytes[0];
}

void ar_enable_broadcast(int fd)
{
	ar_select_motor(fd, 0);
}

int ar_multiplexing_init()
{
	int		fd;
	
	fd = open(GPIO_DEVICE_PATH, 0);

	if (fd < 0) {
		printf("GPIO: open fail\n");
		return fd;
	}

	/* deactivate all outputs */
	int ret = 0;
	ret += ioctl(fd, GPIO_SET, motor_gpios);

	if (ioctl(fd, GPIO_SET_OUTPUT, motor_gpios) != 0) {
		printf("GPIO: output set fail\n");
		close(fd);
		return -1;
	}

	if (ret < 0) {
		printf("GPIO: clearing pins fail\n");
		close(fd);
		return -1;
	}

	return fd;
}

int ar_multiplexing_deinit(int fd)
{
	if (fd < 0) {
		printf("GPIO: no valid descriptor\n");
		return fd;
	}

	int ret = 0;

	/* deselect motor 1-4 */
	ret += ioctl(fd, GPIO_SET, motor_gpios);

	if (ret != 0) {
		printf("GPIO: clear failed %d times\n", ret);
	}

	if (ioctl(fd, GPIO_SET_INPUT, motor_gpios) != 0) {
		printf("GPIO: input set fail\n");
		return -1;
	}

	close(fd);

	return ret;
}

int ar_select_motor(int fd, uint8_t motor)
{
	int ret = 0;
	unsigned long gpioset;
	/*
	 *  Four GPIOS:
	 *		GPIO_EXT1
	 *		GPIO_EXT2
	 *		GPIO_UART2_CTS
	 *		GPIO_UART2_RTS
	 */

	/* select motor 0 to enable broadcast */
	if (motor == 0) {
		/* select motor 1-4 */
		ret += ioctl(fd, GPIO_CLEAR, motor_gpios);

	} else {
		/* deselect all */
		ret += ioctl(fd, GPIO_SET, motor_gpios);

		/* select reqested motor */	
		ret += ioctl(fd, GPIO_CLEAR, motor_gpio[motor - 1]);

		/* deselect all others */
		// gpioset = motor_gpios ^ motor_gpio[motor - 1];
		// ret += ioctl(fd, GPIO_SET, gpioset);
	}

	return ret;
}

int ar_init_motors(int ardrone_uart, int *gpios_pin)
{
	/* Initialize multiplexing */
	*gpios_pin = ar_multiplexing_init();

	/* Write ARDrone commands on UART2 */
	uint8_t initbuf[] = {0xE0, 0x91, 0xA1, 0x00, 0x40};
	uint8_t multicastbuf[] = {0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0};

	/* initialize all motors
	 * - select one motor at a time
	 * - configure motor
	 */
	int i;
	int errcounter = 0;

	for (i = 1; i < 5; ++i) {
		/* Initialize motors 1-4 */
		initbuf[3] = i;
		errcounter += ar_select_motor(*gpios_pin, i);

		write(ardrone_uart, initbuf + 0, 1);

		/* sleep 400 ms */
		usleep(200000);
		usleep(200000);

		write(ardrone_uart, initbuf + 1, 1);
		/* wait 50 ms */
		usleep(50000);

		write(ardrone_uart, initbuf + 2, 1);
		/* wait 50 ms */
		usleep(50000);

		write(ardrone_uart, initbuf + 3, 1);
		/* wait 50 ms */
		usleep(50000);

		write(ardrone_uart, initbuf + 4, 1);
		/* wait 50 ms */
		usleep(50000);

		/* enable multicast */
		write(ardrone_uart, multicastbuf + 0, 1);
		/* wait 1 ms */
		usleep(1000);

		write(ardrone_uart, multicastbuf + 1, 1);
		/* wait 1 ms */
		usleep(1000);

		write(ardrone_uart, multicastbuf + 2, 1);
		/* wait 1 ms */
		usleep(1000);

		write(ardrone_uart, multicastbuf + 3, 1);
		/* wait 1 ms */
		usleep(1000);

		write(ardrone_uart, multicastbuf + 4, 1);
		/* wait 1 ms */
		usleep(1000);

		write(ardrone_uart, multicastbuf + 5, 1);
		/* wait 5 ms */
		usleep(50000);
	}

	/* start the multicast part */
	errcounter += ar_select_motor(*gpios_pin, 0);

	if (errcounter != 0) {
		fprintf(stderr, "[ar motors] init sequence incomplete, failed %d times", -errcounter);
		fflush(stdout);
	}
	return errcounter;
}

/*
 * Sets the leds on the motor controllers, 1 turns led on, 0 off.
 */
void ar_set_leds(int ardrone_uart, uint8_t led1_red, uint8_t led1_green, uint8_t led2_red, uint8_t led2_green, uint8_t led3_red, uint8_t led3_green, uint8_t led4_red, uint8_t led4_green)
{
	/*
	 * 2 bytes are sent. The first 3 bits describe the command: 011 means led control
	 * the following 4 bits are the red leds for motor 4, 3, 2, 1
	 * then 4 bits with unknown function, then 4 bits for green leds for motor 4, 3, 2, 1
	 * the last bit is unknown.
	 *
	 * The packet is therefore:
	 * 011 rrrr 0000 gggg 0
	 */
	uint8_t leds[2];
	leds[0] = 0x60 | ((led4_red & 0x01) << 4) | ((led3_red & 0x01) << 3) | ((led2_red & 0x01) << 2) | ((led1_red & 0x01) << 1);
	leds[1] = ((led4_green & 0x01) << 4) | ((led3_green & 0x01) << 3) | ((led2_green & 0x01) << 2) | ((led1_green & 0x01) << 1);
	write(ardrone_uart, leds, 2);
}

int ardrone_write_motor_commands(int ardrone_fd, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4) {
	const int min_motor_interval = 20000;
	static uint64_t last_motor_time = 0;
	if (hrt_absolute_time() - last_motor_time > min_motor_interval) {
		uint8_t buf[5] = {0};
		ar_get_motor_packet(buf, motor1, motor2, motor3, motor4);
		int ret;
		if ((ret = write(ardrone_fd, buf, sizeof(buf))) > 0) {
			return OK;
		} else {
			return ret;
		}
	} else {
		return -ERROR;
	}
}
