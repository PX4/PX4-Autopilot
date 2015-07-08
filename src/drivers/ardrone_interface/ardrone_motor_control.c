/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/**
 * @file ardrone_motor_control.c
 * Implementation of AR.Drone 1.0 / 2.0 motor control interface
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <systemlib/err.h>

#include "ardrone_motor_control.h"

static unsigned long motor_gpios = GPIO_EXT_1 | GPIO_EXT_2 | GPIO_MULTI_1 | GPIO_MULTI_2;
static unsigned long motor_gpio[4] = { GPIO_EXT_1, GPIO_EXT_2, GPIO_MULTI_1, GPIO_MULTI_2 };

typedef union {
	uint16_t motor_value;
	uint8_t bytes[2];
} motor_union_t;

#define UART_TRANSFER_TIME_BYTE_US (9+50) /**< 9 us per byte at 115200k plus overhead */

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
	
	fd = open(PX4FMU_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("GPIO: open fail");
		return fd;
	}

	/* deactivate all outputs */
	if (ioctl(fd, GPIO_SET, motor_gpios)) {
		warn("GPIO: clearing pins fail");
		close(fd);
		return -1;
	}

	/* configure all motor select GPIOs as outputs */
	if (ioctl(fd, GPIO_SET_OUTPUT, motor_gpios) != 0) {
		warn("GPIO: output set fail");
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
		/* select reqested motor */	
		ret += ioctl(fd, GPIO_CLEAR, motor_gpio[motor - 1]);
	}

	return ret;
}

int ar_deselect_motor(int fd, uint8_t motor)
{
	int ret = 0;
	/*
	 *  Four GPIOS:
	 *		GPIO_EXT1
	 *		GPIO_EXT2
	 *		GPIO_UART2_CTS
	 *		GPIO_UART2_RTS
	 */

	if (motor == 0) {
		/* deselect motor 1-4 */
		ret += ioctl(fd, GPIO_SET, motor_gpios);

	} else {
		/* deselect reqested motor */	
		ret = ioctl(fd, GPIO_SET, motor_gpio[motor - 1]);
	}

	return ret;
}

int ar_init_motors(int ardrone_uart, int gpios)
{
	/* Write ARDrone commands on UART2 */
	uint8_t initbuf[] = {0xE0, 0x91, 0xA1, 0x00, 0x40};
	uint8_t multicastbuf[] = {0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0};

	/* deselect all motors */
	ar_deselect_motor(gpios, 0);

	/* initialize all motors
	 * - select one motor at a time
	 * - configure motor
	 */
	int i;
	int errcounter = 0;


	/* initial setup run */
	for (i = 1; i < 5; ++i) {
		/* Initialize motors 1-4 */
		errcounter += ar_select_motor(gpios, i);
		usleep(200);

		/*
		 * write 0xE0 - request status
		 * receive one status byte
		 */
		write(ardrone_uart, &(initbuf[0]), 1);
		fsync(ardrone_uart);
		usleep(UART_TRANSFER_TIME_BYTE_US*1);

		/*
		 * write 0x91 - request checksum
		 * receive 120 status bytes
		 */
		write(ardrone_uart, &(initbuf[1]), 1);
		fsync(ardrone_uart);
		usleep(UART_TRANSFER_TIME_BYTE_US*120);

		/*
		 * write 0xA1 - set status OK
		 * receive one status byte - should be A0
		 * to confirm status is OK
		 */
		write(ardrone_uart, &(initbuf[2]), 1);
		fsync(ardrone_uart);
		usleep(UART_TRANSFER_TIME_BYTE_US*1);

		/*
		 * set as motor i, where i = 1..4
		 * receive nothing
		 */
		initbuf[3] = (uint8_t)i;
		write(ardrone_uart, &(initbuf[3]), 1);
		fsync(ardrone_uart);

		/*
		 * write 0x40 - check version
		 * receive 11 bytes encoding the version
		 */
		write(ardrone_uart, &(initbuf[4]), 1);
		fsync(ardrone_uart);
		usleep(UART_TRANSFER_TIME_BYTE_US*11);

		ar_deselect_motor(gpios, i);
		/* sleep 200 ms */
		usleep(200000);
	}

	/* start the multicast part */
	errcounter += ar_select_motor(gpios, 0);
	usleep(200);

	/*
	 * first round
	 * write six times A0 - enable broadcast
	 * receive nothing
	 */
	write(ardrone_uart, multicastbuf, sizeof(multicastbuf));
	fsync(ardrone_uart);
	usleep(UART_TRANSFER_TIME_BYTE_US * sizeof(multicastbuf));

	/*
	 * second round
	 * write six times A0 - enable broadcast
	 * receive nothing
	 */
	write(ardrone_uart, multicastbuf, sizeof(multicastbuf));
	fsync(ardrone_uart);
	usleep(UART_TRANSFER_TIME_BYTE_US * sizeof(multicastbuf));

	/* set motors to zero speed (fsync is part of the write command */
	ardrone_write_motor_commands(ardrone_uart, 0, 0, 0, 0);

	if (errcounter != 0) {
		warnx("Failed %d times", -errcounter);
		fflush(stdout);
	}
	return errcounter;
}

/**
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
	const unsigned int min_motor_interval = 4900;
	static uint64_t last_motor_time = 0;

	static struct actuator_outputs_s outputs;
	outputs.timestamp = hrt_absolute_time();
	outputs.output[0] = motor1;
	outputs.output[1] = motor2;
	outputs.output[2] = motor3;
	outputs.output[3] = motor4;
	static orb_advert_t pub = 0;
	if (pub == 0) {
		/* advertise to channel 0 / primary */
		pub = orb_advertise(ORB_ID(actuator_outputs), &outputs);
	}

	if (hrt_absolute_time() - last_motor_time > min_motor_interval) {
		uint8_t buf[5] = {0};
		ar_get_motor_packet(buf, motor1, motor2, motor3, motor4);
		int ret;
		ret = write(ardrone_fd, buf, sizeof(buf));
		fsync(ardrone_fd);

		/* publish just written values */
		orb_publish(ORB_ID(actuator_outputs), pub, &outputs);

		if (ret == sizeof(buf)) {
			return OK;
		} else {
			return ret;
		}
	} else {
		return -ERROR;
	}
}

void ardrone_mixing_and_output(int ardrone_write, const struct actuator_controls_s *actuators) {

	float roll_control = actuators->control[0];
	float pitch_control = actuators->control[1];
	float yaw_control = actuators->control[2];
	float motor_thrust = actuators->control[3];

	const float min_thrust = 0.02f;			/**< 2% minimum thrust */
	const float max_thrust = 1.0f;			/**< 100% max thrust */
	const float scaling = 510.0f;			/**< 100% thrust equals a value of 510 which works, 512 leads to cutoff */
	const float min_gas = min_thrust * scaling;	/**< value range sent to motors, minimum */
	const float max_gas = max_thrust * scaling;	/**< value range sent to motors, maximum */

	/* initialize all fields to zero */
	uint16_t motor_pwm[4] = {0};
	float motor_calc[4] = {0};

	float output_band = 0.0f;
	const float startpoint_full_control = 0.25f;	/**< start full control at 25% thrust */

	/* linearly scale the control inputs from 0 to startpoint_full_control */
	if (motor_thrust < startpoint_full_control) {
		output_band = motor_thrust/startpoint_full_control; // linear from 0 to 1
	} else {
		output_band = 1.0f;
	}

	roll_control *= output_band;
	pitch_control *= output_band;
	yaw_control *= output_band;


	//add the yaw, nick and roll components to the basic thrust //TODO:this should be done by the mixer

	// FRONT (MOTOR 1)
	motor_calc[0] = motor_thrust + (roll_control / 2 + pitch_control / 2 - yaw_control);
	// RIGHT (MOTOR 2)
	motor_calc[1] = motor_thrust + (-roll_control / 2 + pitch_control / 2 + yaw_control);
	// BACK (MOTOR 3)
	motor_calc[2] = motor_thrust + (-roll_control / 2 - pitch_control / 2 - yaw_control);
	// LEFT (MOTOR 4)
	motor_calc[3] = motor_thrust + (roll_control / 2 - pitch_control / 2 + yaw_control);

	/* if one motor is saturated, reduce throttle */
	float saturation = fmaxf(fmaxf(motor_calc[0], motor_calc[1]),fmaxf(motor_calc[2], motor_calc[3])) - max_thrust;


	if (saturation > 0.0f) {

		/* reduce the motor thrust according to the saturation */
		motor_thrust = motor_thrust - saturation;

		// FRONT (MOTOR 1)
		motor_calc[0] = motor_thrust + (roll_control / 2 + pitch_control / 2 - yaw_control);
		// RIGHT (MOTOR 2)
		motor_calc[1] = motor_thrust + (-roll_control / 2 + pitch_control / 2 + yaw_control);
		// BACK (MOTOR 3)
		motor_calc[2] = motor_thrust + (-roll_control / 2 - pitch_control / 2 - yaw_control);
		// LEFT (MOTOR 4)
		motor_calc[3] = motor_thrust + (roll_control / 2 - pitch_control / 2 + yaw_control);
	}

	/* set the motor values */

	/* scale up from 0..1 to 10..500) */
	motor_pwm[0] = (uint16_t) (motor_calc[0] * ((float)max_gas - min_gas) + min_gas);
	motor_pwm[1] = (uint16_t) (motor_calc[1] * ((float)max_gas - min_gas) + min_gas);
	motor_pwm[2] = (uint16_t) (motor_calc[2] * ((float)max_gas - min_gas) + min_gas);
	motor_pwm[3] = (uint16_t) (motor_calc[3] * ((float)max_gas - min_gas) + min_gas);

	/* scale up from 0..1 to 10..500) */
	motor_pwm[0] = (uint16_t) (motor_calc[0] * (float)((max_gas - min_gas) + min_gas));
	motor_pwm[1] = (uint16_t) (motor_calc[1] * (float)((max_gas - min_gas) + min_gas));
	motor_pwm[2] = (uint16_t) (motor_calc[2] * (float)((max_gas - min_gas) + min_gas));
	motor_pwm[3] = (uint16_t) (motor_calc[3] * (float)((max_gas - min_gas) + min_gas));

	/* Failsafe logic for min values - should never be necessary */
	motor_pwm[0] = (motor_pwm[0] > 0) ? motor_pwm[0] : min_gas;
	motor_pwm[1] = (motor_pwm[1] > 0) ? motor_pwm[1] : min_gas;
	motor_pwm[2] = (motor_pwm[2] > 0) ? motor_pwm[2] : min_gas;
	motor_pwm[3] = (motor_pwm[3] > 0) ? motor_pwm[3] : min_gas;

	/* Failsafe logic for max values - should never be necessary */
	motor_pwm[0] = (motor_pwm[0] <= max_gas) ? motor_pwm[0] : max_gas;
	motor_pwm[1] = (motor_pwm[1] <= max_gas) ? motor_pwm[1] : max_gas;
	motor_pwm[2] = (motor_pwm[2] <= max_gas) ? motor_pwm[2] : max_gas;
	motor_pwm[3] = (motor_pwm[3] <= max_gas) ? motor_pwm[3] : max_gas;

	/* send motors via UART */
	ardrone_write_motor_commands(ardrone_write, motor_pwm[0], motor_pwm[1], motor_pwm[2], motor_pwm[3]);
}
