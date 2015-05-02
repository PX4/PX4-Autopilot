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
 * @file ardrone_motor_control.h
 * Definition of AR.Drone 1.0 / 2.0 motor control interface
 */

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>

/**
 * Generate the 5-byte motor set packet.
 *
 * @return the number of bytes (5)
 */
void ar_get_motor_packet(uint8_t *motor_buf, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

/**
 * Select a motor in the multiplexing.
 *
 * @param fd GPIO file descriptor
 * @param motor Motor number, from 1 to 4, 0 selects all
 */
int ar_select_motor(int fd, uint8_t motor);

/**
 * Deselect a motor in the multiplexing.
 *
 * @param fd GPIO file descriptor
 * @param motor Motor number, from 1 to 4, 0 deselects all
 */
int ar_deselect_motor(int fd, uint8_t motor);

void ar_enable_broadcast(int fd);

int ar_multiplexing_init(void);
int ar_multiplexing_deinit(int fd);

/**
 * Write four motor commands to an already initialized port.
 *
 * Writing 0 stops a motor, values from 1-512 encode the full thrust range.
 * on some motor controller firmware revisions a minimum value of 10 is
 * required to spin the motors.
 */
int ardrone_write_motor_commands(int ardrone_fd, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

/**
 * Initialize the motors.
 */
int ar_init_motors(int ardrone_uart, int gpio);

/**
 * Set LED pattern.
 */
void ar_set_leds(int ardrone_uart, uint8_t led1_red, uint8_t led1_green, uint8_t led2_red, uint8_t led2_green,
		 uint8_t led3_red, uint8_t led3_green, uint8_t led4_red, uint8_t led4_green);

/**
 * Mix motors and output actuators
 */
void ardrone_mixing_and_output(int ardrone_write, const struct actuator_controls_s *actuators);
